// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU crash dump formatter
//!
//! Takes a raw dump of firmware/kernel mapped pages from `pgtable` and formats it into
//! an ELF core dump suitable for dumping into userspace.

use core::mem::size_of;

use kernel::{error::Result, page::Page, prelude::*, types::Owned};

use crate::hw;
use crate::pgtable::{self, DumpedPage, Prot, UAT_PGSZ};
use crate::util::align;
use kernel::uapi;

pub(crate) struct CrashDump {
    headers: KVVec<u8>,
    pages: KVVec<Owned<Page>>,
}

const NOTE_NAME_AGX: &str = &"AGX";
const NOTE_AGX_DUMP_INFO: u32 = 1;

const NOTE_NAME_RTKIT: &str = &"RTKIT";
const NOTE_RTKIT_CRASHLOG: u32 = 1;

#[repr(C)]
pub(crate) struct AGXDumpInfo {
    initdata_address: u64,
    chip_id: u32,
    gpu_gen: hw::GpuGen,
    gpu_variant: hw::GpuVariant,
    gpu_rev: hw::GpuRevision,
    total_active_cores: u32,
    firmware_version: [u32; 6],
}

struct ELFNote {
    name: &'static str,
    ty: u32,
    data: KVVec<u8>,
}

pub(crate) struct CrashDumpBuilder {
    page_dump: KVVec<DumpedPage>,
    notes: KVec<ELFNote>,
}

// Helper to convert ELF headers into byte slices
// TODO: Hook this up into kernel::AsBytes somehow
unsafe trait AsBytes: Sized {
    fn as_bytes(&self) -> &[u8] {
        // SAFETY: This trait is only implemented for types with no padding bytes
        unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, size_of::<Self>()) }
    }
    fn slice_as_bytes(slice: &[Self]) -> &[u8] {
        // SAFETY: This trait is only implemented for types with no padding bytes
        unsafe {
            core::slice::from_raw_parts(
                slice.as_ptr() as *const u8,
                slice.len() * size_of::<Self>(),
            )
        }
    }
}

// SAFETY: This type has no padding
unsafe impl AsBytes for uapi::Elf64_Ehdr {}
// SAFETY: This type has no padding
unsafe impl AsBytes for uapi::Elf64_Phdr {}
// SAFETY: This type has no padding
unsafe impl AsBytes for uapi::Elf64_Nhdr {}
// SAFETY: This type has no padding
unsafe impl AsBytes for AGXDumpInfo {}

const FIRMWARE_ENTRYPOINT: u64 = 0xFFFFFF8000000000u64;

impl CrashDumpBuilder {
    pub(crate) fn new(page_dump: KVVec<DumpedPage>) -> Result<CrashDumpBuilder> {
        Ok(CrashDumpBuilder {
            page_dump,
            notes: KVec::new(),
        })
    }

    pub(crate) fn add_agx_info(
        &mut self,
        cfg: &hw::HwConfig,
        dyncfg: &hw::DynConfig,
        initdata_address: u64,
    ) -> Result {
        let mut info = AGXDumpInfo {
            chip_id: cfg.chip_id,
            gpu_gen: dyncfg.id.gpu_gen,
            gpu_variant: dyncfg.id.gpu_variant,
            gpu_rev: dyncfg.id.gpu_rev,
            total_active_cores: dyncfg.id.total_active_cores,
            firmware_version: [0; 6],
            initdata_address,
        };
        info.firmware_version[..dyncfg.firmware_version.len().min(6)]
            .copy_from_slice(&dyncfg.firmware_version);

        let mut data = KVVec::new();
        data.extend_from_slice(info.as_bytes(), GFP_KERNEL)?;

        self.notes.push(
            ELFNote {
                name: NOTE_NAME_AGX,
                ty: NOTE_AGX_DUMP_INFO,
                data,
            },
            GFP_KERNEL,
        )?;
        Ok(())
    }

    pub(crate) fn add_crashlog(&mut self, crashlog: &[u8]) -> Result {
        let mut data = KVVec::new();
        data.extend_from_slice(&crashlog, GFP_KERNEL)?;

        self.notes.push(
            ELFNote {
                name: NOTE_NAME_RTKIT,
                ty: NOTE_RTKIT_CRASHLOG,
                data,
            },
            GFP_KERNEL,
        )?;

        Ok(())
    }

    pub(crate) fn finalize(self) -> Result<CrashDump> {
        let CrashDumpBuilder { page_dump, notes } = self;

        let mut ehdr: uapi::Elf64_Ehdr = Default::default();

        ehdr.e_ident[uapi::EI_MAG0 as usize..=uapi::EI_MAG3 as usize].copy_from_slice(b"\x7fELF");
        ehdr.e_ident[uapi::EI_CLASS as usize] = uapi::ELFCLASS64 as u8;
        ehdr.e_ident[uapi::EI_DATA as usize] = uapi::ELFDATA2LSB as u8;
        ehdr.e_ident[uapi::EI_VERSION as usize] = uapi::EV_CURRENT as u8;
        ehdr.e_type = uapi::ET_CORE as u16;
        ehdr.e_machine = uapi::EM_AARCH64 as u16;
        ehdr.e_version = uapi::EV_CURRENT as u32;
        ehdr.e_entry = FIRMWARE_ENTRYPOINT;
        ehdr.e_ehsize = core::mem::size_of::<uapi::Elf64_Ehdr>() as u16;
        ehdr.e_phentsize = core::mem::size_of::<uapi::Elf64_Phdr>() as u16;

        let phdr_offset = core::mem::size_of::<uapi::Elf64_Ehdr>();

        // PHDRs come after the ELF header
        ehdr.e_phoff = phdr_offset as u64;

        let mut phdrs = KVVec::new();

        // First PHDR is the NOTE section
        phdrs.push(
            uapi::Elf64_Phdr {
                p_type: uapi::PT_NOTE,
                p_flags: uapi::PF_R,
                p_align: 1,
                ..Default::default()
            },
            GFP_KERNEL,
        )?;

        // Generate the page phdrs. The offset will be fixed up later.
        let mut off: usize = 0;
        let mut next = None;
        let mut pages: KVVec<Owned<Page>> = KVVec::new();

        for mut page in page_dump {
            let vaddr = page.iova;
            let paddr = page.pte & pgtable::PTE_ADDR_BITS;
            let flags = Prot::from_pte(page.pte).elf_flags();
            let valid = page.data.is_some();
            let cur = (vaddr, paddr, flags, valid);
            if Some(cur) != next {
                phdrs.push(
                    uapi::Elf64_Phdr {
                        p_type: uapi::PT_LOAD,
                        p_offset: if valid { off as u64 } else { 0 },
                        p_vaddr: vaddr,
                        p_paddr: paddr,
                        p_filesz: if valid { UAT_PGSZ as u64 } else { 0 },
                        p_memsz: UAT_PGSZ as u64,
                        p_flags: flags,
                        p_align: UAT_PGSZ as u64,
                        ..Default::default()
                    },
                    GFP_KERNEL,
                )?;
                if valid {
                    off += UAT_PGSZ;
                }
            } else {
                let ph = phdrs.last_mut().unwrap();
                ph.p_memsz += UAT_PGSZ as u64;
                if valid {
                    ph.p_filesz += UAT_PGSZ as u64;
                    off += UAT_PGSZ;
                }
            }
            if let Some(data_page) = page.data.take() {
                pages.push(data_page, GFP_KERNEL)?;
            }
            next = Some((
                vaddr + UAT_PGSZ as u64,
                paddr + UAT_PGSZ as u64,
                flags,
                valid,
            ));
        }

        ehdr.e_phnum = phdrs.len() as u16;

        let note_offset = phdr_offset + size_of::<uapi::Elf64_Phdr>() * phdrs.len();

        let mut note_data: KVVec<u8> = KVVec::new();

        for note in notes {
            let hdr = uapi::Elf64_Nhdr {
                n_namesz: note.name.len() as u32 + 1,
                n_descsz: note.data.len() as u32,
                n_type: note.ty,
            };
            note_data.extend_from_slice(hdr.as_bytes(), GFP_KERNEL)?;
            note_data.extend_from_slice(note.name.as_bytes(), GFP_KERNEL)?;
            note_data.push(0, GFP_KERNEL)?;
            while note_data.len() & 3 != 0 {
                note_data.push(0, GFP_KERNEL)?;
            }
            note_data.extend_from_slice(&note.data, GFP_KERNEL)?;
            while note_data.len() & 3 != 0 {
                note_data.push(0, GFP_KERNEL)?;
            }
        }

        // NOTE section comes after the PHDRs
        phdrs[0].p_offset = note_offset as u64;
        phdrs[0].p_filesz = note_data.len() as u64;

        // Align data section to the page size
        let data_offset = align(note_offset + note_data.len(), UAT_PGSZ);

        // Fix up data PHDR offsets
        for phdr in &mut phdrs[1..] {
            phdr.p_offset += data_offset as u64;
        }

        // Build ELF header buffer
        let mut headers: KVVec<u8> = KVVec::from_elem(0, data_offset, GFP_KERNEL)?;

        headers[0..size_of::<uapi::Elf64_Ehdr>()].copy_from_slice(ehdr.as_bytes());
        headers[phdr_offset..phdr_offset + phdrs.len() * size_of::<uapi::Elf64_Phdr>()]
            .copy_from_slice(AsBytes::slice_as_bytes(&phdrs));
        headers[note_offset..note_offset + note_data.len()].copy_from_slice(&note_data);

        Ok(CrashDump { headers, pages })
    }
}
