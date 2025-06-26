// SPDX-License-Identifier: GPL-2.0

//! Rust DMA api test (based on QEMU's `pci-testdev`).
//!
//! To make this driver probe, QEMU must be run with `-device pci-testdev`.

use kernel::{
    bindings, device::Core, dma::CoherentAllocation, page::*, pci, prelude::*, scatterlist::*,
    types::ARef,
};

struct DmaSampleDriver {
    pdev: ARef<pci::Device>,
    ca: CoherentAllocation<MyStruct>,
    sgt: DeviceSGTable<PagesArray>,
}

const TEST_VALUES: [(u32, u32); 5] = [
    (0xa, 0xb),
    (0xc, 0xd),
    (0xe, 0xf),
    (0xab, 0xba),
    (0xcd, 0xef),
];

struct MyStruct {
    h: u32,
    b: u32,
}

impl MyStruct {
    fn new(h: u32, b: u32) -> Self {
        Self { h, b }
    }
}
// SAFETY: All bit patterns are acceptable values for `MyStruct`.
unsafe impl kernel::transmute::AsBytes for MyStruct {}
// SAFETY: Instances of `MyStruct` have no uninitialized portions.
unsafe impl kernel::transmute::FromBytes for MyStruct {}

struct PagesArray(KVec<Page>);

impl SGTablePages for PagesArray {
    fn iter<'a>(&'a self) -> impl Iterator<Item = (&'a Page, usize, usize)> {
        self.0.iter().map(|page| (page, kernel::page::PAGE_SIZE, 0))
    }

    fn entries(&self) -> usize {
        self.0.len()
    }
}

kernel::pci_device_table!(
    PCI_TABLE,
    MODULE_PCI_TABLE,
    <DmaSampleDriver as pci::Driver>::IdInfo,
    [(
        pci::DeviceId::from_id(bindings::PCI_VENDOR_ID_REDHAT, 0x5),
        ()
    )]
);

impl pci::Driver for DmaSampleDriver {
    type IdInfo = ();
    const ID_TABLE: pci::IdTable<Self::IdInfo> = &PCI_TABLE;

    fn probe(pdev: &pci::Device<Core>, _info: &Self::IdInfo) -> Result<Pin<KBox<Self>>> {
        dev_info!(pdev.as_ref(), "Probe DMA test driver.\n");

        let ca: CoherentAllocation<MyStruct> =
            CoherentAllocation::alloc_coherent(pdev.as_ref(), TEST_VALUES.len(), GFP_KERNEL)?;

        for (i, value) in TEST_VALUES.into_iter().enumerate() {
            kernel::dma_write!(ca[i] = MyStruct::new(value.0, value.1))?;
        }

        let mut pages = KVec::new();
        for _ in TEST_VALUES.into_iter() {
            let _ = pages.push(Page::alloc_page(GFP_KERNEL)?, GFP_KERNEL);
        }

        let sgt = SGTable::alloc_table(PagesArray(pages), GFP_KERNEL)?;
        let sgt = sgt.dma_map(pdev.as_ref(), kernel::dma::DmaDataDirection::DmaToDevice)?;

        let drvdata = KBox::new(
            Self {
                pdev: pdev.into(),
                ca,
                // Save object to excercise the destructor.
                sgt,
            },
            GFP_KERNEL,
        )?;

        Ok(drvdata.into())
    }
}

impl Drop for DmaSampleDriver {
    fn drop(&mut self) {
        dev_info!(self.pdev.as_ref(), "Unload DMA test driver.\n");
        assert_eq!(self.sgt.iter().count(), TEST_VALUES.len());

        for (i, value) in TEST_VALUES.into_iter().enumerate() {
            let val0 = kernel::dma_read!(self.ca[i].h);
            let val1 = kernel::dma_read!(self.ca[i].b);
            assert!(val0.is_ok());
            assert!(val1.is_ok());

            if let Ok(val0) = val0 {
                assert_eq!(val0, value.0);
            }
            if let Ok(val1) = val1 {
                assert_eq!(val1, value.1);
            }
        }
    }
}

kernel::module_pci_driver! {
    type: DmaSampleDriver,
    name: "rust_dma",
    authors: ["Abdiel Janulgue"],
    description: "Rust DMA test",
    license: "GPL v2",
}
