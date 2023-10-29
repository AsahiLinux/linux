// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Common GPU job firmware structures

use super::types::*;
use crate::{default_zeroed, trivial_gpustruct};

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub(crate) struct JobMeta {
        pub(crate) unk_0: u16,
        pub(crate) unk_2: u8,
        pub(crate) no_preemption: u8,
        pub(crate) stamp: GpuWeakPointer<Stamp>,
        pub(crate) fw_stamp: GpuWeakPointer<FwStamp>,
        pub(crate) stamp_value: EventValue,
        pub(crate) stamp_slot: u32,
        pub(crate) evctl_index: u32,
        pub(crate) flush_stamps: u32,
        pub(crate) uuid: u32,
        pub(crate) event_seq: u32,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct EncoderParams {
        pub(crate) unk_8: u32,
        pub(crate) sync_grow: u32,
        pub(crate) unk_10: u32,
        pub(crate) encoder_id: u32,
        pub(crate) unk_18: u32,
        pub(crate) unk_mask: u32,
        pub(crate) sampler_array: U64,
        pub(crate) sampler_count: u32,
        pub(crate) sampler_max: u32,
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct JobTimestamps {
        pub(crate) start: AtomicU64,
        pub(crate) end: AtomicU64,
    }
    default_zeroed!(JobTimestamps);

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct RenderTimestamps {
        pub(crate) vtx: JobTimestamps,
        pub(crate) frag: JobTimestamps,
    }
    default_zeroed!(RenderTimestamps);

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct Register {
        pub(crate) number: u32,
        pub(crate) value: U64,
    }
    default_zeroed!(Register);

    impl Register {
        fn new(number: u32, value: u64) -> Register {
            Register {
                number,
                value: U64(value),
            }
        }
    }

    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct RegisterArray {
        pub(crate) registers: Array<128, Register>,
        pub(crate) pad: Array<0x100, u8>,

        pub(crate) addr: GpuWeakPointer<Array<128, Register>>,
        pub(crate) count: u16,
        pub(crate) length: u16,
        pub(crate) unk_pad: u32,
    }

    impl RegisterArray {
        pub(crate) fn new(
            self_ptr: GpuWeakPointer<Array<128, Register>>,
            cb: impl FnOnce(&mut RegisterArray),
        ) -> RegisterArray {
            let mut array = RegisterArray {
                registers: Default::default(),
                pad: Default::default(),
                addr: self_ptr,
                count: 0,
                length: 0,
                unk_pad: 0,
            };

            cb(&mut array);

            array
        }

        pub(crate) fn add(&mut self, number: u32, value: u64) {
            self.registers[self.count as usize] = Register::new(number, value);
            self.count += 1;
            self.length += core::mem::size_of::<Register>() as u16;
        }
    }
}

trivial_gpustruct!(JobTimestamps);
trivial_gpustruct!(RenderTimestamps);
