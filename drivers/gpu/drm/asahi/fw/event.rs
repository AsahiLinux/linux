// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU events control structures & stamps

use super::types::*;
use crate::{default_zeroed, trivial_gpustruct};
use core::sync::atomic::Ordering;

pub(crate) mod raw {
    use super::*;

    #[derive(Debug, Clone, Copy, Default)]
    #[repr(C)]
    pub(crate) struct LinkedListHead {
        pub(crate) prev: Option<GpuWeakPointer<LinkedListHead>>,
        pub(crate) next: Option<GpuWeakPointer<LinkedListHead>>,
    }

    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub(crate) struct NotifierList {
        pub(crate) list_head: LinkedListHead,
        pub(crate) unkptr_10: U64,
    }
    default_zeroed!(NotifierList);

    #[versions(AGX)]
    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub(crate) struct NotifierState {
        unk_14: u32,
        unk_18: U64,
        unk_20: u32,
        vm_slot: u32,
        has_vtx: u32,
        pstamp_vtx: Array<4, U64>,
        has_frag: u32,
        pstamp_frag: Array<4, U64>,
        has_comp: u32,
        pstamp_comp: Array<4, U64>,
        #[ver(G >= G14 && V < V13_0B4)]
        unk_98_g14_0: Array<0x14, u8>,
        in_list: u32,
        list_head: LinkedListHead,
        #[ver(G >= G14 && V < V13_0B4)]
        unk_a8_g14_0: Pad<4>,
        #[ver(V >= V13_0B4)]
        pub(crate) unk_buf: Array<0x8, u8>, // Init to all-ff
    }

    #[versions(AGX)]
    impl Default for NotifierState::ver {
        fn default() -> Self {
            #[allow(unused_mut)]
            let mut s: Self = unsafe { core::mem::zeroed() };
            #[ver(V >= V13_0B4)]
            s.unk_buf = Array::new([0xff; 0x8]);
            s
        }
    }

    #[derive(Debug)]
    #[repr(transparent)]
    pub(crate) struct Threshold(AtomicU64);
    default_zeroed!(Threshold);

    impl Threshold {
        pub(crate) fn increment(&self) {
            // We could use fetch_add, but the non-LSE atomic
            // sequence Rust produces confuses the hypervisor.
            let v = self.0.load(Ordering::Relaxed);
            self.0.store(v + 1, Ordering::Relaxed);
        }
    }

    #[versions(AGX)]
    #[derive(Debug)]
    #[repr(C)]
    pub(crate) struct Notifier<'a> {
        pub(crate) threshold: GpuPointer<'a, super::Threshold>,
        pub(crate) generation: AtomicU32,
        pub(crate) cur_count: AtomicU32,
        pub(crate) unk_10: AtomicU32,
        pub(crate) state: NotifierState::ver,
    }
}

trivial_gpustruct!(Threshold);
trivial_gpustruct!(NotifierList);

#[versions(AGX)]
#[derive(Debug)]
pub(crate) struct Notifier {
    pub(crate) threshold: GpuObject<Threshold>,
}

#[versions(AGX)]
impl GpuStruct for Notifier::ver {
    type Raw<'a> = raw::Notifier::ver<'a>;
}
