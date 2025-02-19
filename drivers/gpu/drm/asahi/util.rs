// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Miscellaneous utility functions

use core::ops::{Add, BitAnd, Div, Not, Sub};
use kernel::prelude::*;

/// Aligns an integer type to a power of two.
pub(crate) fn align<T>(a: T, b: T) -> T
where
    T: Copy
        + Default
        + BitAnd<Output = T>
        + Not<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Div<Output = T>
        + core::cmp::PartialEq,
{
    let def: T = Default::default();
    #[allow(clippy::eq_op)]
    let one: T = !def / !def;

    assert!((b & (b - one)) == def);

    (a + b - one) & !(b - one)
}

/// Aligns an integer type down to a power of two.
pub(crate) fn align_down<T>(a: T, b: T) -> T
where
    T: Copy
        + Default
        + BitAnd<Output = T>
        + Not<Output = T>
        + Sub<Output = T>
        + Div<Output = T>
        + core::cmp::PartialEq,
{
    let def: T = Default::default();
    #[allow(clippy::eq_op)]
    let one: T = !def / !def;

    assert!((b & (b - one)) == def);

    a & !(b - one)
}

pub(crate) trait RangeExt<T> {
    fn overlaps(&self, other: Self) -> bool;
    fn is_superset(&self, other: Self) -> bool;
    // fn len(&self) -> usize;
    fn range(&self) -> T;
}

impl<T: PartialOrd<T> + Default + Copy + Sub<Output = T>> RangeExt<T> for core::ops::Range<T>
where
    usize: core::convert::TryFrom<T>,
    <usize as core::convert::TryFrom<T>>::Error: core::fmt::Debug,
{
    fn overlaps(&self, other: Self) -> bool {
        !(self.is_empty() || other.is_empty() || self.end <= other.start || other.end <= self.start)
    }
    fn is_superset(&self, other: Self) -> bool {
        !self.is_empty()
            && (other.is_empty() || (other.start >= self.start && other.end <= self.end))
    }
    fn range(&self) -> T {
        if self.is_empty() {
            Default::default()
        } else {
            self.end - self.start
        }
    }
    // fn len(&self) -> usize {
    //     self.range().try_into().unwrap()
    // }
}

pub(crate) fn gcd(in_n: u64, in_m: u64) -> u64 {
    let mut n = in_n;
    let mut m = in_m;

    while n != 0 {
        let remainder = m % n;
        m = n;
        n = remainder;
    }

    m
}

pub(crate) unsafe trait AnyBitPattern: Default + Sized + Copy + 'static {}

pub(crate) struct Reader<'a> {
    buffer: &'a [u8],
    offset: usize,
}

impl<'a> Reader<'a> {
    pub(crate) fn new(buffer: &'a [u8]) -> Self {
        Reader { buffer, offset: 0 }
    }

    pub(crate) fn read_up_to<T: AnyBitPattern>(&mut self, max_size: usize) -> Result<T> {
        let mut obj: T = Default::default();
        let size: usize = core::mem::size_of::<T>().min(max_size);
        let range = self.offset..self.offset + size;
        let src = self.buffer.get(range).ok_or(EINVAL)?;

        // SAFETY: The output pointer is valid, and the size does not exceed
        // the type size, and all bit patterns are valid.
        let dst = unsafe { core::slice::from_raw_parts_mut(&mut obj as *mut _ as *mut u8, size) };

        dst.copy_from_slice(src);
        self.offset += size;
        Ok(obj)
    }

    pub(crate) fn read<T: Default + AnyBitPattern>(&mut self) -> Result<T> {
        self.read_up_to(!0)
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.offset >= self.buffer.len()
    }

    pub(crate) fn skip(&mut self, size: usize) {
        self.offset += size
    }

    pub(crate) fn rewind(&mut self) {
        self.offset = 0
    }
}
