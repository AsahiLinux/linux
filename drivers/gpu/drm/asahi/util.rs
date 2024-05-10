// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Miscellaneous utility functions

use core::ops::{Add, BitAnd, Div, Not, Sub};

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

/// Integer division rounding up.
pub(crate) fn div_ceil<T>(a: T, b: T) -> T
where
    T: Copy
        + Default
        + BitAnd<Output = T>
        + Not<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Div<Output = T>,
{
    let def: T = Default::default();
    #[allow(clippy::eq_op)]
    let one: T = !def / !def;

    (a + b - one) / b
}

pub(crate) trait RangeExt<T> {
    fn overlaps(&self, other: Self) -> bool;
    fn is_superset(&self, other: Self) -> bool;
    fn len(&self) -> usize;
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
    fn len(&self) -> usize {
        self.range().try_into().unwrap()
    }
}
