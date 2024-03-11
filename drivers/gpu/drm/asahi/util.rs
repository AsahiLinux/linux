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
