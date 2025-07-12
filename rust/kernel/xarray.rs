// SPDX-License-Identifier: GPL-2.0

//! XArray abstraction.
//!
//! C header: [`include/linux/xarray.h`](srctree/include/linux/xarray.h)

use crate::{
    alloc,
    prelude::*,
    types::{ForeignOwnable, NotThreadSafe, Opaque, ScopeGuard},
};
use core::{
    fmt, iter,
    marker::PhantomData,
    mem, ops,
    ptr::{null_mut, NonNull},
};

/// An array which efficiently maps sparse integer indices to owned objects.
///
/// This is similar to a [`Vec<Option<T>>`], but more efficient when there are
/// holes in the index space, and can be efficiently grown.
///
/// # Invariants
///
/// `self.xa` is always an initialized and valid [`bindings::xarray`] whose entries are either
/// `XA_ZERO_ENTRY` or came from `T::into_foreign`.
///
/// # Examples
///
/// ```rust
/// # use kernel::alloc::KBox;
/// # use kernel::xarray::XArray;
/// # use pin_init::stack_pin_init;
///
/// stack_pin_init!(let xa = XArray::new(Default::default()));
///
/// let dead = KBox::new(0xdead, GFP_KERNEL)?;
/// let beef = KBox::new(0xbeef, GFP_KERNEL)?;
///
/// let mut guard = xa.lock();
///
/// assert_eq!(guard.get(0), None);
///
/// assert_eq!(guard.store(0, dead, GFP_KERNEL)?.as_deref(), None);
/// assert_eq!(guard.get(0).copied(), Some(0xdead));
///
/// *guard.get_mut(0).unwrap() = 0xffff;
/// assert_eq!(guard.get(0).copied(), Some(0xffff));
///
/// assert_eq!(guard.store(0, beef, GFP_KERNEL)?.as_deref().copied(), Some(0xffff));
/// assert_eq!(guard.get(0).copied(), Some(0xbeef));
///
/// guard.remove(0);
/// assert_eq!(guard.get(0), None);
///
/// # Ok::<(), Error>(())
/// ```
#[pin_data(PinnedDrop)]
pub struct XArray<T: ForeignOwnable> {
    #[pin]
    xa: Opaque<bindings::xarray>,
    _p: PhantomData<T>,
}

#[pinned_drop]
impl<T: ForeignOwnable> PinnedDrop for XArray<T> {
    fn drop(self: Pin<&mut Self>) {
        self.iter().for_each(|ptr| {
            let ptr = ptr.as_ptr();
            // SAFETY: `ptr` came from `T::into_foreign`.
            //
            // INVARIANT: we own the only reference to the array which is being dropped so the
            // broken invariant is not observable on function exit.
            drop(unsafe { T::from_foreign(ptr) })
        });

        // SAFETY: `self.xa` is always valid by the type invariant.
        unsafe { bindings::xa_destroy(self.xa.get()) };
    }
}

/// Flags passed to [`XArray::new`] to configure the array's allocation tracking behavior.
#[derive(Default)]
pub enum AllocKind {
    /// Consider the first element to be at index 0.
    #[default]
    Alloc,
    /// Consider the first element to be at index 1.
    Alloc1,
}

impl<T: ForeignOwnable> XArray<T> {
    /// Creates a new initializer for this type.
    pub fn new(kind: AllocKind) -> impl PinInit<Self> {
        let flags = match kind {
            AllocKind::Alloc => bindings::XA_FLAGS_ALLOC,
            AllocKind::Alloc1 => bindings::XA_FLAGS_ALLOC1,
        };
        pin_init!(Self {
            // SAFETY: `xa` is valid while the closure is called.
            //
            // INVARIANT: `xa` is initialized here to an empty, valid [`bindings::xarray`].
            xa <- Opaque::ffi_init(|xa| unsafe {
                bindings::xa_init_flags(xa, flags)
            }),
            _p: PhantomData,
        })
    }

    fn iter(&self) -> impl Iterator<Item = NonNull<T::PointedTo>> + '_ {
        let mut index = 0;

        core::iter::Iterator::chain(
            // SAFETY: `self.xa` is always valid by the type invariant.
            iter::once(unsafe {
                bindings::xa_find(self.xa.get(), &mut index, usize::MAX, bindings::XA_PRESENT)
            }),
            iter::from_fn(move || {
                // SAFETY: `self.xa` is always valid by the type invariant.
                Some(unsafe {
                    bindings::xa_find_after(
                        self.xa.get(),
                        &mut index,
                        usize::MAX,
                        bindings::XA_PRESENT,
                    )
                })
            }),
        )
        .map_while(|ptr| NonNull::new(ptr.cast()))
    }

    /// Looks up and returns a reference to the lowest entry in the array between index and max,
    /// returning a tuple of its index and a `Guard` if one exists.
    ///
    /// This guard blocks all other actions on the `XArray`. Callers are expected to drop the
    /// `Guard` eagerly to avoid blocking other users, such as by taking a clone of the value.
    pub fn find(&self, index: usize, max: usize) -> Option<(usize, ValueGuard<'_, T>)> {
        let mut index: usize = index;

        // SAFETY: `self.xa` is always valid by the type invariant.
        unsafe { bindings::xa_lock(self.xa.get()) };

        // SAFETY: `self.xa` is always valid by the type invariant.
        let guard = ScopeGuard::new(|| unsafe { bindings::xa_unlock(self.xa.get()) });

        // SAFETY: `self.xa` is always valid by the type invariant.
        let p = unsafe { bindings::xa_find(self.xa.get(), &mut index, max, bindings::XA_PRESENT) };

        NonNull::new(p as *mut T).map(|ptr| {
            guard.dismiss();
            (
                index,
                ValueGuard {
                    xa: self,
                    ptr,
                    _not_send: NotThreadSafe,
                },
            )
        })
    }

    fn with_guard<F, U>(&self, guard: Option<&mut Guard<'_, T>>, f: F) -> U
    where
        F: FnOnce(&mut Guard<'_, T>) -> U,
    {
        match guard {
            None => f(&mut self.lock()),
            Some(guard) => {
                assert_eq!(guard.xa.xa.get(), self.xa.get());
                f(guard)
            }
        }
    }

    /// Attempts to lock the [`XArray`] for exclusive access.
    pub fn try_lock(&self) -> Option<Guard<'_, T>> {
        // SAFETY: `self.xa` is always valid by the type invariant.
        if (unsafe { bindings::xa_trylock(self.xa.get()) } != 0) {
            Some(Guard {
                xa: self,
                _not_send: NotThreadSafe,
            })
        } else {
            None
        }
    }

    /// Locks the [`XArray`] for exclusive access.
    pub fn lock(&self) -> Guard<'_, T> {
        // SAFETY: `self.xa` is always valid by the type invariant.
        unsafe { bindings::xa_lock(self.xa.get()) };

        Guard {
            xa: self,
            _not_send: NotThreadSafe,
        }
    }

    /// Removes and returns the element at the given index.
    pub fn remove(&self, index: usize) -> Option<T> {
        let mut guard = self.lock();
        guard.remove(index)
    }
}

/// A lock guard.
///
/// The lock is unlocked when the guard goes out of scope.
#[must_use = "the lock unlocks immediately when the guard is unused"]
pub struct Guard<'a, T: ForeignOwnable> {
    xa: &'a XArray<T>,
    _not_send: NotThreadSafe,
}

impl<T: ForeignOwnable> Drop for Guard<'_, T> {
    fn drop(&mut self) {
        // SAFETY:
        // - `self.xa.xa` is always valid by the type invariant.
        // - The caller holds the lock, so it is safe to unlock it.
        unsafe { bindings::xa_unlock(self.xa.xa.get()) };
    }
}

/// A lock guard.
///
/// The lock is unlocked when the guard goes out of scope.
#[must_use = "the lock unlocks immediately when the guard is unused"]
pub struct ValueGuard<'a, T: ForeignOwnable> {
    xa: &'a XArray<T>,
    ptr: NonNull<T>,
    _not_send: NotThreadSafe,
}

impl<'a, T: ForeignOwnable> ValueGuard<'a, T> {
    /// Borrow the underlying value wrapped by the `Guard`.
    ///
    /// Returns a `T::Borrowed` type for the owned `ForeignOwnable` type.
    pub fn borrow(&self) -> T::Borrowed<'_> {
        // SAFETY: The value is owned by the `XArray`, the lifetime it is borrowed for must not
        // outlive the `XArray` itself, nor the Guard that holds the lock ensuring the value
        // remains in the `XArray`.
        unsafe { T::borrow(self.ptr.as_ptr() as _) }
    }
}

impl<T: ForeignOwnable> Drop for ValueGuard<'_, T> {
    fn drop(&mut self) {
        // SAFETY:
        // - `self.xa.xa` is always valid by the type invariant.
        // - The caller holds the lock, so it is safe to unlock it.
        unsafe { bindings::xa_unlock(self.xa.xa.get()) };
    }
}

/// The error returned by [`store`](Guard::store).
///
/// Contains the underlying error and the value that was not stored.
#[derive(Debug)]
pub struct StoreError<T> {
    /// The error that occurred.
    pub error: Error,
    /// The value that was not stored.
    pub value: T,
}

impl<T> From<StoreError<T>> for Error {
    fn from(value: StoreError<T>) -> Self {
        value.error
    }
}

fn to_usize(i: u32) -> usize {
    i.try_into()
        .unwrap_or_else(|_| build_error!("cannot convert u32 to usize"))
}

impl<'a, T: ForeignOwnable> Guard<'a, T> {
    fn load<F, U>(&self, index: usize, f: F) -> Option<U>
    where
        F: FnOnce(NonNull<T::PointedTo>) -> U,
    {
        // SAFETY: `self.xa.xa` is always valid by the type invariant.
        let ptr = unsafe { bindings::xa_load(self.xa.xa.get(), index) };
        let ptr = NonNull::new(ptr.cast())?;
        Some(f(ptr))
    }

    /// Provides a reference to the element at the given index.
    pub fn get(&self, index: usize) -> Option<T::Borrowed<'_>> {
        self.load(index, |ptr| {
            // SAFETY: `ptr` came from `T::into_foreign`.
            unsafe { T::borrow(ptr.as_ptr()) }
        })
    }

    /// Provides a mutable reference to the element at the given index.
    pub fn get_mut(&mut self, index: usize) -> Option<T::BorrowedMut<'_>> {
        self.load(index, |ptr| {
            // SAFETY: `ptr` came from `T::into_foreign`.
            unsafe { T::borrow_mut(ptr.as_ptr()) }
        })
    }

    /// Removes and returns the element at the given index.
    pub fn remove(&mut self, index: usize) -> Option<T> {
        // SAFETY:
        // - `self.xa.xa` is always valid by the type invariant.
        // - The caller holds the lock.
        let ptr = unsafe { bindings::__xa_erase(self.xa.xa.get(), index) }.cast();
        // SAFETY:
        // - `ptr` is either `NULL` or came from `T::into_foreign`.
        // - `&mut self` guarantees that the lifetimes of [`T::Borrowed`] and [`T::BorrowedMut`]
        // borrowed from `self` have ended.
        unsafe { T::try_from_foreign(ptr) }
    }

    /// Stores an element at the given index.
    ///
    /// May drop the lock if needed to allocate memory, and then reacquire it afterwards.
    ///
    /// On success, returns the element which was previously at the given index.
    ///
    /// On failure, returns the element which was attempted to be stored.
    pub fn store(
        &mut self,
        index: usize,
        value: T,
        gfp: alloc::Flags,
    ) -> Result<Option<T>, StoreError<T>> {
        build_assert!(
            mem::align_of::<T::PointedTo>() >= 4,
            "pointers stored in XArray must be 4-byte aligned"
        );
        let new = value.into_foreign();

        let old = {
            let new = new.cast();
            // SAFETY:
            // - `self.xa.xa` is always valid by the type invariant.
            // - The caller holds the lock.
            //
            // INVARIANT: `new` came from `T::into_foreign`.
            unsafe { bindings::__xa_store(self.xa.xa.get(), index, new, gfp.as_raw()) }
        };

        // SAFETY: `__xa_store` returns the old entry at this index on success or `xa_err` if an
        // error happened.
        let errno = unsafe { bindings::xa_err(old) };
        if errno != 0 {
            // SAFETY: `new` came from `T::into_foreign` and `__xa_store` does not take
            // ownership of the value on error.
            let value = unsafe { T::from_foreign(new) };
            Err(StoreError {
                value,
                error: Error::from_errno(errno),
            })
        } else {
            let old = old.cast();
            // SAFETY: `ptr` is either `NULL` or came from `T::into_foreign`.
            //
            // NB: `XA_ZERO_ENTRY` is never returned by functions belonging to the Normal XArray
            // API; such entries present as `NULL`.
            Ok(unsafe { T::try_from_foreign(old) })
        }
    }

    /// Stores an element at the given index if no entry is present.
    ///
    /// May drop the lock if needed to allocate memory, and then reacquire it afterwards.
    ///
    /// On failure, returns the element which was attempted to be stored.
    pub fn insert(
        &mut self,
        index: usize,
        value: T,
        gfp: alloc::Flags,
    ) -> Result<(), StoreError<T>> {
        build_assert!(
            mem::align_of::<T::PointedTo>() >= 4,
            "pointers stored in XArray must be 4-byte aligned"
        );
        let ptr = value.into_foreign();
        // SAFETY: `self.xa` is always valid by the type invariant.
        //
        // INVARIANT: `ptr` came from `T::into_foreign`.
        match unsafe { bindings::__xa_insert(self.xa.xa.get(), index, ptr.cast(), gfp.as_raw()) } {
            0 => Ok(()),
            errno => {
                // SAFETY: `ptr` came from `T::into_foreign` and `__xa_insert` does not take
                // ownership of the value on error.
                let value = unsafe { T::from_foreign(ptr) };
                Err(StoreError {
                    value,
                    error: Error::from_errno(errno),
                })
            }
        }
    }

    /// Wrapper around `__xa_alloc`.
    ///
    /// On success, takes ownership of pointers passed in `op`.
    ///
    /// On failure, ownership returns to the caller.
    ///
    /// # Safety
    ///
    /// `ptr` must be `NULL` or have come from a previous call to `T::into_foreign`.
    unsafe fn alloc(
        &mut self,
        limit: impl ops::RangeBounds<u32>,
        ptr: *mut T::PointedTo,
        gfp: alloc::Flags,
    ) -> Result<usize> {
        // NB: `xa_limit::{max,min}` are inclusive.
        let limit = bindings::xa_limit {
            max: match limit.end_bound() {
                ops::Bound::Included(&end) => end,
                ops::Bound::Excluded(&end) => end - 1,
                ops::Bound::Unbounded => u32::MAX,
            },
            min: match limit.start_bound() {
                ops::Bound::Included(&start) => start,
                ops::Bound::Excluded(&start) => start + 1,
                ops::Bound::Unbounded => 0,
            },
        };

        let mut index = u32::MAX;

        // SAFETY:
        // - `self.xa` is always valid by the type invariant.
        // - `self.xa` was initialized with `XA_FLAGS_ALLOC` or `XA_FLAGS_ALLOC1`.
        //
        // INVARIANT: `ptr` is either `NULL` or came from `T::into_foreign`.
        match unsafe {
            bindings::__xa_alloc(
                self.xa.xa.get(),
                &mut index,
                ptr.cast(),
                limit,
                gfp.as_raw(),
            )
        } {
            0 => Ok(to_usize(index)),
            errno => Err(Error::from_errno(errno)),
        }
    }

    /// Allocates an entry somewhere in the array.
    ///
    /// On success, returns the index at which the entry was stored.
    ///
    /// On failure, returns the entry which was attempted to be stored.
    pub fn insert_limit(
        &mut self,
        limit: impl ops::RangeBounds<u32>,
        value: T,
        gfp: alloc::Flags,
    ) -> Result<usize, StoreError<T>> {
        build_assert!(
            mem::align_of::<T::PointedTo>() >= 4,
            "pointers stored in XArray must be 4-byte aligned"
        );
        let ptr = value.into_foreign();
        // SAFETY: `ptr` came from `T::into_foreign`.
        unsafe { self.alloc(limit, ptr, gfp) }.map_err(|error| {
            // SAFETY: `ptr` came from `T::into_foreign` and `self.alloc` does not take ownership of
            // the value on error.
            let value = unsafe { T::from_foreign(ptr) };
            StoreError { value, error }
        })
    }

    /// Reserves an entry in the array.
    pub fn reserve(&mut self, index: usize, gfp: alloc::Flags) -> Result<Reservation<'a, T>> {
        // NB: `__xa_insert` internally coerces `NULL` to `XA_ZERO_ENTRY` on ingress.
        let ptr = null_mut();
        // SAFETY: `self.xa` is always valid by the type invariant.
        //
        // INVARIANT: `ptr` is `NULL`.
        match unsafe { bindings::__xa_insert(self.xa.xa.get(), index, ptr, gfp.as_raw()) } {
            0 => Ok(Reservation { xa: self.xa, index }),
            errno => Err(Error::from_errno(errno)),
        }
    }

    /// Reserves an entry somewhere in the array.
    pub fn reserve_limit(
        &mut self,
        limit: impl ops::RangeBounds<u32>,
        gfp: alloc::Flags,
    ) -> Result<Reservation<'a, T>> {
        // NB: `__xa_alloc` internally coerces `NULL` to `XA_ZERO_ENTRY` on ingress.
        let ptr = null_mut();
        // SAFETY: `ptr` is `NULL`.
        unsafe { self.alloc(limit, ptr, gfp) }.map(|index| Reservation { xa: self.xa, index })
    }
}

/// A reserved slot in an array.
///
/// The slot is released when the reservation goes out of scope.
///
/// Note that the array lock *must not* be held when the reservation is filled or dropped as this
/// will lead to deadlock. [`Reservation::fill_locked`] and [`Reservation::release_locked`] can be
/// used in context where the array lock is held.
#[must_use = "the reservation is released immediately when the reservation is unused"]
pub struct Reservation<'a, T: ForeignOwnable> {
    xa: &'a XArray<T>,
    index: usize,
}

impl<T: ForeignOwnable> fmt::Debug for Reservation<'_, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Reservation")
            .field("index", &self.index())
            .finish()
    }
}

impl<T: ForeignOwnable> Reservation<'_, T> {
    /// Returns the index of the reservation.
    pub fn index(&self) -> usize {
        self.index
    }

    /// Replaces the reserved entry with the given entry.
    ///
    /// # Safety
    ///
    /// `ptr` must be `NULL` or have come from a previous call to `T::into_foreign`.
    unsafe fn replace(guard: &mut Guard<'_, T>, index: usize, ptr: *mut T::PointedTo) -> Result {
        // SAFETY: `xa_zero_entry` wraps `XA_ZERO_ENTRY` which is always safe to use.
        let old = unsafe { bindings::xa_zero_entry() };

        // NB: `__xa_cmpxchg_raw` is used over `__xa_cmpxchg` because the latter coerces
        // `XA_ZERO_ENTRY` to `NULL` on egress, which would prevent us from determining whether a
        // replacement was made.
        //
        // SAFETY: `self.xa` is always valid by the type invariant.
        //
        // INVARIANT: `ptr` is either `NULL` or came from `T::into_foreign` and `old` is
        // `XA_ZERO_ENTRY`.
        let ret =
            unsafe { bindings::__xa_cmpxchg_raw(guard.xa.xa.get(), index, old, ptr.cast(), 0) };

        // SAFETY: `__xa_cmpxchg_raw` returns the old entry at this index on success or `xa_err` if
        // an error happened.
        match unsafe { bindings::xa_err(ret) } {
            0 => {
                if ret == old {
                    Ok(())
                } else {
                    Err(EBUSY)
                }
            }
            errno => Err(Error::from_errno(errno)),
        }
    }

    fn fill_inner(&self, guard: Option<&mut Guard<'_, T>>, value: T) -> Result<(), StoreError<T>> {
        let Self { xa, index } = self;
        let index = *index;

        let ptr = value.into_foreign();
        xa.with_guard(guard, |guard| {
            // SAFETY: `ptr` came from `T::into_foreign`.
            unsafe { Self::replace(guard, index, ptr) }
        })
        .map_err(|error| {
            // SAFETY: `ptr` came from `T::into_foreign` and `Self::replace` does not take ownership
            // of the value on error.
            let value = unsafe { T::from_foreign(ptr) };
            StoreError { value, error }
        })
    }

    /// Fills the reservation.
    pub fn fill(self, value: T) -> Result<(), StoreError<T>> {
        let result = self.fill_inner(None, value);
        mem::forget(self);
        result
    }

    /// Fills the reservation without acquiring the array lock.
    ///
    /// # Panics
    ///
    /// Panics if the passed guard locks a different array.
    pub fn fill_locked(self, guard: &mut Guard<'_, T>, value: T) -> Result<(), StoreError<T>> {
        let result = self.fill_inner(Some(guard), value);
        mem::forget(self);
        result
    }

    fn release_inner(&self, guard: Option<&mut Guard<'_, T>>) -> Result {
        let Self { xa, index } = self;
        let index = *index;

        xa.with_guard(guard, |guard| {
            let ptr = null_mut();
            // SAFETY: `ptr` is `NULL`.
            unsafe { Self::replace(guard, index, ptr) }
        })
    }

    /// Releases the reservation without acquiring the array lock.
    ///
    /// # Panics
    ///
    /// Panics if the passed guard locks a different array.
    pub fn release_locked(self, guard: &mut Guard<'_, T>) -> Result {
        let result = self.release_inner(Some(guard));
        mem::forget(self);
        result
    }
}

impl<T: ForeignOwnable> Drop for Reservation<'_, T> {
    fn drop(&mut self) {
        // NB: Errors here are possible since `Guard::store` does not honor reservations.
        let _: Result = self.release_inner(None);
    }
}

// SAFETY: `XArray<T>` has no shared mutable state so it is `Send` iff `T` is `Send`.
unsafe impl<T: ForeignOwnable + Send> Send for XArray<T> {}

// SAFETY: `XArray<T>` serialises the interior mutability it provides so it is `Sync` iff `T` is
// `Send`.
unsafe impl<T: ForeignOwnable + Send> Sync for XArray<T> {}

#[macros::kunit_tests(rust_xarray_kunit)]
mod tests {
    use super::*;
    use pin_init::stack_pin_init;

    fn new_kbox<T>(value: T) -> Result<KBox<T>> {
        KBox::new(value, GFP_KERNEL).map_err(Into::into)
    }

    #[test]
    fn test_alloc_kind_alloc() -> Result {
        test_alloc_kind(AllocKind::Alloc, 0)
    }

    #[test]
    fn test_alloc_kind_alloc1() -> Result {
        test_alloc_kind(AllocKind::Alloc1, 1)
    }

    fn test_alloc_kind(kind: AllocKind, expected_index: usize) -> Result {
        stack_pin_init!(let xa = XArray::new(kind));
        let mut guard = xa.lock();

        let reservation = guard.reserve_limit(.., GFP_KERNEL)?;
        assert_eq!(reservation.index(), expected_index);
        reservation.release_locked(&mut guard)?;

        let insertion = guard.insert_limit(.., new_kbox(0x1337)?, GFP_KERNEL);
        assert!(insertion.is_ok());
        let insertion_index = insertion.unwrap();
        assert_eq!(insertion_index, expected_index);

        Ok(())
    }

    const IDX: usize = 0x1337;

    fn insert<T: ForeignOwnable>(guard: &mut Guard<'_, T>, value: T) -> Result<(), StoreError<T>> {
        guard.insert(IDX, value, GFP_KERNEL)
    }

    fn reserve<'a, T: ForeignOwnable>(guard: &mut Guard<'a, T>) -> Result<Reservation<'a, T>> {
        guard.reserve(IDX, GFP_KERNEL)
    }

    #[track_caller]
    fn check_not_vacant<'a>(guard: &mut Guard<'a, KBox<usize>>) -> Result {
        // Insertion fails.
        {
            let beef = new_kbox(0xbeef)?;
            let ret = insert(guard, beef);
            assert!(ret.is_err());
            let StoreError { error, value } = ret.unwrap_err();
            assert_eq!(error, EBUSY);
            assert_eq!(*value, 0xbeef);
        }

        // Reservation fails.
        {
            let ret = reserve(guard);
            assert!(ret.is_err());
            assert_eq!(ret.unwrap_err(), EBUSY);
        }

        Ok(())
    }

    #[test]
    fn test_insert_and_reserve_interaction() -> Result {
        stack_pin_init!(let xa = XArray::new(Default::default()));
        let mut guard = xa.lock();

        // Vacant.
        assert_eq!(guard.get(IDX), None);

        // Reservation succeeds.
        let reservation = {
            let ret = reserve(&mut guard);
            assert!(ret.is_ok());
            ret.unwrap()
        };

        // Reserved presents as vacant.
        assert_eq!(guard.get(IDX), None);

        check_not_vacant(&mut guard)?;

        // Release reservation.
        {
            let ret = reservation.release_locked(&mut guard);
            assert!(ret.is_ok());
            let () = ret.unwrap();
        }

        // Vacant again.
        assert_eq!(guard.get(IDX), None);

        // Insert succeeds.
        {
            let dead = new_kbox(0xdead)?;
            let ret = insert(&mut guard, dead);
            assert!(ret.is_ok());
            let () = ret.unwrap();
        }

        check_not_vacant(&mut guard)?;

        // Remove.
        assert_eq!(guard.remove(IDX).as_deref(), Some(&0xdead));

        // Reserve and fill.
        {
            let beef = new_kbox(0xbeef)?;
            let ret = reserve(&mut guard);
            assert!(ret.is_ok());
            let reservation = ret.unwrap();
            let ret = reservation.fill_locked(&mut guard, beef);
            assert!(ret.is_ok());
            let () = ret.unwrap();
        };

        check_not_vacant(&mut guard)?;

        // Remove.
        assert_eq!(guard.remove(IDX).as_deref(), Some(&0xbeef));

        Ok(())
    }
}
