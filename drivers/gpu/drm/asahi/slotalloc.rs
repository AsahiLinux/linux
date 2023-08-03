// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Generic slot allocator
//!
//! This is a simple allocator to manage fixed-size pools of GPU resources that are transiently
//! required during command execution. Each item resides in a "slot" at a given index. Users borrow
//! and return free items from the available pool.
//!
//! Allocations are "sticky", and return a token that callers can use to request the same slot
//! again later. This allows slots to be lazily invalidated, so that multiple uses by the same user
//! avoid any actual cleanup work.
//!
//! The allocation policy is currently a simple LRU mechanism, doing a full linear scan over the
//! slots when no token was previously provided. This is probably good enough, since in the absence
//! of serious system contention most allocation requests will be immediately fulfilled from the
//! previous slot without doing an LRU scan.

use core::ops::{Deref, DerefMut};
use kernel::{
    error::{code::*, Result},
    prelude::*,
    str::CStr,
    sync::{Arc, CondVar, LockClassKey, Mutex},
};

/// Trait representing a single item within a slot.
pub(crate) trait SlotItem {
    /// Arbitrary user data associated with the SlotAllocator.
    type Data;

    /// Called eagerly when this item is released back into the available pool.
    fn release(&mut self, _data: &mut Self::Data, _slot: u32) {}
}

/// Trivial implementation for users which do not require any slot data nor any allocator data.
impl SlotItem for () {
    type Data = ();
}

/// Represents a current or previous allocation of an item from a slot. Users keep `SlotToken`s
/// around across allocations to request that, if possible, the same slot be reused.
#[derive(Copy, Clone, Debug)]
pub(crate) struct SlotToken {
    time: u64,
    slot: u32,
}

impl SlotToken {
    /// Returns the slot index that this token represents a past assignment to.
    pub(crate) fn last_slot(&self) -> u32 {
        self.slot
    }
}

/// A guard representing active ownership of a slot.
pub(crate) struct Guard<T: SlotItem> {
    item: Option<T>,
    changed: bool,
    token: SlotToken,
    alloc: Arc<SlotAllocatorOuter<T>>,
}

impl<T: SlotItem> Guard<T> {
    /// Returns the active slot owned by this `Guard`.
    pub(crate) fn slot(&self) -> u32 {
        self.token.slot
    }

    /// Returns `true` if the slot changed since the last allocation (or no `SlotToken` was
    /// provided), or `false` if the previously allocated slot was successfully re-acquired with
    /// no other users in the interim.
    pub(crate) fn changed(&self) -> bool {
        self.changed
    }

    /// Returns a `SlotToken` that can be used to re-request the same slot at a later time, after
    /// this `Guard` is dropped.
    pub(crate) fn token(&self) -> SlotToken {
        self.token
    }
}

impl<T: SlotItem> Deref for Guard<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.item.as_ref().expect("SlotItem Guard lost our item!")
    }
}

impl<T: SlotItem> DerefMut for Guard<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.item.as_mut().expect("SlotItem Guard lost our item!")
    }
}

/// A slot item that is currently free.
struct Entry<T: SlotItem> {
    item: T,
    get_time: u64,
    drop_time: u64,
}

/// Inner data for the `SlotAllocator`, protected by a `Mutex`.
struct SlotAllocatorInner<T: SlotItem> {
    data: T::Data,
    slots: KVec<Option<Entry<T>>>,
    get_count: u64,
    drop_count: u64,
}

/// A single slot allocator instance.
#[pin_data]
struct SlotAllocatorOuter<T: SlotItem> {
    #[pin]
    inner: Mutex<SlotAllocatorInner<T>>,
    #[pin]
    cond: CondVar,
}

/// A shared reference to a slot allocator instance.
pub(crate) struct SlotAllocator<T: SlotItem>(Arc<SlotAllocatorOuter<T>>);

impl<T: SlotItem> SlotAllocator<T> {
    /// Creates a new `SlotAllocator`, with a fixed number of slots and arbitrary associated data.
    ///
    /// The caller provides a constructor callback which takes a reference to the `T::Data` and
    /// creates a single slot. This is called during construction to create all the initial
    /// items, which then live the lifetime of the `SlotAllocator`.
    pub(crate) fn new(
        num_slots: u32,
        mut data: T::Data,
        mut constructor: impl FnMut(&mut T::Data, u32) -> Option<T>,
        name: &'static CStr,
        lock_key1: LockClassKey,
        lock_key2: LockClassKey,
    ) -> Result<SlotAllocator<T>> {
        let mut slots = KVec::with_capacity(num_slots as usize, GFP_KERNEL)?;

        for i in 0..num_slots {
            slots
                .push(
                    constructor(&mut data, i).map(|item| Entry {
                        item,
                        get_time: 0,
                        drop_time: 0,
                    }),
                    GFP_KERNEL,
                )
                .expect("try_push() failed after reservation");
        }

        let inner = SlotAllocatorInner {
            data,
            slots,
            get_count: 0,
            drop_count: 0,
        };

        let alloc = Arc::pin_init(
            pin_init!(SlotAllocatorOuter {
                // SAFETY: `mutex_init!` is called below.
                inner <- Mutex::new_with_key(inner, name, lock_key1),
                // SAFETY: `condvar_init!` is called below.
                cond <- CondVar::new(name, lock_key2),
            }),
            GFP_KERNEL,
        )?;

        Ok(SlotAllocator(alloc))
    }

    /// Calls a callback on the inner data associated with this allocator, taking the lock.
    pub(crate) fn with_inner<RetVal>(&self, cb: impl FnOnce(&mut T::Data) -> RetVal) -> RetVal {
        let mut inner = self.0.inner.lock();
        cb(&mut inner.data)
    }

    /// Gets a fresh slot, optionally reusing a previous allocation if a `SlotToken` is provided.
    ///
    /// Blocks if no slots are free.
    pub(crate) fn get(&self, token: Option<SlotToken>) -> Result<Guard<T>> {
        self.get_inner(token, |_a, _b| Ok(()))
    }

    /// Gets a fresh slot, optionally reusing a previous allocation if a `SlotToken` is provided.
    ///
    /// Blocks if no slots are free.
    ///
    /// This version allows the caller to pass in a callback that gets a mutable reference to the
    /// user data for the allocator and the freshly acquired slot, which is called before the
    /// allocator lock is released. This can be used to perform bookkeeping associated with
    /// specific slots (such as tracking their current owner).
    pub(crate) fn get_inner(
        &self,
        token: Option<SlotToken>,
        cb: impl FnOnce(&mut T::Data, &mut Guard<T>) -> Result<()>,
    ) -> Result<Guard<T>> {
        let mut inner = self.0.inner.lock();

        if let Some(token) = token {
            let slot = &mut inner.slots[token.slot as usize];
            if slot.is_some() {
                let count = slot.as_ref().unwrap().get_time;
                if count == token.time {
                    let mut guard = Guard {
                        item: Some(slot.take().unwrap().item),
                        token,
                        changed: false,
                        alloc: self.0.clone(),
                    };
                    cb(&mut inner.data, &mut guard)?;
                    return Ok(guard);
                }
            }
        }

        let mut first = true;
        let slot = loop {
            let mut oldest_time = u64::MAX;
            let mut oldest_slot = 0u32;

            for (i, slot) in inner.slots.iter().enumerate() {
                if let Some(slot) = slot.as_ref() {
                    if slot.drop_time < oldest_time {
                        oldest_slot = i as u32;
                        oldest_time = slot.drop_time;
                    }
                }
            }

            if oldest_time == u64::MAX {
                if first {
                    pr_warn!(
                        "{}: out of slots, blocking\n",
                        core::any::type_name::<Self>()
                    );
                }
                first = false;
                if self.0.cond.wait_interruptible(&mut inner) {
                    return Err(ERESTARTSYS);
                }
            } else {
                break oldest_slot;
            }
        };

        inner.get_count += 1;

        let item = inner.slots[slot as usize]
            .take()
            .expect("Someone stole our slot?")
            .item;

        let mut guard = Guard {
            item: Some(item),
            changed: true,
            token: SlotToken {
                time: inner.get_count,
                slot,
            },
            alloc: self.0.clone(),
        };

        cb(&mut inner.data, &mut guard)?;
        Ok(guard)
    }
}

impl<T: SlotItem> Clone for SlotAllocator<T> {
    fn clone(&self) -> Self {
        SlotAllocator(self.0.clone())
    }
}

impl<T: SlotItem> Drop for Guard<T> {
    fn drop(&mut self) {
        let mut inner = self.alloc.inner.lock();
        if inner.slots[self.token.slot as usize].is_some() {
            pr_crit!(
                "{}: tried to return an item into a full slot ({})\n",
                core::any::type_name::<Self>(),
                self.token.slot
            );
        } else {
            inner.drop_count += 1;
            let mut item = self.item.take().expect("Guard lost its item");
            item.release(&mut inner.data, self.token.slot);
            inner.slots[self.token.slot as usize] = Some(Entry {
                item,
                get_time: self.token.time,
                drop_time: inner.drop_count,
            });
            self.alloc.cond.notify_one();
        }
    }
}
