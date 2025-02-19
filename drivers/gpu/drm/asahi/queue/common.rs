// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Common queue functionality.
//!
//! Shared helpers used by the submission logic for multiple command types.

use crate::file;
use crate::fw::job::UserTimestamp;

use kernel::prelude::*;
use kernel::uapi;
use kernel::xarray;

pub(super) fn get_timestamp_object(
    objects: Pin<&xarray::XArray<KBox<file::Object>>>,
    timestamp: uapi::drm_asahi_timestamp,
) -> Result<Option<UserTimestamp>> {
    if timestamp.handle == 0 {
        return Ok(None);
    }

    let object = objects.get(timestamp.handle.try_into()?).ok_or(ENOENT)?;

    #[allow(irrefutable_let_patterns)]
    if let file::Object::TimestampBuffer(mapping) = object.borrow() {
        let offset = timestamp.offset;
        if (offset.checked_add(8).ok_or(EINVAL)?) as usize > mapping.size() {
            return Err(ERANGE);
        }
        Ok(Some(UserTimestamp {
            mapping: mapping.clone(),
            offset: offset as usize,
        }))
    } else {
        Err(EINVAL)
    }
}
