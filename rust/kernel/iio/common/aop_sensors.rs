// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP sensors common code
//!
//! Copyright (C) The Asahi Linux Contributors

use core::marker::{PhantomData, PhantomPinned};
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};

use kernel::{
    bindings,
    device,
    prelude::*,
    soc::apple::aop::FakehidListener,
    sync::{
        aref::ARef,
        Arc, //
    },
    types::ForeignOwnable,
    ThisModule, //
};

/// TODO: add documentation
pub trait MessageProcessor {
    /// TODO: add documentation
    fn process(&self, message: &[u8]) -> u32;
}

/// TODO: add documentation
pub struct AopSensorData<T: MessageProcessor> {
    dev: ARef<device::Device>,
    ty: u32,
    value: AtomicU32,
    msg_proc: T,
}

impl<T: MessageProcessor> AopSensorData<T> {
    /// TODO: add documentation
    pub fn new(dev: ARef<device::Device>, ty: u32, msg_proc: T) -> Result<Arc<AopSensorData<T>>> {
        Ok(Arc::new(
            AopSensorData {
                dev,
                ty,
                value: AtomicU32::new(0),
                msg_proc,
            },
            GFP_KERNEL,
        )?)
    }
}

impl<T: MessageProcessor> FakehidListener for AopSensorData<T> {
    fn process_fakehid_report(&self, data: &[u8]) -> Result<()> {
        self.value
            .store(self.msg_proc.process(data), Ordering::Relaxed);
        Ok(())
    }
}

unsafe extern "C" fn aop_read_raw<T: MessageProcessor + 'static>(
    dev: *mut bindings::iio_dev,
    chan: *const bindings::iio_chan_spec,
    val: *mut i32,
    _: *mut i32,
    mask: isize,
) -> i32 {
    let data = unsafe { Arc::<AopSensorData<T>>::borrow((*dev).priv_.cast()) };
    let ty = unsafe { (*chan).type_ };
    if mask != bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED as isize
        && mask != bindings::BINDINGS_IIO_CHAN_INFO_RAW as isize
    {
        return EINVAL.to_errno();
    }
    if data.ty != ty {
        return EINVAL.to_errno();
    }
    let value = data.value.load(Ordering::Relaxed);
    unsafe {
        *val = value as i32;
    }
    bindings::IIO_VAL_INT as i32
}

struct IIOSpec {
    spec: [bindings::iio_chan_spec; 1],
    vtable: bindings::iio_info,
    _p: PhantomPinned,
}

/// TODO: add documentation
pub struct IIORegistration<T: MessageProcessor + 'static> {
    dev: *mut bindings::iio_dev,
    spec: Pin<KBox<IIOSpec>>,
    registered: bool,
    _p: PhantomData<AopSensorData<T>>,
}

impl<T: MessageProcessor + 'static> IIORegistration<T> {
    /// TODO: add documentation
    pub fn new(
        data: Arc<AopSensorData<T>>,
        name: &'static CStr,
        ty: u32,
        info_mask: usize,
        module: &ThisModule,
    ) -> Result<Self> {
        let spec = KBox::pin(
            IIOSpec {
                spec: [bindings::iio_chan_spec {
                    type_: ty,
                    __bindgen_anon_1: bindings::iio_chan_spec__bindgen_ty_1 {
                        scan_type: bindings::iio_scan_type {
                            __bindgen_anon_1: bindings::iio_scan_type__bindgen_ty_1 {
                                format: b'u' as _,
                            },
                            realbits: 32,
                            storagebits: 32,
                            ..Default::default()
                        },
                    },
                    info_mask_separate: info_mask,
                    ..Default::default()
                }],
                vtable: bindings::iio_info {
                    read_raw: Some(aop_read_raw::<T>),
                    ..Default::default()
                },
                _p: PhantomPinned,
            },
            GFP_KERNEL,
        )?;
        let mut this = IIORegistration {
            dev: ptr::null_mut(),
            spec,
            registered: false,
            _p: PhantomData,
        };
        this.dev = unsafe { bindings::iio_device_alloc(data.dev.as_raw(), 0) };
        unsafe {
            (*this.dev).priv_ = data.clone().into_foreign().cast();
            (*this.dev).name = name.as_ptr() as _;
            // spec is now pinned
            (*this.dev).channels = this.spec.spec.as_ptr();
            (*this.dev).num_channels = this.spec.spec.len() as i32;
            (*this.dev).info = &this.spec.vtable;
        }
        let ret = unsafe { bindings::__iio_device_register(this.dev, module.as_ptr()) };
        if ret < 0 {
            dev_err!(data.dev, "Unable to register iio sensor");
            return Err(Error::from_errno(ret));
        }
        this.registered = true;
        Ok(this)
    }
}

impl<T: MessageProcessor + 'static> Drop for IIORegistration<T> {
    fn drop(&mut self) {
        if self.dev != ptr::null_mut() {
            unsafe {
                if self.registered {
                    bindings::iio_device_unregister(self.dev);
                }
                Arc::<AopSensorData<T>>::from_foreign((*self.dev).priv_.cast());
                bindings::iio_device_free(self.dev);
            }
        }
    }
}

unsafe impl<T: MessageProcessor> Send for IIORegistration<T> {}
unsafe impl<T: MessageProcessor> Sync for IIORegistration<T> {}
