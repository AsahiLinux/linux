// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple AOP audio driver
//!
//! Copyright (C) The Asahi Linux Contributors

use core::sync::atomic::{AtomicU32, Ordering};
use core::{mem, ptr, slice};

use kernel::{
    bindings, c_str, device,
    device::property::FwNode,
    device::Core,
    error::from_err_ptr,
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::aop::{from_fourcc, EPICService, AOP},
    str::CString,
    sync::Arc,
    types::{ARef, ForeignOwnable},
};

use pin_init::Zeroable;

const EPIC_SUBTYPE_WRAPPED_CALL: u16 = 0x20;
const CALLTYPE_AUDIO_ATTACH_DEVICE: u32 = 0xc3000002;
const CALLTYPE_AUDIO_SET_PROP: u32 = 0xc3000005;
const PDM_NUM_COEFFS: usize = 120;
const DECIMATION_RATIOS: [u8; 3] = [0xf, 5, 2];
const COEFFICIENTS: [u8; PDM_NUM_COEFFS * mem::size_of::<u32>()] = [
    0x88, 0x03, 0x00, 0x00, 0x82, 0x08, 0x00, 0x00, 0x51, 0x12, 0x00, 0x00, 0x0a, 0x23, 0x00, 0x00,
    0xce, 0x3d, 0x00, 0x00, 0x97, 0x66, 0x00, 0x00, 0x43, 0xa2, 0x00, 0x00, 0x9c, 0xf6, 0x00, 0x00,
    0x53, 0x6a, 0x01, 0x00, 0xe6, 0x04, 0x02, 0x00, 0x7e, 0xce, 0x02, 0x00, 0xae, 0xcf, 0x03, 0x00,
    0x2e, 0x11, 0x05, 0x00, 0x7d, 0x9b, 0x06, 0x00, 0x75, 0x76, 0x08, 0x00, 0xd8, 0xa8, 0x0a, 0x00,
    0xd2, 0x37, 0x0d, 0x00, 0x82, 0x26, 0x10, 0x00, 0x86, 0x75, 0x13, 0x00, 0x97, 0x22, 0x17, 0x00,
    0x39, 0x28, 0x1b, 0x00, 0x89, 0x7d, 0x1f, 0x00, 0x2e, 0x16, 0x24, 0x00, 0x69, 0xe2, 0x28, 0x00,
    0x56, 0xcf, 0x2d, 0x00, 0x51, 0xc7, 0x32, 0x00, 0x80, 0xb2, 0x37, 0x00, 0x87, 0x77, 0x3c, 0x00,
    0x4c, 0xfc, 0x40, 0x00, 0xd9, 0x26, 0x45, 0x00, 0x47, 0xde, 0x48, 0x00, 0xa0, 0x0b, 0x4c, 0x00,
    0xc1, 0x9a, 0x4e, 0x00, 0x1f, 0x7b, 0x50, 0x00, 0x68, 0xa0, 0x51, 0x00, 0x06, 0x03, 0x52, 0x00,
    0x4a, 0x25, 0x00, 0x00, 0x4c, 0xaf, 0x00, 0x00, 0xc0, 0x07, 0x02, 0x00, 0x45, 0x99, 0x04, 0x00,
    0x9a, 0x84, 0x08, 0x00, 0x7d, 0x38, 0x0d, 0x00, 0x5f, 0x1a, 0x11, 0x00, 0xd9, 0x81, 0x11, 0x00,
    0x80, 0x44, 0x0b, 0x00, 0x8e, 0xe5, 0xfb, 0xff, 0xca, 0x32, 0xe3, 0xff, 0x52, 0xc7, 0xc4, 0xff,
    0xa6, 0xbc, 0xa8, 0xff, 0x83, 0xe6, 0x9a, 0xff, 0xb8, 0x5b, 0xa8, 0xff, 0x6b, 0xae, 0xdb, 0xff,
    0xe7, 0xd8, 0x38, 0x00, 0x24, 0x42, 0xba, 0x00, 0x33, 0x20, 0x50, 0x01, 0x6e, 0xdc, 0xe2, 0x01,
    0x42, 0x23, 0x58, 0x02, 0x2c, 0x50, 0x99, 0x02, 0xcf, 0xfa, 0xff, 0xff, 0x53, 0x0a, 0xff, 0xff,
    0x66, 0x23, 0xfb, 0xff, 0xa0, 0x3e, 0xf4, 0xff, 0xe6, 0x68, 0xf0, 0xff, 0xb8, 0x35, 0xf7, 0xff,
    0x56, 0xec, 0x04, 0x00, 0x37, 0xa3, 0x09, 0x00, 0x00, 0xd4, 0xfe, 0xff, 0x78, 0xa3, 0xf5, 0xff,
    0x03, 0xbf, 0xfe, 0xff, 0x84, 0xd5, 0x0b, 0x00, 0xbe, 0x0b, 0x04, 0x00, 0x52, 0x54, 0xf2, 0xff,
    0x6d, 0x3f, 0xf8, 0xff, 0xc5, 0x7f, 0x0f, 0x00, 0xe6, 0x9e, 0x0c, 0x00, 0x79, 0x03, 0xef, 0xff,
    0xd5, 0x33, 0xed, 0xff, 0xec, 0xd1, 0x11, 0x00, 0x7d, 0x69, 0x1a, 0x00, 0xd6, 0x55, 0xee, 0xff,
    0x88, 0x66, 0xdc, 0xff, 0x57, 0x26, 0x10, 0x00, 0xc7, 0x8d, 0x2e, 0x00, 0x82, 0x2e, 0xf3, 0xff,
    0x63, 0x69, 0xc4, 0xff, 0xcd, 0x08, 0x07, 0x00, 0x35, 0x34, 0x4b, 0x00, 0xaf, 0x21, 0x02, 0x00,
    0x83, 0xb6, 0xa1, 0xff, 0xe2, 0xd5, 0xef, 0xff, 0x94, 0x9b, 0x76, 0x00, 0xf3, 0xd7, 0x25, 0x00,
    0xff, 0xfc, 0x67, 0xff, 0xe3, 0xac, 0xb6, 0xff, 0x52, 0x1b, 0xcc, 0x00, 0x3c, 0x8a, 0x8b, 0x00,
    0x9f, 0x0c, 0xcd, 0xfe, 0x5c, 0x68, 0xcc, 0xfe, 0x4d, 0xc5, 0x98, 0x02, 0x82, 0xcf, 0xfb, 0x06,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];
const FILTER_LENGTHS: u32 = 0x542c47;
const AUDIO_DEV_PDM0: u32 = from_fourcc(b"pdm0");
const AUDIO_DEV_LPAI: u32 = from_fourcc(b"lpai");
const AUDIO_DEV_HPAI: u32 = from_fourcc(b"hpai");
const POWER_STATE_OFF: u32 = from_fourcc(b"idle");
const POWER_STATE_IDLE: u32 = from_fourcc(b"pw1 ");
const POWER_STATE_ON: u32 = from_fourcc(b"pwrd");

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct AudioAttachDevice {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: u64,
    _zero2: u64,
    _pad0: u32,
    len: u64,
    dev_id: u32,
    _pad1: u32,
}

impl AudioAttachDevice {
    fn new(dev_id: u32) -> AudioAttachDevice {
        AudioAttachDevice {
            unk0: 0xFFFFFFFF,
            calltype: CALLTYPE_AUDIO_ATTACH_DEVICE,
            dev_id,
            len: 0x2c,
            ..AudioAttachDevice::default()
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct LpaiChannelConfig {
    unk1: u32,
    unk2: u32,
    unk3: u32,
    unk4: u32,
}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
struct PDMConfig {
    bytes_per_sample: u32,
    clock_source: u32,
    pdm_frequency: u32,
    pdmc_frequency: u32,
    slow_clock_speed: u32,
    fast_clock_speed: u32,
    channel_polarity_select: u32,
    channel_phase_select: u32,
    unk1: u32,
    unk2: u16,
    ratio1: u8,
    ratio2: u8,
    ratio3: u8,
    _pad0: u8,
    filter_lengths: u32,
    coeff_bulk: u32,
    coeffs: [u8; PDM_NUM_COEFFS * mem::size_of::<u32>()],
    unk3: u32,
    mic_turn_on_time_ms: u32,
    _zero0: u64,
    _zero1: u64,
    unk4: u32,
    mic_settle_time_ms: u32,
    _zero2: [u8; 69], // ?????
}

unsafe impl Zeroable for PDMConfig {}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
struct DecimatorConfig {
    latency: u32,
    ratio1: u8,
    ratio2: u8,
    ratio3: u8,
    _pad0: u8,
    filter_lengths: u32,
    coeff_bulk: u32,
    coeffs: [u8; PDM_NUM_COEFFS * mem::size_of::<u32>()],
}

unsafe impl Zeroable for DecimatorConfig {}

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
struct PowerSetting {
    dev_id: u32,
    cookie: u32,
    _unk0: u32,
    _zero0: u64,
    target_pstate: u32,
    unk1: u32,
    _zero1: [u8; 20],
}

impl PowerSetting {
    fn new(dev_id: u32, cookie: u32, target_pstate: u32, unk1: u32) -> PowerSetting {
        PowerSetting {
            dev_id,
            cookie,
            target_pstate,
            unk1,
            ..PowerSetting::default()
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default, Debug)]
struct AudioSetDeviceProp<T> {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: u64,
    _zero2: u64,
    _pad0: u32,
    len: u64,
    dev_id: u32,
    modifier: u32,
    len2: u32,
    data: T,
}

impl<T: Default> AudioSetDeviceProp<T> {
    fn new(dev_id: u32, modifier: u32, data: T) -> AudioSetDeviceProp<T> {
        AudioSetDeviceProp {
            unk0: 0xFFFFFFFF,
            calltype: CALLTYPE_AUDIO_SET_PROP,
            dev_id,
            modifier,
            len: mem::size_of::<T>() as u64 + 0x30,
            len2: mem::size_of::<T>() as u32,
            data,
            ..AudioSetDeviceProp::default()
        }
    }
}

unsafe impl<T> Zeroable for AudioSetDeviceProp<T> {}

impl<T: Zeroable> AudioSetDeviceProp<T> {
    fn try_init<E>(
        dev_id: u32,
        modifier: u32,
        data: impl Init<T, E>,
    ) -> impl Init<AudioSetDeviceProp<T>, Error>
    where
        Error: From<E>,
    {
        try_init!(
            AudioSetDeviceProp {
                unk0: 0xFFFFFFFF,
                calltype: CALLTYPE_AUDIO_SET_PROP,
                dev_id,
                modifier,
                len: mem::size_of::<T>() as u64 + 0x30,
                len2: mem::size_of::<T>() as u32,
                data <- data,
                ..Zeroable::init_zeroed()
            }
        )
    }
}

struct SndSocAopData {
    dev: ARef<device::Device>,
    adata: Arc<dyn AOP>,
    service: EPICService,
    pstate_cookie: AtomicU32,
    fwnode: ARef<FwNode>,
}

impl SndSocAopData {
    fn new(
        dev: ARef<device::Device>,
        adata: Arc<dyn AOP>,
        service: EPICService,
        fwnode: ARef<FwNode>,
    ) -> Result<Arc<SndSocAopData>> {
        Ok(Arc::new(
            SndSocAopData {
                dev,
                adata,
                service,
                fwnode,
                pstate_cookie: AtomicU32::new(1),
            },
            GFP_KERNEL,
        )?)
    }
    fn set_pdm_config(&self) -> Result<()> {
        let pdm_cfg = init!(PDMConfig {
            bytes_per_sample: 2,
            clock_source: 0x706c6c20, // 'pll '
            pdm_frequency: 2400000,
            pdmc_frequency: 24000000,
            slow_clock_speed: 24000000,
            fast_clock_speed: 24000000,
            channel_polarity_select: 256,
            channel_phase_select: 0,
            unk1: 0xf7600,
            unk2: 0,
            ratio1: DECIMATION_RATIOS[0],
            ratio2: DECIMATION_RATIOS[1],
            ratio3: DECIMATION_RATIOS[2],
            filter_lengths: FILTER_LENGTHS,
            coeff_bulk: PDM_NUM_COEFFS as u32,
            coeffs: COEFFICIENTS,
            unk3: 1,
            mic_turn_on_time_ms: 20,
            unk4: 1,
            mic_settle_time_ms: 50,
            ..Zeroable::init_zeroed()
        });
        let set_prop = AudioSetDeviceProp::<PDMConfig>::try_init(AUDIO_DEV_PDM0, 200, pdm_cfg);
        let msg = KBox::try_init(set_prop, GFP_KERNEL)?;
        let ret = self.epic_wrapped_call(msg.as_ref())?;
        if ret != 0 {
            dev_err!(self.dev, "Unable to set pdm config, return code {}", ret);
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn set_decimator_config(&self) -> Result<()> {
        let pdm_cfg = init!(DecimatorConfig {
            latency: 15,
            ratio1: DECIMATION_RATIOS[0],
            ratio2: DECIMATION_RATIOS[1],
            ratio3: DECIMATION_RATIOS[2],
            filter_lengths: FILTER_LENGTHS,
            coeff_bulk: PDM_NUM_COEFFS as u32,
            coeffs: COEFFICIENTS,
            ..Zeroable::init_zeroed()
        });
        let set_prop =
            AudioSetDeviceProp::<DecimatorConfig>::try_init(AUDIO_DEV_PDM0, 210, pdm_cfg);
        let msg = KBox::try_init(set_prop, GFP_KERNEL)?;
        let ret = self.epic_wrapped_call(msg.as_ref())?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set decimator config, return code {}",
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn set_lpai_channel_cfg(&self) -> Result<()> {
        let cfg = LpaiChannelConfig {
            unk1: 7,
            unk2: 7,
            unk3: 1,
            unk4: 7,
        };
        let msg = AudioSetDeviceProp::new(AUDIO_DEV_LPAI, 301, cfg);
        let ret = self.epic_wrapped_call(&msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set lpai channel config, return code {}",
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn audio_attach_device(&self, dev_id: u32) -> Result<()> {
        let msg = AudioAttachDevice::new(dev_id);
        let ret = self.epic_wrapped_call(&msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to attach device {:?}, return code {}",
                dev_id,
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn set_audio_power(&self, pstate: u32, unk1: u32) -> Result<()> {
        let set_pstate = PowerSetting::new(
            AUDIO_DEV_HPAI,
            self.pstate_cookie.fetch_add(1, Ordering::Relaxed),
            pstate,
            unk1,
        );
        let msg = AudioSetDeviceProp::new(AUDIO_DEV_HPAI, 202, set_pstate);
        let ret = self.epic_wrapped_call(&msg)?;
        if ret != 0 {
            dev_err!(
                self.dev,
                "Unable to set power state {:?}, return code {}",
                pstate,
                ret
            );
            return Err(EIO);
        } else {
            Ok(())
        }
    }
    fn epic_wrapped_call<T>(&self, data: &T) -> Result<u32> {
        let msg_bytes =
            unsafe { slice::from_raw_parts(data as *const T as *const u8, mem::size_of::<T>()) };
        self.adata
            .epic_call(&self.service, EPIC_SUBTYPE_WRAPPED_CALL, msg_bytes)
    }
    fn request_dma_channel(&self) -> Result<*mut bindings::dma_chan> {
        let res = unsafe {
            from_err_ptr(bindings::dma_request_chan(
                self.dev.as_raw(),
                c_str!("dma").as_ptr() as _,
            ))
        };
        if res.is_err() {
            dev_err!(self.dev, "Unable to get dma channel");
        }
        res
    }
}

#[repr(transparent)]
struct SndSocAopDriver(*mut bindings::snd_card);

fn copy_str(target: &mut [u8], source: &[u8]) {
    for i in 0..source.len() {
        target[i] = source[i];
    }
}

unsafe fn dmaengine_slave_config(
    chan: *mut bindings::dma_chan,
    config: *mut bindings::dma_slave_config,
) -> i32 {
    unsafe {
        match (*(*chan).device).device_config {
            Some(dc) => dc(chan, config),
            None => ENOSYS.to_errno(),
        }
    }
}

unsafe extern "C" fn aop_hw_params(
    substream: *mut bindings::snd_pcm_substream,
    params: *mut bindings::snd_pcm_hw_params,
) -> i32 {
    let chan = unsafe { bindings::snd_dmaengine_pcm_get_chan(substream) };
    let mut slave_config = bindings::dma_slave_config::default();
    let ret =
        unsafe { bindings::snd_hwparams_to_dma_slave_config(substream, params, &mut slave_config) };
    if ret < 0 {
        return ret;
    }
    slave_config.src_port_window_size = 4;
    unsafe { dmaengine_slave_config(chan, &mut slave_config) }
}

unsafe extern "C" fn aop_pcm_open(substream: *mut bindings::snd_pcm_substream) -> i32 {
    let data = unsafe { Arc::<SndSocAopData>::borrow((*substream).private_data.cast()) };
    if let Err(e) = data.set_audio_power(POWER_STATE_IDLE, 0) {
        dev_err!(data.dev, "Unable to enter 'pw1 ' state");
        return e.to_errno();
    }
    let mut hwparams = bindings::snd_pcm_hardware {
        info: bindings::SNDRV_PCM_INFO_MMAP
            | bindings::SNDRV_PCM_INFO_MMAP_VALID
            | bindings::SNDRV_PCM_INFO_INTERLEAVED,
        formats: bindings::BINDINGS_SNDRV_PCM_FMTBIT_FLOAT_LE,
        subformats: 0,
        rates: bindings::SNDRV_PCM_RATE_48000,
        rate_min: 48000,
        rate_max: 48000,
        channels_min: 3,
        channels_max: 3,
        periods_min: 2,
        buffer_bytes_max: usize::MAX,
        period_bytes_max: 0x4000,
        periods_max: u32::MAX,
        period_bytes_min: 256,
        fifo_size: 16,
    };
    let dma_chan = match data.request_dma_channel() {
        Ok(dc) => dc,
        Err(e) => return e.to_errno(),
    };

    if unsafe { (*substream).dma_buffer.dev.type_ == bindings::SNDRV_DMA_TYPE_UNKNOWN as _ } {
        let ret = unsafe {
            bindings::snd_pcm_set_managed_buffer(
                substream,
                bindings::SNDRV_DMA_TYPE_DEV_IRAM as i32,
                (*(*dma_chan).device).dev,
                0,
                0,
            )
        };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate dma buffers");
            unsafe {
                bindings::dma_release_channel(dma_chan);
            }
            return ret;
        }
    }

    let ret = unsafe {
        let mut dai_data = bindings::snd_dmaengine_dai_dma_data::default();
        bindings::snd_dmaengine_pcm_refine_runtime_hwparams(
            substream,
            &mut dai_data,
            &mut hwparams,
            dma_chan,
        )
    };
    if ret != 0 {
        dev_err!(data.dev, "Unable to refine hwparams");
        return ret;
    }
    if let Err(e) = data.set_audio_power(POWER_STATE_ON, 1) {
        dev_err!(data.dev, "Unable to power mic on");
        return e.to_errno();
    }
    unsafe {
        (*(*substream).runtime).hw = hwparams;
        bindings::snd_dmaengine_pcm_open(substream, dma_chan)
    }
}

unsafe extern "C" fn aop_pcm_prepare(_: *mut bindings::snd_pcm_substream) -> i32 {
    0
}

unsafe extern "C" fn aop_pcm_close(substream: *mut bindings::snd_pcm_substream) -> i32 {
    let data = unsafe { Arc::<SndSocAopData>::borrow((*substream).private_data.cast()) };
    if let Err(e) = data.set_audio_power(POWER_STATE_IDLE, 1) {
        dev_err!(data.dev, "Unable to power mic off");
        return e.to_errno();
    }
    let ret = unsafe { bindings::snd_dmaengine_pcm_close_release_chan(substream) };
    if ret != 0 {
        dev_err!(data.dev, "Unable to close channel");
        return ret;
    }
    if let Err(e) = data.set_audio_power(POWER_STATE_OFF, 0) {
        dev_err!(data.dev, "Unable to enter 'idle' power state");
        return e.to_errno();
    }
    0
}

unsafe extern "C" fn aop_pcm_free_private(pcm: *mut bindings::snd_pcm) {
    unsafe {
        Arc::<SndSocAopData>::from_foreign((*pcm).private_data.cast());
    }
}

impl SndSocAopDriver {
    const VTABLE: bindings::snd_pcm_ops = bindings::snd_pcm_ops {
        open: Some(aop_pcm_open),
        close: Some(aop_pcm_close),
        prepare: Some(aop_pcm_prepare),
        trigger: Some(bindings::snd_dmaengine_pcm_trigger),
        pointer: Some(bindings::snd_dmaengine_pcm_pointer),
        ioctl: None,
        hw_params: Some(aop_hw_params),
        hw_free: None,
        sync_stop: None,
        get_time_info: None,
        fill_silence: None,
        copy: None,
        page: None,
        mmap: None,
        ack: None,
    };
    fn new(data: Arc<SndSocAopData>) -> Result<Self> {
        let mut this = SndSocAopDriver(ptr::null_mut());
        let ret = unsafe {
            bindings::snd_card_new(
                data.dev.as_raw(),
                -1,
                ptr::null(),
                THIS_MODULE.as_ptr(),
                0,
                &mut this.0,
            )
        };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate sound card");
            return Err(Error::from_errno(ret));
        }
        let chassis = data
            .fwnode
            .property_read::<CString>(c_str!("apple,chassis-name"))
            .required_by(&data.dev)?;
        let machine_kind = data
            .fwnode
            .property_read::<CString>(c_str!("apple,machine-kind"))
            .required_by(&data.dev)?;
        unsafe {
            let name = b"aop_audio\0";
            let target = (*this.0).driver.as_mut();
            copy_str(target, name.as_ref());
        }
        unsafe {
            let prefix = b"Apple";
            let target = (*this.0).id.as_mut();
            copy_str(target, prefix.as_ref());
            let mut ptr = prefix.len();
            copy_str(&mut target[ptr..], chassis.to_bytes_with_nul());
            ptr += chassis.count_bytes();
            let suffix = b"HPAI\0";
            copy_str(&mut target[ptr..], suffix);
        }
        let longname_suffix = b"High-Power Audio Interface\0";
        let mut machine_name = KVec::with_capacity(
            chassis.count_bytes() + 2 + machine_kind.count_bytes() + longname_suffix.len(),
            GFP_KERNEL,
        )?;
        machine_name.extend_from_slice(machine_kind.to_bytes_with_nul(), GFP_KERNEL)?;
        let last_item = machine_name.len() - 1;
        machine_name[last_item] = b' ';
        machine_name.extend_from_slice(chassis.to_bytes_with_nul(), GFP_KERNEL)?;
        let last_item = machine_name.len() - 1;
        machine_name[last_item] = b' ';
        unsafe {
            let target = (*this.0).shortname.as_mut();
            copy_str(target, machine_name.as_ref());
            let ptr = machine_name.len();
            let suffix = b"HPAI\0";
            copy_str(&mut target[ptr..], suffix);
        }
        machine_name.extend_from_slice(longname_suffix, GFP_KERNEL)?;
        unsafe {
            let target = (*this.0).longname.as_mut();
            copy_str(target, machine_name.as_ref());
        }

        let mut pcm = ptr::null_mut();
        let ret =
            unsafe { bindings::snd_pcm_new(this.0, machine_name.as_ptr() as _, 0, 0, 1, &mut pcm) };
        if ret < 0 {
            dev_err!(data.dev, "Unable to allocate PCM device");
            return Err(Error::from_errno(ret));
        }

        unsafe {
            bindings::snd_pcm_set_ops(
                pcm,
                bindings::SNDRV_PCM_STREAM_CAPTURE as i32,
                &Self::VTABLE,
            );
        }

        unsafe {
            (*pcm).private_data = data.clone().into_foreign() as _;
            (*pcm).private_free = Some(aop_pcm_free_private);
            (*pcm).info_flags = 0;
            let name = c"aop_audio";
            copy_str((*pcm).name.as_mut(), name.to_bytes());
        }

        let ret = unsafe { bindings::snd_card_register(this.0) };
        if ret < 0 {
            dev_err!(data.dev, "Unable to register sound card");
            return Err(Error::from_errno(ret));
        }
        Ok(this)
    }
}

impl Drop for SndSocAopDriver {
    fn drop(&mut self) {
        if self.0 != ptr::null_mut() {
            unsafe {
                bindings::snd_card_free(self.0);
            }
        }
    }
}

unsafe impl Send for SndSocAopDriver {}
unsafe impl Sync for SndSocAopDriver {}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c_str!("apple,aop-audio")), ())]
);

impl platform::Driver for SndSocAopDriver {
    type IdInfo = ();

    const OF_ID_TABLE: Option<of::IdTable<()>> = Some(&OF_TABLE);

    fn probe(pdev: &platform::Device<Core>, _info: Option<&()>) -> impl PinInit<Self, Error> {
        let dev = ARef::<device::Device>::from(pdev.as_ref());
        let parent = pdev.as_ref().parent().unwrap();
        // SAFETY: our parent is AOP, and AopDriver is repr(transparent) for Arc<dyn Aop>
        let adata_ptr = unsafe { Pin::<KBox<Arc<dyn AOP>>>::borrow(parent.get_drvdata()) };
        let adata = (&*adata_ptr).clone();
        // SAFETY: AOP sets the platform data correctly
        let svc = unsafe { *((*dev.as_raw()).platform_data as *const EPICService) };
        let parent_fwnode = parent.fwnode().ok_or(ENOENT)?;
        let fwnode = parent_fwnode
            .get_child_by_name(c_str!("audio"))
            .ok_or(EIO)?;
        let data = SndSocAopData::new(dev, adata, svc, fwnode)?;
        for dev in [AUDIO_DEV_PDM0, AUDIO_DEV_HPAI, AUDIO_DEV_LPAI] {
            data.audio_attach_device(dev)?;
        }
        data.set_lpai_channel_cfg()?;
        data.set_pdm_config()?;
        data.set_decimator_config()?;
        Ok(Self::new(data)?)
    }
}

module_platform_driver! {
    type: SndSocAopDriver,
    name: "snd_soc_apple_aop",
    description: "AOP microphone capture driver",
    license: "Dual MIT/GPL",
    alias: ["platform:snd_soc_apple_aop"],
}
