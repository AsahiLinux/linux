// SPDX-License-Identifier: GPL-2.0 OR MIT

//! IIO common modules

#[cfg(any(CONFIG_IIO_AOP_SENSOR_LAS, CONFIG_IIO_AOP_SENSOR_ALS,))]
pub mod aop_sensors;
