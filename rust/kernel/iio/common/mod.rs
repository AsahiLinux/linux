// SPDX-License-Identifier: GPL-2.0 OR MIT

//! IIO common modules

#[cfg(any(
    CONFIG_IIO_AOP_SENSOR_LAS = "y",
    CONFIG_IIO_AOP_SENSOR_ALS = "m",
    CONFIG_IIO_AOP_SENSOR_LAS = "y",
    CONFIG_IIO_AOP_SENSOR_ALS = "m",
))]
pub mod aop_sensors;
