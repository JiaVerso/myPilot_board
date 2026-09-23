/*
 * File      : MP_Baro_DPS368.cpp
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, Fy Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Infineon XENSIV DPS368 pressure sensor driver for RT-Thread.
 *
 * The register layout, coefficient decoding and scaling factors follow the
 * Infineon arduino-xensiv-dps3xx reference driver (DPS368/DPS310 family). 
 * 
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-22     JiaVerso      first commit.
 */
#include "MP_Baro_DPS368.h"

#include <drivers/dev_i2c.h>
#include <math.h>
#include <string.h>

#define DPS368_REG_PRESSURE          0x00U
#define DPS368_REG_PRESSURE_CFG      0x06U
#define DPS368_REG_TEMPERATURE_CFG   0x07U
#define DPS368_REG_MEAS_CFG          0x08U
#define DPS368_REG_CFG               0x09U
#define DPS368_REG_RESET             0x0CU
#define DPS368_REG_PRODUCT_ID        0x0DU
#define DPS368_REG_COEFFICIENTS      0x10U
#define DPS368_REG_COEF_SOURCE       0x28U

#define DPS368_PRODUCT_ID_MASK       0x0FU
#define DPS368_EXPECTED_PRODUCT_ID   0x00U
#define DPS368_REVISION_ID_MASK      0xF0U
#define DPS368_REVISION_ID_SHIFT     4U

#define DPS368_MEAS_PRESSURE_READY   (1U << 4)
#define DPS368_MEAS_SENSOR_READY     (1U << 6)
#define DPS368_MEAS_COEF_READY       (1U << 7)
#define DPS368_MEAS_CONTINUOUS_BOTH  0x07U

#define DPS368_CFG_PRESSURE_SHIFT    (1U << 2)     /* 数据位移使能位 */
#define DPS368_CFG_TEMPERATURE_SHIFT (1U << 3)
#define DPS368_SOFT_RESET            0x09U       
#define DPS368_TEMPERATURE_EXTERNAL  (1U << 7)

#define DPS368_READY_TIMEOUT_MS      100
#define DPS368_RESET_DELAY_MS        50
#define DPS368_MAX_TRANSFER_ERRORS   3U

static const float dps368_scaling_factors[8] =
{
    524288.0f,
    1572864.0f,
    3670016.0f,
    7864320.0f,
    253952.0f,
    516096.0f,
    1040384.0f,
    2088960.0f
};

/**
 * operations set for device object
 */
static const struct rt_device_ops dps368_device_ops =
{
    MP_Baro_DPS368::device_init,
    MP_Baro_DPS368::device_open,
    MP_Baro_DPS368::device_close,
    MP_Baro_DPS368::device_read,
    RT_NULL,
    MP_Baro_DPS368::device_control
};

MP_Baro_DPS368::MP_Baro_DPS368()
    : _bus(RT_NULL),
      _sequence(0),
      _transfer_errors(0),
      _address(MP_BARO_DPS368_I2C_ADDR_HIGH),
      _product_id(0),
      _revision_id(0),
      _registered(false),
      _lock_initialized(false),
      _initialized(false),
      _healthy(false)
{
    rt_memset(&_device, 0, sizeof(_device));
    rt_memset(&_calibration, 0, sizeof(_calibration));

    /* A feasible flight-control profile: 32 Hz pressure at 16x and
     * 4 Hz temperature at 8x.  Unlike 32 Hz/16x on both channels, this
     * remains below the conversion-time limit stated by Infineon.
     */
    _config.pressure_rate = MP_BARO_DPS368_RATE_32_HZ;
    _config.pressure_oversampling = MP_BARO_DPS368_OSR_16;
    _config.temperature_rate = MP_BARO_DPS368_RATE_4_HZ;
    _config.temperature_oversampling = MP_BARO_DPS368_OSR_8;
}

MP_Baro_DPS368::~MP_Baro_DPS368()
{
    if (_lock_initialized)
    {
        rt_mutex_detach(&_lock);
    }
}

rt_err_t MP_Baro_DPS368::register_device(const char *device_name,
                                         const char *i2c_bus_name,
                                         rt_uint8_t address)
{
    rt_err_t result;

    if (_registered || device_name == RT_NULL || i2c_bus_name == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (address != MP_BARO_DPS368_I2C_ADDR_LOW &&
        address != MP_BARO_DPS368_I2C_ADDR_HIGH)
    {
        return -RT_EINVAL;
    }
    if (rt_device_find(device_name) != RT_NULL)
    {
        return -RT_EBUSY;
    }

    _bus = rt_i2c_bus_device_find(i2c_bus_name);
    if (_bus == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    result = rt_mutex_init(&_lock, device_name, RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        _bus = RT_NULL;
        return result;
    }
    _lock_initialized = true;
    _address = address;

    _device.type = RT_Device_Class_Sensor;
    _device.ops = &dps368_device_ops;
    _device.user_data = this;

    result = rt_device_register(&_device, device_name,
                                RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);
    if (result != RT_EOK)
    {
        rt_mutex_detach(&_lock);
        _lock_initialized = false;
        _bus = RT_NULL;
        return result;
    }

    _registered = true;
    return RT_EOK;
}

rt_err_t MP_Baro_DPS368::init()
{
    rt_err_t result;

    if (!_lock_initialized || _bus == RT_NULL)
    {
        return -RT_ENOSYS;
    }

    result = rt_mutex_take(&_lock, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        return result;
    }
    result = initialize_unlocked();
    rt_mutex_release(&_lock);
    return result;
}

rt_err_t MP_Baro_DPS368::initialize_unlocked()
{
    rt_uint8_t id;
    rt_err_t result;

    _initialized = false;
    _healthy = false;

    /* Reset both the sensor and its coefficient state, then wait for NVM. */
    result = write_register(DPS368_REG_RESET, DPS368_SOFT_RESET);
    if (result != RT_EOK)
    {
        return result;
    }
    rt_thread_mdelay(DPS368_RESET_DELAY_MS);

    result = wait_sensor_ready(DPS368_READY_TIMEOUT_MS);
    if (result != RT_EOK)
    {
        return result;
    }

    result = read_registers(DPS368_REG_PRODUCT_ID, &id, 1);
    if (result != RT_EOK)
    {
        return result;
    }
    _product_id = id & DPS368_PRODUCT_ID_MASK;
    _revision_id = (id & DPS368_REVISION_ID_MASK) >> DPS368_REVISION_ID_SHIFT;
    if (_product_id != DPS368_EXPECTED_PRODUCT_ID)
    {
        return -RT_ERROR;
    }

    result = read_calibration();
    if (result != RT_EOK)
    {
        return result;
    }

    result = configure_unlocked(_config);
    if (result != RT_EOK)
    {
        return result;
    }

    _sequence = 0;
    _transfer_errors = 0;
    _initialized = true;
    _healthy = true;
    return RT_EOK;
}

rt_err_t MP_Baro_DPS368::reset()
{
    return init();
}

rt_err_t MP_Baro_DPS368::wait_sensor_ready(rt_int32_t timeout_ms)
{
    rt_uint8_t status;

    while (timeout_ms-- > 0)
    {
        if (read_registers(DPS368_REG_MEAS_CFG, &status, 1) == RT_EOK &&
            (status & (DPS368_MEAS_SENSOR_READY | DPS368_MEAS_COEF_READY)) ==
            (DPS368_MEAS_SENSOR_READY | DPS368_MEAS_COEF_READY))
        {
            return RT_EOK;
        }
        rt_thread_mdelay(1);
    }
    return -RT_ETIMEOUT;
}

rt_err_t MP_Baro_DPS368::read_calibration()
{
    rt_uint8_t data[18];
    rt_uint8_t source;
    rt_err_t result;

    result = read_registers(DPS368_REG_COEFFICIENTS, data, sizeof(data));
    if (result != RT_EOK)
    {
        return result;
    }

    _calibration.c0 = (rt_int16_t)sign_extend(((rt_uint32_t)data[0] << 4) |
                                              ((rt_uint32_t)data[1] >> 4), 12);
    _calibration.c1 = (rt_int16_t)sign_extend((((rt_uint32_t)data[1] & 0x0FU) << 8) |
                                              data[2], 12);
    _calibration.c00 = sign_extend(((rt_uint32_t)data[3] << 12) |
                                   ((rt_uint32_t)data[4] << 4) |
                                   ((rt_uint32_t)data[5] >> 4), 20);
    _calibration.c10 = sign_extend((((rt_uint32_t)data[5] & 0x0FU) << 16) |
                                   ((rt_uint32_t)data[6] << 8) | data[7], 20);
    _calibration.c01 = (rt_int16_t)sign_extend(((rt_uint32_t)data[8] << 8) | data[9], 16);
    _calibration.c11 = (rt_int16_t)sign_extend(((rt_uint32_t)data[10] << 8) | data[11], 16);
    _calibration.c20 = (rt_int16_t)sign_extend(((rt_uint32_t)data[12] << 8) | data[13], 16);
    _calibration.c21 = (rt_int16_t)sign_extend(((rt_uint32_t)data[14] << 8) | data[15], 16);
    _calibration.c30 = (rt_int16_t)sign_extend(((rt_uint32_t)data[16] << 8) | data[17], 16);

    result = read_registers(DPS368_REG_COEF_SOURCE, &source, 1);
    if (result != RT_EOK)
    {
        return result;
    }
    _calibration.temperature_source = source & DPS368_TEMPERATURE_EXTERNAL;
    return RT_EOK;
}

bool MP_Baro_DPS368::config_is_valid(const struct mp_baro_dps368_config &config)
{
    rt_uint32_t pressure_busy;
    rt_uint32_t temperature_busy;

    if (config.pressure_rate > 7U || config.pressure_oversampling > 7U ||
        config.temperature_rate > 7U || config.temperature_oversampling > 7U)
    {
        return false;
    }

    /* Infineon expresses this formula in 0.1 ms units. Leave 10 ms/s
     * margin, matching its reference driver.
     */
    pressure_busy = (20UL << config.pressure_rate) +
                    (16UL << (config.pressure_oversampling + config.pressure_rate));
    temperature_busy = (20UL << config.temperature_rate) +
                       (16UL << (config.temperature_oversampling + config.temperature_rate));
    return (pressure_busy + temperature_busy) < 9900UL;
}

rt_err_t MP_Baro_DPS368::configure_unlocked(const struct mp_baro_dps368_config &config)
{
    rt_uint8_t cfg;
    rt_err_t result;

    if (!config_is_valid(config))
    {
        return -RT_EINVAL;
    }

    /* Stop conversion while changing rate and resolution. */
    result = update_register(DPS368_REG_MEAS_CFG, 0x07U, 0x00U);
    if (result != RT_EOK)
    {
        return result;
    }

    result = write_register(DPS368_REG_PRESSURE_CFG,
                            (rt_uint8_t)((config.pressure_rate << 4) |
                                         config.pressure_oversampling));
    if (result != RT_EOK)
    {
        return result;
    }
    result = write_register(DPS368_REG_TEMPERATURE_CFG,
                            (rt_uint8_t)(_calibration.temperature_source |
                                         (config.temperature_rate << 4) |
                                         config.temperature_oversampling));
    if (result != RT_EOK)
    {
        return result;
    }

    cfg = 0;
    if (config.pressure_oversampling > MP_BARO_DPS368_OSR_8)
    {
        cfg |= DPS368_CFG_PRESSURE_SHIFT;
    }
    if (config.temperature_oversampling > MP_BARO_DPS368_OSR_8)
    {
        cfg |= DPS368_CFG_TEMPERATURE_SHIFT;
    }
    result = update_register(DPS368_REG_CFG,
                             DPS368_CFG_PRESSURE_SHIFT | DPS368_CFG_TEMPERATURE_SHIFT,
                             cfg);
    if (result != RT_EOK)
    {
        return result;
    }

    result = update_register(DPS368_REG_MEAS_CFG, 0x07U,
                             DPS368_MEAS_CONTINUOUS_BOTH);
    if (result == RT_EOK)
    {
        _config = config;
    }
    return result;
}

rt_err_t MP_Baro_DPS368::set_config(const struct mp_baro_dps368_config_t &config)
{
    rt_err_t result;

    if (!_initialized)
    {
        return -RT_ENOSYS;
    }
    result = rt_mutex_take(&_lock, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        return result;
    }
    result = configure_unlocked(config);
    if (result != RT_EOK)
    {
        _healthy = false;
    }
    rt_mutex_release(&_lock);
    return result;
}

rt_err_t MP_Baro_DPS368::read(struct mp_baro_dps368_report_t &report)
{
    rt_uint8_t status;
    rt_int32_t raw_pressure;
    rt_int32_t raw_temperature;
    rt_err_t result;

    if (!_initialized)
    {
        return -RT_ENOSYS;
    }

    result = rt_mutex_take(&_lock, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        return result;
    }

    result = read_registers(DPS368_REG_MEAS_CFG, &status, 1);
    if (result == RT_EOK && !(status & DPS368_MEAS_PRESSURE_READY))
    {
        result = -RT_EBUSY;
    }
    if (result == RT_EOK)
    {
        result = read_raw(raw_pressure, raw_temperature);
    }
    if (result == RT_EOK)
    {
        compensate(raw_pressure, raw_temperature,
                   report.pressure_pa, report.temperature_c);

        if (!isfinite(report.pressure_pa) || !isfinite(report.temperature_c) ||
            report.pressure_pa < 30000.0f || report.pressure_pa > 120000.0f ||
            report.temperature_c < -60.0f || report.temperature_c > 120.0f)
        {
            result = -RT_ERROR;
        }
    }

    if (result == RT_EOK)
    {
        report.timestamp_us = timestamp_us();
        report.sequence = ++_sequence;
        _transfer_errors = 0;
        _healthy = true;
    }
    else if (result != -RT_EBUSY)
    {
        ++_transfer_errors;
        if (_transfer_errors >= DPS368_MAX_TRANSFER_ERRORS)
        {
            _healthy = false;
        }
    }

    rt_mutex_release(&_lock);
    return result;
}

rt_err_t MP_Baro_DPS368::read_raw(rt_int32_t &pressure, rt_int32_t &temperature)
{
    rt_uint8_t data[6];
    rt_err_t result = read_registers(DPS368_REG_PRESSURE, data, sizeof(data));

    if (result != RT_EOK)
    {
        return result;
    }
    pressure = sign_extend(((rt_uint32_t)data[0] << 16) |
                           ((rt_uint32_t)data[1] << 8) | data[2], 24);
    temperature = sign_extend(((rt_uint32_t)data[3] << 16) |
                              ((rt_uint32_t)data[4] << 8) | data[5], 24);
    return RT_EOK;
}

 /* 温度与气压补偿 */
void MP_Baro_DPS368::compensate(rt_int32_t raw_pressure,
                                rt_int32_t raw_temperature,
                                float &pressure_pa,
                                float &temperature_c) const  /* 不修改成员函数内部的变量 */
{
    const float temperature_scaled =
        (float)raw_temperature / dps368_scaling_factors[_config.temperature_oversampling];
    const float pressure_scaled =
        (float)raw_pressure / dps368_scaling_factors[_config.pressure_oversampling];

    temperature_c = (float)_calibration.c0 * 0.5f +
                    (float)_calibration.c1 * temperature_scaled;

    pressure_pa = (float)_calibration.c00 +
                  pressure_scaled * ((float)_calibration.c10 +
                  pressure_scaled * ((float)_calibration.c20 +
                  pressure_scaled * (float)_calibration.c30)) +
                  temperature_scaled * (float)_calibration.c01 +
                  temperature_scaled * pressure_scaled *
                  ((float)_calibration.c11 +
                  pressure_scaled * (float)_calibration.c21);
}

rt_err_t MP_Baro_DPS368::read_registers(rt_uint8_t reg,
                                        rt_uint8_t *data,
                                        rt_size_t length)
{
    struct rt_i2c_msg messages[2];

    if (_bus == RT_NULL || data == RT_NULL || length == 0 || length > 0xFFFFU)
    {
        return -RT_EINVAL;
    }

	/* write i2c address pointer */
    messages[0].addr = _address;
    messages[0].flags = RT_I2C_WR;
    messages[0].len = 1;
    messages[0].buf = &reg;

    messages[1].addr = _address;
    messages[1].flags = RT_I2C_RD;
    messages[1].len = (rt_uint16_t)length;
    messages[1].buf = data;

    return (rt_i2c_transfer(_bus, messages, 2) == 2) ? RT_EOK : -RT_EIO;
}

rt_err_t MP_Baro_DPS368::write_register(rt_uint8_t reg, rt_uint8_t value)
{
    rt_uint8_t data[2];
    struct rt_i2c_msg message;

    if (_bus == RT_NULL)
    {
        return -RT_ENOSYS;
    }
    data[0] = reg;
    data[1] = value;
    message.addr = _address;
    message.flags = RT_I2C_WR;
    message.len = sizeof(data);
    message.buf = data;

    return (rt_i2c_transfer(_bus, &message, 1) == 1) ? RT_EOK : -RT_EIO;
}

rt_err_t MP_Baro_DPS368::update_register(rt_uint8_t reg,
                                         rt_uint8_t clear_mask,
                                         rt_uint8_t set_mask)
{
    rt_uint8_t value;
    rt_err_t result = read_registers(reg, &value, 1);

    if (result != RT_EOK)
    {
        return result;
    }
    value = (rt_uint8_t)((value & (rt_uint8_t)~clear_mask) | set_mask);
    return write_register(reg, value);
}

rt_int32_t MP_Baro_DPS368::sign_extend(rt_uint32_t value, rt_uint8_t bits)
{
    const rt_uint32_t sign = 1UL << (bits - 1U);
    const rt_uint32_t mask = (1UL << bits) - 1UL;
    value &= mask;
    return (rt_int32_t)((value ^ sign) - sign);
}

rt_uint64_t MP_Baro_DPS368::timestamp_us()
{
    return (rt_uint64_t)rt_tick_get_millisecond() * 1000ULL;
}

void MP_Baro_DPS368::get_health(struct mp_baro_dps368_health_t &health) const     /* 常成员函数 */ 
{																				/* const void --> 返回值是常量 */ 	
    health.healthy = _healthy ? 1U : 0U;
    health.initialized = _initialized ? 1U : 0U;
    health.product_id = _product_id;
    health.revision_id = _revision_id;
    health.transfer_errors = _transfer_errors;
}

// 解引用
rt_err_t MP_Baro_DPS368::device_init(rt_device_t device)
{
    MP_Baro_DPS368 *driver = static_cast<MP_Baro_DPS368 *>(device->user_data);
    return driver != RT_NULL ? driver->init() : -RT_EINVAL;
}

/* dummy operations. */
rt_err_t MP_Baro_DPS368::device_open(rt_device_t device, rt_uint16_t oflag)
{
    (void)device;
    (void)oflag;
    return RT_EOK;
}

/* dummy operations. */
rt_err_t MP_Baro_DPS368::device_close(rt_device_t device)
{
    (void)device;
    return RT_EOK;
}

rt_ssize_t MP_Baro_DPS368::device_read(rt_device_t device, rt_off_t pos,
                                       void *buffer, rt_size_t size)
{
    MP_Baro_DPS368 *driver = static_cast<MP_Baro_DPS368 *>(device->user_data);
    (void)pos;

    if (driver == RT_NULL || buffer == RT_NULL ||
        size < sizeof(struct mp_baro_dps368_report_t))
    {
        return 0;
    }
    return driver->read(*static_cast<struct mp_baro_dps368_report_t *>(buffer)) == RT_EOK ?
           (rt_ssize_t)sizeof(struct mp_baro_dps368_report_t) : 0;
}

rt_err_t MP_Baro_DPS368::device_control(rt_device_t device, int command, void *args)
{
    MP_Baro_DPS368 *driver = static_cast<MP_Baro_DPS368 *>(device->user_data);

    if (driver == RT_NULL)
    {
        return -RT_EINVAL;
    }

    switch (command)
    {
    case MP_BARO_DPS368_CTRL_RESET:
        return driver->reset();

    case MP_BARO_DPS368_CTRL_GET_ID:
        if (args != RT_NULL)
        {
            rt_uint8_t *id = static_cast<rt_uint8_t *>(args);
            id[0] = driver->_product_id;
            id[1] = driver->_revision_id;
            return RT_EOK;
        }
        break;

    case MP_BARO_DPS368_CTRL_GET_HEALTH:
        if (args != RT_NULL)
        {
            driver->get_health(*static_cast<struct mp_baro_dps368_health_t *>(args));
            return RT_EOK;
        }
        break;

    case MP_BARO_DPS368_CTRL_SET_CONFIG:
        if (args != RT_NULL)
        {
            return driver->set_config(*static_cast<const struct mp_baro_dps368_config_t *>(args));
        }
        break;

    case MP_BARO_DPS368_CTRL_GET_CONFIG:
        if (args != RT_NULL)
        {
            *static_cast<struct mp_baro_dps368_config_t *>(args) = driver->_config;
            return RT_EOK;
        }
        break;

    default:
        return -RT_ENOSYS;
    }
    return -RT_EINVAL;
}

extern "C" rt_err_t mp_baro_dps368_register(const char *device_name,
                                             const char *i2c_bus_name,
                                             rt_uint8_t address)
{
    MP_Baro_DPS368 *driver = new MP_Baro_DPS368;
    rt_err_t result;

    if (driver == RT_NULL)
    {
        return -RT_ENOMEM;
    }
    result = driver->register_device(device_name, i2c_bus_name, address);
    if (result != RT_EOK)
    {
        delete driver;
    }
    return result;
}

#ifdef MYPILOT_USING_DPS368
static int mp_baro_dps368_auto_register(void)
{
    rt_err_t result = mp_baro_dps368_register(MYPILOT_DPS368_DEVICE_NAME,
                                               MYPILOT_DPS368_I2C_BUS_NAME,
                                               MYPILOT_DPS368_I2C_ADDRESS);
    if (result != RT_EOK)
    {
        rt_kprintf("dps368: register %s on %s failed: %d\n",
                   MYPILOT_DPS368_DEVICE_NAME,
                   MYPILOT_DPS368_I2C_BUS_NAME,
                   result);
    }
    return result;
}
INIT_APP_EXPORT(mp_baro_dps368_auto_register);
#endif
