/*
 * File      : MP_Baro_DPS368.h
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

#ifndef MYPILOT_MP_BARO_DPS368_H
#define MYPILOT_MP_BARO_DPS368_H

#include <rtthread.h>
#include <rtdevice.h>

/* DPS368 supports both addresses, selected by SDO. */
#define MP_BARO_DPS368_I2C_ADDR_LOW       0x76U

/* DPS368 supports both addresses, this default address. */
#define MP_BARO_DPS368_I2C_ADDR_HIGH      0x77U

/* output rate settings. */
enum mp_baro_dps368_rate
{
    MP_BARO_DPS368_RATE_1_HZ = 0,
    MP_BARO_DPS368_RATE_2_HZ,
    MP_BARO_DPS368_RATE_4_HZ,
    MP_BARO_DPS368_RATE_8_HZ,
    MP_BARO_DPS368_RATE_16_HZ,
    MP_BARO_DPS368_RATE_32_HZ,
    MP_BARO_DPS368_RATE_64_HZ,
    MP_BARO_DPS368_RATE_128_HZ
};

/* oversampling osr settings. */
enum mp_baro_dps368_oversampling
{
    MP_BARO_DPS368_OSR_1 = 0,
    MP_BARO_DPS368_OSR_2,
    MP_BARO_DPS368_OSR_4,
    MP_BARO_DPS368_OSR_8,
    MP_BARO_DPS368_OSR_16,
    MP_BARO_DPS368_OSR_32,
    MP_BARO_DPS368_OSR_64,
    MP_BARO_DPS368_OSR_128
};

typedef struct mp_baro_dps368_config_t
{
    rt_uint8_t pressure_rate;
    rt_uint8_t pressure_oversampling;
    rt_uint8_t temperature_rate;
    rt_uint8_t temperature_oversampling;
};

typedef struct mp_baro_dps368_report_t
{
    float pressure_pa;
    float temperature_c; 
    rt_uint64_t timestamp_us;
    rt_uint32_t sequence;
};

typedef struct mp_baro_dps368_health_t
{
    rt_uint8_t healthy;
    rt_uint8_t initialized;
    rt_uint8_t product_id;
    rt_uint8_t revision_id;
    rt_uint32_t transfer_errors;
};

/* Arguments:
 * RESET      : none
 * GET_ID     : rt_uint8_t[2] = {product id, revision id}
 * GET_HEALTH : struct mp_baro_dps368_health *
 * SET_CONFIG : const struct mp_baro_dps368_config *
 * GET_CONFIG : struct mp_baro_dps368_config *
 * 
 */
#define MP_BARO_DPS368_CTRL_RESET         0x1001     /* Device Control Commands */
#define MP_BARO_DPS368_CTRL_GET_ID        0x1002
#define MP_BARO_DPS368_CTRL_GET_HEALTH    0x1003
#define MP_BARO_DPS368_CTRL_SET_CONFIG    0x1004
#define MP_BARO_DPS368_CTRL_GET_CONFIG    0x1005

#ifdef __cplusplus
extern "C" {
#endif

/* Register one DPS368 as an RT-Thread character device.
 * The I2C bus must already have been registered. The sensor is initialized by
 * rt_device_open(), through the normal RT-Thread device init callback.
 */
rt_err_t mp_baro_dps368_register(const char *device_name,
                                 const char *i2c_bus_name,
                                 rt_uint8_t address);

#ifdef __cplusplus
}

struct rt_i2c_bus_device;

class MP_Baro_DPS368
{
public:
    MP_Baro_DPS368();
    ~MP_Baro_DPS368();

    rt_err_t register_device(const char *device_name,
                             const char *i2c_bus_name,
                             rt_uint8_t address);
    rt_err_t init();
    rt_err_t reset();
    rt_err_t read(struct mp_baro_dps368_report_t &report);
    rt_err_t set_config(const struct mp_baro_dps368_config_t &config);
    void get_health(struct mp_baro_dps368_health_t &health) const;

private:

    /* 温度与气压补偿多项式参数 */
    typedef struct Calibration_t
    {
        rt_int16_t c0;
        rt_int16_t c1;
        rt_int32_t c00;
        rt_int32_t c10;
        rt_int16_t c01;
        rt_int16_t c11;
        rt_int16_t c20;
        rt_int16_t c21;
        rt_int16_t c30;
        rt_uint8_t temperature_source;
    };

    rt_err_t initialize_unlocked();
    rt_err_t configure_unlocked(const struct mp_baro_dps368_config_t &config);
    rt_err_t read_calibration();
    rt_err_t wait_sensor_ready(rt_int32_t timeout_ms);
    rt_err_t read_registers(rt_uint8_t reg, rt_uint8_t *data, rt_size_t length);
    rt_err_t write_register(rt_uint8_t reg, rt_uint8_t value);
    rt_err_t update_register(rt_uint8_t reg, rt_uint8_t clear_mask, rt_uint8_t set_mask);
    rt_err_t read_raw(rt_int32_t &pressure, rt_int32_t &temperature);

    void compensate(rt_int32_t raw_pressure, rt_int32_t raw_temperature,
                    float &pressure_pa, float &temperature_c) const;

    /* static functions belong to ClassName , don't have this pointer */
    static rt_int32_t sign_extend(rt_uint32_t value, rt_uint8_t bits);
    static bool config_is_valid(const struct mp_baro_dps368_config &config);
    static rt_uint64_t timestamp_us();

public:
    /* Exposed only for the RT-Thread static operations table. */
    static rt_err_t device_init(rt_device_t device);
    static rt_err_t device_open(rt_device_t device, rt_uint16_t oflag);
    static rt_err_t device_close(rt_device_t device);
    static rt_ssize_t device_read(rt_device_t device, rt_off_t pos,
                                  void *buffer, rt_size_t size);
    static rt_err_t device_control(rt_device_t device, int command, void *args);

private:
    struct rt_device _device;
    struct rt_i2c_bus_device *_bus;
    struct rt_mutex _lock;
    Calibration_t _calibration;
    struct mp_baro_dps368_config_t _config;
    rt_uint32_t _sequence;
    rt_uint32_t _transfer_errors;
    rt_uint8_t _address;
    rt_uint8_t _product_id;
    rt_uint8_t _revision_id;
    bool _registered;
    bool _lock_initialized;
    bool _initialized;
    bool _healthy;
};

#endif /* __cplusplus */
#endif /* MYPILOT_MP_BARO_DPS368_H */
