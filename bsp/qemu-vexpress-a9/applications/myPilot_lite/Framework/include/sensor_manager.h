/*
 * File      : sensor_manager.h
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
 * 2026-10-08     JiaVerso      first commit.
 */

#ifndef __SENSOR_MANAGER_H__
#define __SENSOR_MANAGER_H__

#include <rtthread.h>
#include <stdint.h>

#include "MP_Baro_DPS368.h"

#ifdef USE_LIDAR
	#ifdef BLUEJAY	
		#define USE_LIDAR_I2C
	#else
		#define USE_LIDAR_PWM
	#endif
#endif

//#define ACC_DEVICE_NAME			"lsm303d"
#define ACC_DEVICE_NAME			"mpu6000"
#ifdef USE_EXTERNAL_MAG_DEV
#define MAG_DEVICE_NAME			"hmc5883"
#else
#define MAG_DEVICE_NAME			"lsm303d"
#endif
//#define GYR_DEVICE_NAME			"l3gd20h"
#define GYR_DEVICE_NAME			"mpu6000"
#define BARO_DEVICE_NAME		"baro"
#define GPS_DEVICE_NAME			"gps"
#define LIDAR_DEVICE_NAME		"lidar"

#define GYR_ACC_UPDATE_INTERVAL		2
#define MAG_UPDATE_INTERVAL			10

#define GYR_RAW_POS					0
#define GYR_SCALE_POS				1
#define ACC_RAW_POS					2
#define ACC_SCALE_POS				3
#define MAG_RAW_POS					4
#define MAG_SCLAE_POS				5

/* control cmd */

//common cmd
#define SENSOR_GET_DEVICE_ID		0x00

//acc,mag cmd
#define SENSOR_SET_ACC_RANGE		0x01
#define SENSOR_SET_ACC_SAMPLERATE	0x02
#define SENSOR_SET_MAG_RANGE		0x03
#define SENSOR_SET_MAG_SAMPLERATE	0x04

//gyr cmd
#define SENSOR_SET_GYR_RANGE		0x20

  /* imu channels configuration */
typedef struct {
    const char*    name;            
    rt_err_t     (*get_data)(float data[3]);
    void         (*filter_input)(const float* data);
    const float* (*filter_output)(void);
    int            raw_topic;
    int            filt_topic;
    uint32_t       err_cnt;
} Imu_Channel_t;

typedef struct {
    float pressure_pa;
    float temperature_c;
    
    float altitude;		/* NED: 向下为正，向上为负 */

    uint32_t time_stamp;
    uint32_t sequence;
} Baro_Report_Def;

typedef struct
{
	float altitude;
	float velocity;
	uint32_t time_stamp;
}Baro_Position_t;

typedef enum
{
	GPS_UNDETECTED,
	GPS_AVAILABLE,
	GPS_INAVAILABLE
}Gps_Status_Def;

typedef struct
{
	Gps_Status_Def status;
	uint8_t fix_cnt;
}Gps_Status_t;
    
typedef struct
{
	Vector3f_t velocity;
	Vector3f_t last_pos;
}Gps_Driv_Vel_t;

rt_err_t device_sensor_init(void);
void sensor_manager_init(void);
void sensor_loop(void *parameter);

extern float _lidar_dis;
extern uint32_t _lidar_recv_stamp;

/* acc API */
rt_err_t sensor_acc_raw_measure(int16_t acc[3]);
rt_err_t sensor_acc_measure(float acc[3]);
rt_err_t sensor_acc_get_calibrated_data(float acc[3]);

/* mag API */
rt_err_t sensor_mag_raw_measure(int16_t mag[3]);
rt_err_t sensor_mag_measure(float mag[3]);
rt_err_t sensor_mag_get_calibrated_data(float mag[3]);
bool sensor_mag_get_update_flag(void);
void sensor_mag_clear_update_flag(void);

/* gyr API */
rt_err_t sensor_gyr_raw_measure(int16_t gyr[3]);
rt_err_t sensor_gyr_measure(float gyr[3]);
rt_err_t sensor_gyr_get_calibrated_data(float gyr[3]);

/* barometer API */
rt_err_t sensor_baro_update(void);
Baro_Report_Def* sensor_baro_get_report(void);
Baro_Position_t sensor_baro_get_position(void);

bool sensor_baro_get_update_flag(void);
void sensor_baro_clear_update_flag(void);

/* lidar-lite API */
float lidar_lite_get_dis(void);
bool lidar_lite_is_connect(void);
bool lidar_is_ready(void);
void lidar_lite_store(float dis);

/* gps API */
struct vehicle_gps_position_s gps_get_report(void);
int gps_get_position(Vector3f_t* gps_pos, struct vehicle_gps_position_s gps_report);
int gps_get_velocity(Vector3f_t* gps_vel, struct vehicle_gps_position_s gps_report);
void gps_calc_geometry_distance(Vector3f_t* dis, double lat1, double lon1, double lat2, double lon2);
void gps_calc_geometry_distance2(Vector3f_t* dis, double ref_lat, double ref_lon, double lat, double lon);
void gps_get_status(GPS_Status* gps_sta);

/* common api */
void sensor_collect(void);
void sensor_get_gyr(float gyr[3]);
void sensor_get_acc(float acc[3]);
void sensor_get_mag(float mag[3]);

#endif
