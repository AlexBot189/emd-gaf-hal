/*
 * frontend.h - Linux userspace frontend
 *
 * Provides:
 *   - INV_MSG → fprintf(stderr, ...)
 *   - MASK_NOTIFY_RAW_ACC_DATA / MASK_NOTIFY_RAW_GYR_DATA
 *   - inv_edmp_gaf_outputs_t (application-level, Q30 format)
 *   - notify_data / notify_raw_data declarations
 *
 * Derived from original MCU frontend.h
 */
#ifndef FRONTEND_H
#define FRONTEND_H

#include <stdio.h>
#include <stdint.h>

/* Access to inv_imu_edmp_gaf_outputs_t data type */
#include "imu/inv_imu_edmp.h"

/* ── Message levels ── */
#define INV_MSG_LEVEL_ERROR   0
#define INV_MSG_LEVEL_WARNING 1
#define INV_MSG_LEVEL_INFO    2
#define INV_MSG_LEVEL_DEBUG   3
#define INV_MSG_LEVEL_VERBOSE 4

#define MSG_LEVEL INV_MSG_LEVEL_INFO

#define INV_MSG(level, fmt, ...) \
    fprintf(stderr, "[%d] " fmt "\n", level, ##__VA_ARGS__)

/* ── MASK_ defines (from original frontend.h) ── */
#define MASK_NOTIFY_RAW_ACC_DATA 0x01 /** raw accelerometer data from IMU sensor register */
#define MASK_NOTIFY_RAW_GYR_DATA 0x02 /** raw gyroscope data from IMU sensor register */

/** Add capability to enable/disable GAF */
#define FRONTEND_CONFIG_GAF 0x00000001
/** Add capability to enable/disable SIF */
#define FRONTEND_CONFIG_SIF 0x00000002
/** Add capability to enable/disable TAP */
#define FRONTEND_CONFIG_TAP 0x00000004
/** Add capability to enable/disable FREEFALL */
#define FRONTEND_CONFIG_FREEFALL 0x00000008
/** Add capability to enable/disable B2S */
#define FRONTEND_CONFIG_B2S 0x00000010
/** Add capability to provide advanced configuration of GAF */
#define FRONTEND_CONFIG_GAF_CFG 0x00000020
/** Add capability to get raw mag as it would be acquired for GAF */
#define FRONTEND_CONFIG_GAF_RMAG 0x00000040

/*
 * Application-level GAF outputs (Q30 format).
 * Converted from inv_imu_edmp_gaf_outputs_t (Q14/Q12 format) in sensor_event_cb.
 */
typedef struct {
	/** 6-axis (accel and gyro fusion) quaternion */
	int32_t grv_quat_q30[4];
	uint8_t grv_quat_valid;

	/** 6-axis (accel and mag fusion) quaternion */
	int32_t gmrv_quat_q30[4];
	int32_t gmrv_heading_q27;
	uint8_t gmrv_quat_valid;

	/** 9-axis (accel, gyro and mag fusion) quaternion */
	int32_t rv_quat_q30[4];
	int32_t rv_heading_q27;
	uint8_t rv_quat_valid;

	/** Calibrated accelerometer (1 g = 1<<16) */
	int32_t acc_cal_q16[3];
	uint8_t acc_cal_valid;

	/** Calibrated gyroscope (1 dps = 1<<16) */
	int32_t gyr_cal_q16[3];
	/** Gyro bias (1 dps = 1<<16)*/
	int32_t gyr_bias_q16[3];
	/** Gyro accuracy, from 0 (non calibrated) to 3 (well calibrated) */
	int8_t gyr_accuracy_flag;
	/** Stationary detection based on gyro data */
	int8_t  stationary_flag;
	uint8_t gyr_bias_valid;
	uint8_t gyr_flags_valid;

	int16_t raw_mag[3];
	uint8_t rmag_valid;

	/** Calibrated Magnetometer (1 uT = 1<<16) */
	int32_t mag_cal_q16[3];
	/** Mag bias (1 uT = 1<<16)*/
	int32_t mag_bias_q16[3];
	/** Mag accuracy, from 0 (non calibrated) to 3 (well calibrated) */
	int8_t  mag_accuracy_flag;
	int8_t  mag_anomaly;
	uint8_t mag_bias_valid;

	inv_imu_auto_mrm_state_t mrm_state;
	uint8_t                  mrm_evt_chg_st;
	uint8_t                  mrm_evt_exe_mrm;
	uint8_t                  mrm_evt_exc_thr;
	uint8_t                  mrm_state_valid;

	/** Temperature value (1 degree Celsius = 1<<16)*/
	int32_t temp_degC_q16;
	uint8_t temperature_valid;
} inv_edmp_gaf_outputs_t;

#endif /* FRONTEND_H */
