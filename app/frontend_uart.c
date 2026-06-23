/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "frontend.h"
#include "system_interface.h"

/* Math */
#include <math.h>
#include <stdio.h>

/* Helper to convert radian to degrees */
#define RAD_TO_DEG(rad) ((float)rad * 57.2957795131)

/* 
 * Callback to be called when command is received 
 */
int (*command_cb)(inv_edmp_all_algo_commands cmd, void *params, uint32_t *param_size);

/*
 * Static functions definition
 */
static uint8_t check_frontend_config(uint32_t cfg_mask);
static void    fixedpoint_to_float(const int32_t *in, float *out, const uint8_t fxp_shift,
                                   const uint8_t dim);
static void    quaternions_to_angles(const float quat[4], float angles[3]);
static void    print_help(void);
static void    print_current_config(void);
static void    print_frontend_version(void);
static char *  auto_mrm_state_as_string(inv_imu_auto_mrm_state_t mrm_state);

static char convert_tap_axis_to_str(inv_imu_edmp_tap_axis_t axis);
static char convert_tap_dir_to_str(inv_imu_edmp_tap_dir_t dir);

/* Static variables for command interface */
static uint8_t accel_en; /* Indicates accel state */
static uint8_t gyro_en; /* Indicates gyro state */
static uint8_t sif_en; /* Indicates sif state */
static uint8_t gaf_en; /* Indicates gaf state */
static uint8_t b2s_en; /* Indicates b2s state */
static uint8_t tap_en; /* Indicates tap state */
static uint8_t ff_en; /* Indicates freefall state */
static uint8_t lowg_en; /* Indicates lowG state */
static uint8_t highg_en; /* Indicates highG state */
static uint8_t power_save_en; /* Indicates power save mode state */

static uint32_t frontend_config;

static uint32_t current_config;

void init_frontend(uint32_t config, int (*cmd_cb)(inv_edmp_all_algo_commands cmd, void *params,
                                                  uint32_t *param_size))
{
	/* Configure UART for log */
	si_config_uart_for_print(SI_UART_ID_FTDI, MSG_LEVEL);

	command_cb      = cmd_cb;
	frontend_config = config;

	/* Should remain aligned with edmp_gaf.c default value, being 0 */
	current_config = 0;

	/* Reset commands interface states */
	accel_en = 0;
	gyro_en  = 0;
	if (check_frontend_config(FRONTEND_CONFIG_SIF))
		sif_en = 1;
	if (check_frontend_config(FRONTEND_CONFIG_GAF))
		gaf_en = 1;
	if (check_frontend_config(FRONTEND_CONFIG_B2S))
		b2s_en = 1;
	if (check_frontend_config(FRONTEND_CONFIG_TAP))
		tap_en = 1;
	if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
		ff_en    = 1;
		lowg_en  = 0;
		highg_en = 0;
	}
	power_save_en = 0;
}

void notify_b2s(uint64_t time, uint8_t b2s_not_revb2s)
{
	if (!b2s_not_revb2s) {
		INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   B2S REVERSE", time);
		INV_MSG(INV_MSG_LEVEL_INFO, "");
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   B2S", time);
		INV_MSG(INV_MSG_LEVEL_INFO, "");
	}
}

void notify_sif(uint64_t time, char *sif_label, int16_t sif_label_idx)
{
	INV_MSG(INV_MSG_LEVEL_INFO, "  %10llu us  class label = %s(%hd).", time, sif_label,
	        sif_label_idx);
	INV_MSG(INV_MSG_LEVEL_INFO, "");
}

void notify_tap(uint64_t time, uint8_t num_value, uint8_t axis_value, uint8_t direction,
                uint32_t tap_duration_us)
{
	char tap_dir_str[20];
	char tap_str[80];

	snprintf(tap_dir_str, 20, "Direction: %c%c",
	         convert_tap_axis_to_str((inv_imu_edmp_tap_axis_t)axis_value),
	         convert_tap_dir_to_str((inv_imu_edmp_tap_dir_t)direction));

	if (num_value == INV_IMU_EDMP_TAP_DOUBLE)
		snprintf(tap_str, 80, "%s   %s   duration: %lu us", "Double", tap_dir_str,
		         (unsigned long)tap_duration_us);
	else if (num_value == INV_IMU_EDMP_TAP_TRIPLE)
		snprintf(tap_str, 80, "%s   %s   duration: %lu us", "Triple", tap_dir_str,
		         (unsigned long)tap_duration_us);
	else
		snprintf(tap_str, 80, "%s   %s", "Single", tap_dir_str);

	INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   TAP     %s", time, tap_str);
	INV_MSG(INV_MSG_LEVEL_INFO, "");
}

void notify_ff(uint64_t time, uint16_t duration, uint32_t duration_ms)
{
	INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   FREEFALL     duration: %u samples (%u ms)", time,
	        duration, duration_ms);
	INV_MSG(INV_MSG_LEVEL_INFO, "");
}

void notify_lowg(uint64_t time)
{
	if (lowg_en) {
		INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   LOW_G", time);
		INV_MSG(INV_MSG_LEVEL_INFO, "");
	}
}

void notify_highg(uint64_t time)
{
	if (highg_en) {
		INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   HIGH_G", time);
		INV_MSG(INV_MSG_LEVEL_INFO, "");
	}
}

void notify_data(uint64_t time, const inv_edmp_gaf_outputs_t *gaf_outputs, const int16_t rtemp_data)
{
	float acc_g[3];
	float temp;
	(void)rtemp_data;

	if (gaf_outputs->grv_quat_valid) {
		float grv_quat[4], angles_deg_grv[3];
		fixedpoint_to_float(gaf_outputs->grv_quat_q30, grv_quat, 30, 4);
		quaternions_to_angles(grv_quat, angles_deg_grv);
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: GRV=[%f, %f, %f, %f]", time, grv_quat[0], grv_quat[1],
		        grv_quat[2], grv_quat[3]);
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: EulerGRV=[%.2f, %.2f, %.2f]deg", time, angles_deg_grv[0],
		        angles_deg_grv[1], angles_deg_grv[2]);
	}

	if (gaf_outputs->gmrv_quat_valid) {
		float gmrv_quat[4], angles_deg_gmrv[3];
		float gmrv_quat_heading;
		fixedpoint_to_float(gaf_outputs->gmrv_quat_q30, gmrv_quat, 30, 4);
		fixedpoint_to_float(&gaf_outputs->gmrv_heading_q27, &gmrv_quat_heading, 27, 1);
		quaternions_to_angles(gmrv_quat, angles_deg_gmrv);
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: GMRV=[%f, %f, %f, %f] accuracy=%.2fdeg", time,
		        gmrv_quat[0], gmrv_quat[1], gmrv_quat[2], gmrv_quat[3],
		        RAD_TO_DEG(gmrv_quat_heading));
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: EulerGMRV=[%.2f, %.2f, %.2f]deg", time,
		        angles_deg_gmrv[0], angles_deg_gmrv[1], angles_deg_gmrv[2]);
	}

	if (gaf_outputs->rv_quat_valid) {
		float rv_quat[4], angles_deg_rv[3];
		float rv_quat_heading;
		fixedpoint_to_float(gaf_outputs->rv_quat_q30, rv_quat, 30, 4);
		fixedpoint_to_float(&gaf_outputs->rv_heading_q27, &rv_quat_heading, 27, 1);
		quaternions_to_angles(rv_quat, angles_deg_rv);
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: RV=[%f, %f, %f, %f] accuracy=%.2fdeg", time, rv_quat[0],
		        rv_quat[1], rv_quat[2], rv_quat[3], RAD_TO_DEG(rv_quat_heading));
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: EulerRV=[%.2f, %.2f, %.2f]deg", time, angles_deg_rv[0],
		        angles_deg_rv[1], angles_deg_rv[2]);
	}

	if (gaf_outputs->acc_cal_valid) {
		fixedpoint_to_float(gaf_outputs->acc_cal_q16, acc_g, 16, 3);
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: Acc=[%.3f, %.3f, %.3f]g", time, acc_g[0], acc_g[1],
		        acc_g[2]);
	}

	if ((gaf_outputs->gyr_bias_valid) || (gaf_outputs->gyr_flags_valid)) {
		float gyr_dps[3], gyr_bias[3];
		fixedpoint_to_float(gaf_outputs->gyr_cal_q16, gyr_dps, 16, 3);
		fixedpoint_to_float(gaf_outputs->gyr_bias_q16, gyr_bias, 16, 3);
		INV_MSG(
		    INV_MSG_LEVEL_INFO,
		    "%lld: Gyr=[%.3f, %.3f, %.3f]dps GyrBias=[%.3f, %.3f, %.3f]dps Stationary=%d Accuracy=%d",
		    time, gyr_dps[0], gyr_dps[1], gyr_dps[2], gyr_bias[0], gyr_bias[1], gyr_bias[2],
		    (int32_t)gaf_outputs->stationary_flag, (int32_t)gaf_outputs->gyr_accuracy_flag);
	}

	if (gaf_outputs->rmag_valid) {
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: RMag=[%d, %d, %d]", time,
		        (int32_t)gaf_outputs->raw_mag[0], (int32_t)gaf_outputs->raw_mag[1],
		        (int32_t)gaf_outputs->raw_mag[2]);
	}

	/* We must also check for rmag_valid because we report here both mag bias
	 * and cal mag data which are a combination of raw mag and bias mag
	 * and if mag ODR is slower than GAF PDR, we can have rmag_valid = 0
	 * while other fields are valid
	 */
	if (gaf_outputs->mag_bias_valid && gaf_outputs->rmag_valid) {
		float mag_ut[3], mag_bias[3];
		fixedpoint_to_float(gaf_outputs->mag_cal_q16, mag_ut, 16, 3);
		fixedpoint_to_float(gaf_outputs->mag_bias_q16, mag_bias, 16, 3);
		INV_MSG(
		    INV_MSG_LEVEL_INFO,
		    "%lld: Mag=[%.3f, %.3f, %.3f]uT MagBias=[%.3f, %.3f, %.3f]uT MagAnomalies=%d Accuracy=%d",
		    time, mag_ut[0], mag_ut[1], mag_ut[2], mag_bias[0], mag_bias[1], mag_bias[2],
		    (int32_t)gaf_outputs->mag_anomaly, (int32_t)gaf_outputs->mag_accuracy_flag);
	}

	if (gaf_outputs->mrm_state_valid) {
		INV_MSG(INV_MSG_LEVEL_INFO,
		        "%lld: Auto MRM state=[%d:%s] evt_chg_st=[%d] evt_exe_mrm=[%d] evt_exc_thr=[%d]",
		        time, (uint32_t)gaf_outputs->mrm_state,
		        auto_mrm_state_as_string(gaf_outputs->mrm_state),
		        (uint32_t)gaf_outputs->mrm_evt_chg_st, (uint32_t)gaf_outputs->mrm_evt_exe_mrm,
		        (uint32_t)gaf_outputs->mrm_evt_exc_thr);
	}

	if (gaf_outputs->temperature_valid) {
		fixedpoint_to_float(&gaf_outputs->temp_degC_q16, &temp, 16, 1);
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: Temp=%.2f C", time, temp);
	}

	INV_MSG(INV_MSG_LEVEL_INFO, "");
}

void notify_raw_data(uint64_t time, uint8_t input_mask, const int16_t accel_data[3],
                     const int16_t gyro_data[3], const int16_t temp_data)
{
	if (input_mask & MASK_NOTIFY_RAW_ACC_DATA)
		/* IMU data */
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: RAcc=[%d, %d, %d]", time, (uint32_t)accel_data[0],
		        (uint32_t)accel_data[1], (uint32_t)accel_data[2]);

	if (input_mask & MASK_NOTIFY_RAW_GYR_DATA)
		/* IMU data */
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: RGyr=[%d, %d, %d]", time, (uint32_t)gyro_data[0],
		        (uint32_t)gyro_data[1], (uint32_t)gyro_data[2]);

	if ((input_mask & MASK_NOTIFY_RAW_ACC_DATA) || (input_mask & MASK_NOTIFY_RAW_GYR_DATA))
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: RTemp=%d", time, (uint32_t)temp_data);
}

static uint8_t check_frontend_config(uint32_t cfg_mask)
{
	if (frontend_config & cfg_mask) {
		return 1;
	} else {
		return 0;
	}
}

/* Get command from user through UART */
int check_received_command(void)
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	/* Verify if a callback is registered */
	if (command_cb == 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "/!\\No callback registered, can't process commands");
		return -1;
	}

	switch (cmd) {
	case 'a':
		accel_en = !accel_en;
		rc |= command_cb(RAW_ACC_TOGGLE, NULL, NULL);
		SI_CHECK_RC(rc);
		break;
	case 'g':
		if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
			gyro_en = !gyro_en;
			rc |= command_cb(RAW_GYR_TOGGLE, NULL, NULL);
			SI_CHECK_RC(rc);
		}
		break;

	case 'z':
		if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
			gaf_en = 1;
			rc |= command_cb(GAF_ENABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Enabled GAF.");
		}
		break;
	case 'Z':
		if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
			gaf_en = 0;
			rc |= command_cb(GAF_DISABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Disabled GAF.");
		}
		break;
	case '0':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 0;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case '1':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 1;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case '2':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 2;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case '3':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 3;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case '4':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 4;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
			break;
		}
	case '5':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 5;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case '6':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 6;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case '7':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 7;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case '8':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 8;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case '9':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			current_config = 9;
			rc |= command_cb(SET_OPMODE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'V':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			rc |= command_cb(SAVE_BIAS_NVM, NULL, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'm':
		if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
			rc |= command_cb(ENABLE_AUTO_MRM, NULL, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'M':
		if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
			rc |= command_cb(DISABLE_AUTO_MRM, NULL, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'A':
		if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
			rc |= command_cb(RUN_MRM, NULL, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'n':
		if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
			rc |= command_cb(GYRO_LP_ENABLE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'N':
		if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
			rc |= command_cb(GYRO_LP_DISABLE, &current_config, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'e':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			int32_t highres_en = 1;
			rc |= command_cb(SET_HIGHRES_EN, &highres_en, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'E':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			int32_t highres_en = 0;
			rc |= command_cb(SET_HIGHRES_EN, &highres_en, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'j':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			int32_t freeze_en = 1;
			rc |= command_cb(FREEZE_MAG_BIAS, &freeze_en, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'J':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			int32_t freeze_en = 0;
			rc |= command_cb(FREEZE_MAG_BIAS, &freeze_en, NULL);
			SI_CHECK_RC(rc);
		}
		break;
	case 'r':
		if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
			rc |= command_cb(ALGORITHM_RESET, NULL, NULL);
			SI_CHECK_RC(rc);
		}
		break;

	case 's':
		if (check_frontend_config(FRONTEND_CONFIG_SIF)) {
			sif_en = 1;
			rc |= command_cb(SIF_ENABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Enabled SIF.");
		}
		break;
	case 'S':
		if (check_frontend_config(FRONTEND_CONFIG_SIF)) {
			sif_en = 0;
			rc |= command_cb(SIF_DISABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Disabled SIF.");
		}
		break;

	case 't':
		if (check_frontend_config(FRONTEND_CONFIG_TAP)) {
			tap_en = 1;
			rc |= command_cb(TAP_ENABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Enabled TAP.");
		}
		break;
	case 'T':
		if (check_frontend_config(FRONTEND_CONFIG_TAP)) {
			tap_en = 0;
			rc |= command_cb(TAP_DISABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Disabled TAP.");
		}
		break;
	case 'w':
		if (check_frontend_config(FRONTEND_CONFIG_TAP)) {
			rc |= command_cb(SET_WAT_PARAMS, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Configured TAP for Wide Area TAP.");
		}
		break;
	case 'W':
		if (check_frontend_config(FRONTEND_CONFIG_TAP)) {
			rc |= command_cb(SET_REGULAR_TAP_PARAMS, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Configured TAP for regular TAP.");
		}
		break;

	case 'f':
		if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
			ff_en = 1;
			rc |= command_cb(FF_ENABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Enabled FF.");
		}
		break;
	case 'F':
		if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
			ff_en = 0;
			rc |= command_cb(FF_DISABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Disabled FF.");
		}
		break;
	case 'l':
		if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
			lowg_en = 1;
			rc |= command_cb(LOWG_ENABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Enabled LowG.");
		}
		break;
	case 'L':
		if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
			lowg_en = 0;
			rc |= command_cb(LOWG_DISABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Disabled LowG.");
		}
		break;
	case 'i':
		if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
			highg_en = 1;
			rc |= command_cb(HIGHG_ENABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Enabled HighG.");
		}
		break;
	case 'I':
		if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
			highg_en = 0;
			rc |= command_cb(HIGHG_DISABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Disabled HighG.");
		}
		break;

	case 'b':
		if (check_frontend_config(FRONTEND_CONFIG_B2S)) {
			b2s_en = 1;
			rc |= command_cb(B2S_ENABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Enable B2S.");
		}
		break;
	case 'B':
		if (check_frontend_config(FRONTEND_CONFIG_B2S)) {
			b2s_en = 0;
			rc |= command_cb(B2S_DISABLE, NULL, NULL);
			SI_CHECK_RC(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Disable B2S.");
		}
		break;

	case 'p':
		power_save_en = 1;
		rc |= command_cb(POWER_SAVE_ENABLE, NULL, NULL);
		SI_CHECK_RC(rc);
		INV_MSG(INV_MSG_LEVEL_INFO, "Enabled Power save mode.");
		break;
	case 'P':
		power_save_en = 0;
		rc |= command_cb(POWER_SAVE_DISABLE, NULL, NULL);
		SI_CHECK_RC(rc);
		INV_MSG(INV_MSG_LEVEL_INFO, "Disabled Power save mode.");
		break;
	case '+':
		print_frontend_version();
		break;
	case 'c':
		print_current_config();
		break;
	case 'h':
	case 'H':
		print_help();
		break;
	case 0:
		break; /* No command received */
	default:
		INV_MSG(INV_MSG_LEVEL_INFO, "Unknown command : %c", cmd);
		print_help();
		break;
	}

	return rc;
}

/* Help for UART command interface */
static void print_help()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Help");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'a' : toggle raw accel printing");

	if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'g' : toggle raw gyro printing");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'z' : enable GAF algorithm");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'Z' : disable GAF algorithm");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'm' : enable GAF auto MRM");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'M' : disable GAF auto MRM");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'A' : run GAF MRM");
	}
	if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG)) {
		INV_MSG(INV_MSG_LEVEL_INFO, "# '0-9' : set GAF opmode 0 to 9");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'V' : save GAF current biases in non-volatile memory");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'e' : enable gyro high resolution");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'E' : disable gyro high resolution");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'j' : freeze mag bias");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'J' : unfreeze mag bias");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'r' : algorithm soft reset");
	}

	if (check_frontend_config(FRONTEND_CONFIG_SIF)) {
		INV_MSG(INV_MSG_LEVEL_INFO, "# 's' : enable SIF algorithm");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'S' : disable SIF algorithm");
	}

	if (check_frontend_config(FRONTEND_CONFIG_TAP)) {
		INV_MSG(INV_MSG_LEVEL_INFO, "# 't' : enable TAP algorithm");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'T' : disable TAP algorithm");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'w' : reconfigure TAP algorithm for Wide Area Tap");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'W' : reconfigure TAP algorithm for Regular Tap");
	}

	if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'f' : enable FreeFall algorithm");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'F' : disable FreeFall algorithm");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'l' : enable LowG algorithm output from EDMP");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'L' : disable LowG algorithm output from EDMP");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'i' : enable HighG algorithm output from EDMP");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'I' : disable HighG algorithm output from EDMP");
	}

	if (check_frontend_config(FRONTEND_CONFIG_B2S)) {
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'b' : enable B2S algorithm");
		INV_MSG(INV_MSG_LEVEL_INFO, "# 'B' : disable B2S algorithm");
	}

	INV_MSG(INV_MSG_LEVEL_INFO, "# 'p' : enable EDMP power save mode");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'P' : disable EDMP power save mode");

	INV_MSG(INV_MSG_LEVEL_INFO, "# '+' : Print current frontend version");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'c' : Print current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'h' : Print this helper");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */
}

static void print_current_config()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Current configuration is :");
	if (accel_en) {
		INV_MSG(INV_MSG_LEVEL_INFO, "# Raw accel printing ON");
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "# Raw accel printing OFF");
	}
	if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
		if (gyro_en) {
			INV_MSG(INV_MSG_LEVEL_INFO, "# Raw gyro printing ON");
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "# Raw gyro printing OFF");
		}
	}
	if (check_frontend_config(FRONTEND_CONFIG_SIF)) {
		if (sif_en) {
			INV_MSG(INV_MSG_LEVEL_INFO, "# SIF algorithm enabled");
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "# SIF algorithm disabled");
		}
	}
	if (check_frontend_config(FRONTEND_CONFIG_GAF)) {
		if (gaf_en) {
			INV_MSG(INV_MSG_LEVEL_INFO, "# GAF algorithm enabled");
			if (check_frontend_config(FRONTEND_CONFIG_GAF_CFG))
				INV_MSG(INV_MSG_LEVEL_INFO, "# Current configuration is %d", current_config);
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "# GAF algorithm disabled");
		}
	}
	if (check_frontend_config(FRONTEND_CONFIG_B2S)) {
		if (b2s_en) {
			INV_MSG(INV_MSG_LEVEL_INFO, "# B2S algorithm enabled");
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "# B2S algorithm disabled");
		}
	}
	if (check_frontend_config(FRONTEND_CONFIG_TAP)) {
		if (tap_en) {
			INV_MSG(INV_MSG_LEVEL_INFO, "# TAP algorithm enabled");
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "# TAP algorithm disabled");
		}
	}
	if (check_frontend_config(FRONTEND_CONFIG_FREEFALL)) {
		if (ff_en) {
			INV_MSG(INV_MSG_LEVEL_INFO, "# FreeFall algorithm enabled");
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "# FreeFall algorithm disabled");
		}
		if (lowg_en) {
			INV_MSG(INV_MSG_LEVEL_INFO, "# LowG algorithm enabled");
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "# LowG algorithm disabled");
		}
		if (highg_en) {
			INV_MSG(INV_MSG_LEVEL_INFO, "# HighG algorithm enabled");
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "# HighG algorithm disabled");
		}
	}
	if (power_save_en) {
		INV_MSG(INV_MSG_LEVEL_INFO, "# EDMP Power save mode enabled");
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "# EDMP Power save mode disabled");
	}
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */
}

static void print_frontend_version()
{
	char frontend_gaf_str[20];
	char frontend_sif_str[20];
	char frontend_tap_str[20];
	char frontend_ff_str[20];
	char frontend_b2s_str[20];
	char frontend_gafcfg_str[20];
	char frontend_gafrmag_str[20];
	/* Print version without overloading UART */
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Frontend version is :");
	snprintf(frontend_gaf_str, 30, "GAF %s",
	         check_frontend_config(FRONTEND_CONFIG_GAF) ? "ON" : "OFF");
	snprintf(frontend_sif_str, 30, "SIF %s",
	         check_frontend_config(FRONTEND_CONFIG_SIF) ? "ON" : "OFF");
	snprintf(frontend_tap_str, 30, "TAP %s",
	         check_frontend_config(FRONTEND_CONFIG_TAP) ? "ON" : "OFF");
	snprintf(frontend_ff_str, 30, "FREEFALL %s",
	         check_frontend_config(FRONTEND_CONFIG_FREEFALL) ? "ON" : "OFF");
	snprintf(frontend_b2s_str, 30, "B2S %s",
	         check_frontend_config(FRONTEND_CONFIG_B2S) ? "ON" : "OFF");
	snprintf(frontend_gafcfg_str, 30, "GAF_CFG %s",
	         check_frontend_config(FRONTEND_CONFIG_GAF_CFG) ? "ON" : "OFF");
	snprintf(frontend_gafrmag_str, 30, "GAF_RMAG %s",
	         check_frontend_config(FRONTEND_CONFIG_GAF_RMAG) ? "ON" : "OFF");
	INV_MSG(INV_MSG_LEVEL_INFO, "# %s ; %s ; %s ; %s ; %s ; %s ; %s", frontend_gaf_str,
	        frontend_sif_str, frontend_tap_str, frontend_ff_str, frontend_b2s_str,
	        frontend_gafcfg_str, frontend_gafrmag_str);
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	si_sleep_us(2000000); /* Give user some time to read */
}

static char *auto_mrm_state_as_string(inv_imu_auto_mrm_state_t mrm_state)
{
	switch (mrm_state) {
	case INV_IMU_AUTO_MRM_STATE_CHECK_NORM:
		return "CHECK_NORM";
	case INV_IMU_AUTO_MRM_STATE_CHECK_OFFSETS:
		return "CHECK_OFFSETS";
	case INV_IMU_AUTO_MRM_STATE_RUN:
		return "RUN";
	case INV_IMU_AUTO_MRM_STATE_WAIT_STAB:
		return "WAIT_STAB";
	default:
		return "UNKNOWN";
	}
}

static void fixedpoint_to_float(const int32_t *in, float *out, const uint8_t fxp_shift,
                                const uint8_t dim)
{
	int   i;
	float scale = 1.f / (1 << fxp_shift);

	for (i = 0; i < dim; i++)
		out[i] = scale * in[i];
}

static void quaternions_to_angles(const float quat[4], float angles[3])
{
	const float RAD_2_DEG = (180.f / 3.14159265358979f);
	float       rot_matrix[9];

	// quaternion to rotation matrix
	const float dTx  = 2.0f * quat[1];
	const float dTy  = 2.0f * quat[2];
	const float dTz  = 2.0f * quat[3];
	const float dTwx = dTx * quat[0];
	const float dTwy = dTy * quat[0];
	const float dTwz = dTz * quat[0];
	const float dTxx = dTx * quat[1];
	const float dTxy = dTy * quat[1];
	const float dTxz = dTz * quat[1];
	const float dTyy = dTy * quat[2];
	const float dTyz = dTz * quat[2];
	const float dTzz = dTz * quat[3];

	rot_matrix[0] = 1.0f - (dTyy + dTzz);
	rot_matrix[1] = dTxy - dTwz;
	rot_matrix[2] = dTxz + dTwy;
	rot_matrix[3] = dTxy + dTwz;
	rot_matrix[4] = 1.0f - (dTxx + dTzz);
	rot_matrix[5] = dTyz - dTwx;
	rot_matrix[6] = dTxz - dTwy;
	rot_matrix[7] = dTyz + dTwx;
	rot_matrix[8] = 1.0f - (dTxx + dTyy);

	angles[0] = atan2f(-rot_matrix[3], rot_matrix[0]) * RAD_2_DEG;
	angles[1] = atan2f(-rot_matrix[7], rot_matrix[8]) * RAD_2_DEG;
	angles[2] = asinf(-rot_matrix[6]) * RAD_2_DEG;
}

static char convert_tap_axis_to_str(inv_imu_edmp_tap_axis_t axis)
{
	if (axis == INV_IMU_EDMP_TAP_AXIS_X)
		return 'X';
	else if (axis == INV_IMU_EDMP_TAP_AXIS_Y)
		return 'Y';
	else
		return 'Z';
}

static char convert_tap_dir_to_str(inv_imu_edmp_tap_dir_t dir)
{
	if (dir == INV_IMU_EDMP_TAP_DIR_POSITIVE)
		return '+';
	else
		return '-';
}
