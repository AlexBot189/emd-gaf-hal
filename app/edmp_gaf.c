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

/* Driver */
#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"

/* Magnetometer drivers */
#include "invn_mag.h"

/* Frontend */
#include "frontend.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * This example showcases how to configure and use GAF in eDMP.
 * It enables accelerometer and gyroscope by default in Low-Noise mode at 50Hz.
 * It starts eDMP to output GAF values.
 * There is also capability to get and print raw data from sensor registers reading.
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

#define ACCEL_FSR_ENUM (ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G)
#define ACCEL_FSR_G    (4)
#define RAW_ACC_SCALE  (ACCEL_FSR_G /* gee */ * 2 /* / 32768 * (1<<16)) */)

#if INV_IMU_HIGH_FSR_SUPPORTED
#define GYRO_FSR_ENUM (GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS)
#define GYRO_FSR_DPS  (4000)
#else
#define GYRO_FSR_ENUM (GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS)
#define GYRO_FSR_DPS  (2000)
#endif
#define RAW_GYR_SCALE    (GYRO_FSR_DPS /* dps */ * 2 /* / (1<<15) * (1<<16)) */)
#define RAW_GYR_SCALE_HR (4000 /* dps */ / 8 /* / (1<<19) * (1<<16)) */)

#define RAW_MAG_SCALE (4915) /* 0.075 * (2 << 16) */

/*
 * WOM threshold value in mg.
 * 1g/256 resolution (wom_th = mg * 256 / 1000)
 */
#define DEFAULT_WOM_THS_MG 52 >> 2 // 52 mg

/*
 * Handy defines to describe power modes
 */
#define LOW_POWER_MODE 0
#define LOW_NOISE_MODE 1

/* Raw data conversion */

typedef struct op_mode {
	uint32_t dmp_sensor_odr_us;
	uint32_t gaf_pdr_us;
	uint8_t  fusion_enabled;
	struct {
		pwr_mgmt0_accel_mode_t pm;
		union {
			uint32_t bw;
			uint32_t avg;
		} lpf;
	} acc;
	struct {
		pwr_mgmt0_gyro_mode_t pm;
		union {
			uint32_t bw;
			uint32_t avg;
		} lpf;
	} gyr;
	struct {
		uint8_t  is_on;
		uint32_t mag_odr_us;
	} mag;
} op_mode_t;

op_mode_t supported_cfg[] = {
	{
	    // 0: HRC ALN GLN BYPASS 200 HZ MAG 100 HZ HRC 100Hz
	    .dmp_sensor_odr_us = 5000,
	    .gaf_pdr_us        = 10000,
	    .fusion_enabled    = 0,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LN,
	    .acc.lpf.bw        = 0,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_LN,
	    .gyr.lpf.bw        = 0,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 10000,
	},
	{
	    // 1: HRC ALP GLP AVG 1x 100 HZ MAG 50 HZ HRC 50Hz
	    .dmp_sensor_odr_us = 10000,
	    .gaf_pdr_us        = 20000,
	    .fusion_enabled    = 0,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LP,
	    .acc.lpf.avg       = 1,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_LP,
	    .gyr.lpf.avg       = 1,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 20000,
	},
	{
	    // 2: HRC ALP AVG 1x 100 HZ GYRO OFF MAG 50 HZ HRC 50Hz
	    .dmp_sensor_odr_us = 10000,
	    .gaf_pdr_us        = 20000,
	    .fusion_enabled    = 0,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LP,
	    .acc.lpf.avg       = 1,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_OFF,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 20000,
	},
	{
	    // 3: ALN GLN BYPASS 400 HZ MAG 50 HZ GAF 200 HZ
	    .dmp_sensor_odr_us = 2500,
	    .gaf_pdr_us        = 5000,
	    .fusion_enabled    = 1,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LN,
	    .acc.lpf.bw        = 0,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_LN,
	    .gyr.lpf.bw        = 0,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 20000,
	},
	{
	    // 4: ALP GLP AVG 1x 100 HZ MAG 50 HZ GAF 50 HZ
	    .dmp_sensor_odr_us = 10000,
	    .gaf_pdr_us        = 20000,
	    .fusion_enabled    = 1,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LP,
	    .acc.lpf.avg       = 1,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_LP,
	    .gyr.lpf.avg       = 1,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 20000,
	},
	{
	    // 5: ALN GLN BYPASS 100 HZ MAG 50 HZ GAF 50 HZ
	    .dmp_sensor_odr_us = 10000,
	    .gaf_pdr_us        = 20000,
	    .fusion_enabled    = 1,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LN,
	    .acc.lpf.bw        = 0,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_LN,
	    .gyr.lpf.bw        = 0,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 20000,
	},
	{
	    // 6: ALN GLN BYPASS 400 HZ MAG 50 HZ GAF 50 HZ
	    .dmp_sensor_odr_us = 2500,
	    .gaf_pdr_us        = 20000,
	    .fusion_enabled    = 1,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LN,
	    .acc.lpf.bw        = 0,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_LN,
	    .gyr.lpf.bw        = 0,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 20000,
	},
	{
	    // 7: ALN GLN BYPASS 800 HZ MAG 50 HZ GAF 50 HZ
	    .dmp_sensor_odr_us = 1250,
	    .gaf_pdr_us        = 20000,
	    .fusion_enabled    = 1,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LN,
	    .acc.lpf.bw        = 0,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_LN,
	    .gyr.lpf.bw        = 0,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 20000,
	},
	{
	    // 8: ALP GLP AVG 1x 50 HZ MAG OFF GAF 50 HZ
	    .dmp_sensor_odr_us = 20000,
	    .gaf_pdr_us        = 20000,
	    .fusion_enabled    = 1,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LP,
	    .acc.lpf.avg       = 1,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_LP,
	    .gyr.lpf.avg       = 1,
	    .mag.is_on         = 0,
	},
	{
	    // 9: ALP AVG 1x 100 HZ GYRO OFF MAG 50 HZ GAF 50 HZ
	    .dmp_sensor_odr_us = 10000,
	    .gaf_pdr_us        = 20000,
	    .fusion_enabled    = 1,
	    .acc.pm            = PWR_MGMT0_ACCEL_MODE_LP,
	    .acc.lpf.avg       = 1,
	    .gyr.pm            = PWR_MGMT0_GYRO_MODE_OFF,
	    .mag.is_on         = 1,
	    .mag.mag_odr_us    = 20000,
	},
};

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp; /* Store timestamp when int from IMU fires */
static uint64_t timestamp; /* Store int1_timestamp when IMU IRQ is being processed in main loop */
static uint32_t dmp_odr_us; /* Gyro and DMP ODR in us */
static uint32_t mag_odr_us; /* Mag ODR in us */
static uint32_t gaf_pdr_us; /* GAF PDR in us */
static uint8_t
           fusion_enabled; /* 1 if eDMP outputs quaternion, 0 if only calibration is run by eDMP and only bias are output */
static int power_save_en; /* Indicates power save mode state */

/* Default matrix to be used: identity */
static int8_t mounting_matrix[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
/* An example of a mounting-matrix with -90deg around Y, followed -90 around Z */
//static int8_t mounting_matrix[9] = {0, 0, 1, 1, 0, 0, 0, 1, 0};

/* 
 * Soft-iron matrix applied to mag data in EDMP
 * Align mag axis to IMU
 */
static int32_t soft_iron_matrix[3][3] = {
	{ (1 << 30), 0, 0 },
	{ 0, (1 << 30), 0 },
	{ 0, 0, (1 << 30) },
};

static uint8_t fifo_data[FIFO_MIRRORING_SIZE]; /* Memory where to store FIFO data */

/* Static variables for command interface */
static uint8_t accel_en; /* Indicates accel state */
static uint8_t gyro_en; /* Indicates gyro state */

/* Accelerometer biases to be applied */
static int32_t acc_bias_q16[3];

static uint8_t gyro_is_on;
static uint8_t mag_is_on;

/* 
 * High Resolution configuration
 */
static uint32_t high_res_en;

/* 
 * MRM automatic sequence configuration
 */
static uint32_t mrm_auto_is_on;

/* 
 * FIFO push from DMP configuration
 */
static uint8_t fifo_push_en;

/* 
 * Buffer to hold on MRM IRQ events before being sent to frontend
 */
static inv_imu_edmp_int_state_t mrm_event;

/* 
 * Gyroscope biases and accuracy to be applied
 */
static int16_t gyr_bias_q12[3];
static uint8_t gyr_accuracy;
static int32_t gyr_bias_temperature;

/* 
 * Magnetometer biases and accuracy to be applied
 */
static int32_t mag_bias_q16[3];
static uint8_t mag_accuracy;

/* Indicates if new mag biases shall be ignored (required to verify mag environment with the GUI) */
static int32_t freeze_mag_bias;
static int32_t frozen_bias_mag[3];

/*
 * Output extracted from edmp filling FIFO frames
 */
static inv_edmp_gaf_outputs_t edmp_outputs;

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  stop_algo();
static int  set_operation_mode(const op_mode_t *op_mode);
static int  start_algo();
static void sensor_event_cb(inv_imu_sensor_event_t *event);
static void int_cb(void *context, unsigned int int_num);
static int  on_command_received_cb(inv_edmp_all_algo_commands cmd, void *params,
                                   uint32_t *param_size);
static int  init_imu_biases(int32_t m_bias_q16[3], uint8_t *const m_accuracy, int16_t g_bias_q12[3],
                            uint8_t *const g_accuracy, int32_t *const t_q16);
static int  store_imu_biases(const int32_t m_bias_q16[3], const uint8_t m_accuracy,
                             const int16_t g_bias_q12[3], const uint8_t g_accuracy,
                             const int32_t t_q16);

static uint8_t current_opmode;

/* Main function implementation */
int main(void)
{
	int rc = 0;

	accel_en      = 0;
	gyro_en       = 0;
	power_save_en = 0;

	/* Acceleromater bias are 0 by default */
	acc_bias_q16[0] = 0;
	acc_bias_q16[1] = 0;
	acc_bias_q16[2] = 0;

	rc |= setup_mcu();
	SI_CHECK_RC(rc);

	/* Setup message facility (log) and set callback to be called when a command is received */
	init_frontend(FRONTEND_CONFIG_GAF | FRONTEND_CONFIG_GAF_RMAG | FRONTEND_CONFIG_GAF_CFG,
	              on_command_received_cb);

	INV_MSG(INV_MSG_LEVEL_INFO, "###");
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example EDMP GAF");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	high_res_en    = 1; /* By default high resolution data are built */
	mrm_auto_is_on = 1; /* By default MRM is enabled */
	fifo_push_en   = 1; /* By default FIFO PUSH is enabled */
	current_opmode = 0; /* By default op_mode #0 is selected */
	freeze_mag_bias =
	    0; /* By default, ensure mag bias are taken into account to compute calibrated mag data */

	mrm_event.INV_GAF_MRM_CHG = 0;
	mrm_event.INV_GAF_MRM_RUN = 0;
	mrm_event.INV_GAF_MRM_THR = 0;

	/* By default bias are 0 */
	mag_bias_q16[0] = 0;
	mag_bias_q16[1] = 0;
	mag_bias_q16[2] = 0;

	/* Retrieve from NVM backend */
	rc |= init_imu_biases(mag_bias_q16, &mag_accuracy, gyr_bias_q12, &gyr_accuracy,
	                      &gyr_bias_temperature);
	edmp_outputs.gyr_bias_q16[0]   = gyr_bias_q12[0] << 4;
	edmp_outputs.gyr_bias_q16[1]   = gyr_bias_q12[1] << 4;
	edmp_outputs.gyr_bias_q16[2]   = gyr_bias_q12[2] << 4;
	edmp_outputs.gyr_accuracy_flag = gyr_accuracy;
	edmp_outputs.mag_bias_q16[0]   = mag_bias_q16[0];
	edmp_outputs.mag_bias_q16[1]   = mag_bias_q16[1];
	edmp_outputs.mag_bias_q16[2]   = mag_bias_q16[2];
	edmp_outputs.mag_accuracy_flag = mag_accuracy;

	SI_CHECK_RC(rc);

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	/* Configure sensors */
	rc |= set_operation_mode(&supported_cfg[current_opmode]);
	rc |= start_algo();
	SI_CHECK_RC(rc);

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			inv_imu_int_state_t int_state;

			si_disable_irq();
			/* Clear interrupt flag */
			int1_flag = 0;
			/* Retrieve timestamp */
			timestamp = int1_timestamp;
			si_enable_irq();

			/* Read interrupt status */
			rc |= inv_imu_get_int_status(&imu_dev, INV_IMU_INT1, &int_state);
			SI_CHECK_RC(rc);

			if (int_state.INV_EDMP_EVENT) {
				inv_imu_edmp_int_state_t apex_state = { 0 };
				/* Read APEX interrupt status */
				rc |= inv_imu_edmp_get_int_apex_status(&imu_dev, &apex_state);
				SI_CHECK_RC(rc);
				mrm_event.INV_GAF_MRM_CHG = apex_state.INV_GAF_MRM_CHG;
				mrm_event.INV_GAF_MRM_RUN = apex_state.INV_GAF_MRM_RUN;
				mrm_event.INV_GAF_MRM_THR = apex_state.INV_GAF_MRM_THR;
			}

			if (int_state.INV_FIFO_THS) {
				uint16_t fifo_count;
				rc |= inv_imu_adv_get_data_from_fifo(&imu_dev, fifo_data, &fifo_count);
				SI_CHECK_RC(rc);
				rc |= inv_imu_adv_parse_fifo_data(&imu_dev, fifo_data, fifo_count);
				SI_CHECK_RC(rc);
			}
		}

		rc |= check_received_command();
	} while (rc == 0);

	return rc;
}

/* Initializes MCU peripherals. */
static int setup_mcu()
{
	int rc = 0;

	rc |= si_board_init();

	/* Configure GPIO to call `int_cb` when INT1 fires. */
	rc |= si_init_gpio_int(SI_GPIO_INT1, int_cb);

	/* Init timer peripheral for sleep and get_time */
	rc |= si_init_timers();

	/* Initialize serial interface between MCU and IMU */
	rc |= si_io_imu_init(SERIF_TYPE);

	/* Initialize flash storage (NVM) to keep biases from one execution to the other */
	rc |= si_flash_storage_init();

	return rc;
}
/* Initializes IMU device and apply configuration. */
static int setup_imu()
{
	int                       rc = 0;
	inv_imu_int_pin_config_t  int_pin_config;
	inv_imu_adv_var_t *       e = (inv_imu_adv_var_t *)imu_dev.adv_var;
	inv_imu_int_state_t       int_config;
	inv_imu_adv_fifo_config_t fifo_config;

	/* Init transport layer */
	imu_dev.transport.read_reg   = si_io_imu_read_reg;
	imu_dev.transport.write_reg  = si_io_imu_write_reg;
	imu_dev.transport.serif_type = SERIF_TYPE;
	imu_dev.transport.sleep_us   = si_sleep_us;

	/* Init sensor event callback */
	e->sensor_event_cb = sensor_event_cb;

	/* Wait 3 ms to ensure device is properly supplied  */
	si_sleep_us(3000);

	/* In SPI, configure slew-rate to prevent bus corruption on DK-SMARTMOTION-REVG */
	if (imu_dev.transport.serif_type == UI_SPI3 || imu_dev.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
		rc |= inv_imu_write_reg(&imu_dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
		SI_CHECK_RC(rc);
		si_sleep_us(2); /* Takes effect 1.5 us after the register is programmed */
	}

	rc |= inv_imu_adv_init(&imu_dev);
	SI_CHECK_RC(rc);

	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
	SI_CHECK_RC(rc);

	/* Set sensor FSR */
	rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_FSR_ENUM);
	rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_FSR_ENUM);
	SI_CHECK_RC(rc);

	/* Interrupts configuration for auto MRM */
	if (mrm_auto_is_on) {
		inv_imu_edmp_int_state_t apex_int_config;
		memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));
		apex_int_config.INV_GAF_MRM_CHG = INV_IMU_ENABLE;
		apex_int_config.INV_GAF_MRM_RUN = INV_IMU_ENABLE;
		apex_int_config.INV_GAF_MRM_THR = INV_IMU_ENABLE;
		rc |= inv_imu_edmp_set_config_int_apex(&imu_dev, &apex_int_config);
	}

	/* Interrupts configuration:
	 * - Enable FIFO interrupt for GRV
	 * - Enable EDMP interrupt for MRM events if auto MRM is loaded */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_FIFO_THS = INV_IMU_ENABLE;
	if (mrm_auto_is_on)
		int_config.INV_EDMP_EVENT = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* 
	 * Configure FIFO so that eDMP can push GAF outputs in ES FIFO frame
	 * Set ES0 format to 9bytes and enable ES1 as well.
	 * Configure FIFO WM to 1 (number of FIFO frame per GAF run).
	 */
	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.base_conf.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;
	fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
	fifo_config.base_conf.fifo_wm_th = 1;
	fifo_config.base_conf.hires_en   = INV_IMU_DISABLE;
	fifo_config.base_conf.gyro_en    = INV_IMU_ENABLE;
	fifo_config.base_conf.accel_en   = INV_IMU_ENABLE;
	fifo_config.fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;
	fifo_config.es1_en               = INV_IMU_ENABLE;
	fifo_config.es0_en               = INV_IMU_ENABLE;
	fifo_config.comp_en              = INV_IMU_DISABLE;
	fifo_config.tmst_fsync_en        = INV_IMU_ENABLE;
	fifo_config.es0_6b_9b            = FIFO_CONFIG4_FIFO_ES0_9B;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	return rc;
}

/* FIFO sensor event callback */
static void sensor_event_cb(inv_imu_sensor_event_t *event)
{
	static int16_t prev_rgyr[3] = { 0 };
	uint8_t        input_mask   = 0;

	if (event->sensor_mask & (1 << INV_SENSOR_ACCEL)) {
		/* Align IMU data with EDMP input data */
		/* we need event->accel to build cal accel even if we don't want to print raw accel data */
		inv_imu_remap_data(event->accel, mounting_matrix);
		if (accel_en) {
			input_mask |= MASK_NOTIFY_RAW_ACC_DATA;
		}
	}

	if (event->sensor_mask & (1 << INV_SENSOR_GYRO)) {
		/* Align IMU data with EDMP input data */
		/* we need event->gyro to build cal gyro even if we don't want to print raw gyro data */
		inv_imu_remap_data(event->gyro, mounting_matrix);
		if (gyro_en) {
			input_mask |= MASK_NOTIFY_RAW_GYR_DATA;
		}
		if (high_res_en) {
			prev_rgyr[0] = event->gyro[0];
			prev_rgyr[1] = event->gyro[1];
			prev_rgyr[2] = event->gyro[2];
		}
	}

	if (((1 << INV_SENSOR_ES0) | (1 << INV_SENSOR_ES1)) ==
	    (event->sensor_mask & ((1 << INV_SENSOR_ES0) | (1 << INV_SENSOR_ES1)))) {
		static inv_imu_edmp_gaf_outputs_t gaf_outputs = { 0 };
		int                               rc;

		rc = inv_imu_edmp_gaf_decode_fifo(&imu_dev, (const uint8_t *)event->es0,
		                                  (const uint8_t *)event->es1, &gaf_outputs);
		if (high_res_en) {
			/* High resolution bits for gyro are available in FIFO only if fusion is not enabled */
			if (!fusion_enabled) {
				/** Decoded 20 bits high resolution gyro value {x, y, z} (4000 dps = 1<<19) */
				static int32_t rgyr_highres[3] = { 0 };

				if (gaf_outputs.hr_g_valid) {
					rc |= inv_imu_edmp_decode_gaf_rgyr_highres(
					    &imu_dev, gaf_outputs.hr_g, GYRO_FSR_ENUM, prev_rgyr, rgyr_highres);
				}
				event->gyro[0] = rgyr_highres[0];
				event->gyro[1] = rgyr_highres[1];
				event->gyro[2] = rgyr_highres[2];
			}
		}

		if (rc == -1) {
			INV_MSG(INV_MSG_LEVEL_ERROR,
			        "Error when rebuilding GAF output, unknown FIFO frame received");
			si_sleep_us(10 * 1000 * 1000);
		} else if (gaf_outputs.frame_complete) {
			edmp_outputs.acc_cal_q16[0] =
			    ((int32_t)event->accel[0] * RAW_ACC_SCALE) - acc_bias_q16[0];
			edmp_outputs.acc_cal_q16[1] =
			    ((int32_t)event->accel[1] * RAW_ACC_SCALE) - acc_bias_q16[1];
			edmp_outputs.acc_cal_q16[2] =
			    ((int32_t)event->accel[2] * RAW_ACC_SCALE) - acc_bias_q16[2];
			edmp_outputs.acc_cal_valid = 1;

			edmp_outputs.grv_quat_valid = gaf_outputs.grv_quat_valid;
			if (edmp_outputs.grv_quat_valid) {
				edmp_outputs.grv_quat_q30[0] = (int32_t)gaf_outputs.grv_quat_q14[0] << 16;
				edmp_outputs.grv_quat_q30[1] = (int32_t)gaf_outputs.grv_quat_q14[1] << 16;
				edmp_outputs.grv_quat_q30[2] = (int32_t)gaf_outputs.grv_quat_q14[2] << 16;
				edmp_outputs.grv_quat_q30[3] = (int32_t)gaf_outputs.grv_quat_q14[3] << 16;
			}
			edmp_outputs.gmrv_quat_valid =
			    gaf_outputs.gmrv_quat_valid && gaf_outputs.gmrv_heading_valid;
			if (edmp_outputs.gmrv_quat_valid) {
				edmp_outputs.gmrv_quat_q30[0] = (int32_t)gaf_outputs.gmrv_quat_q14[0] << 16;
				edmp_outputs.gmrv_quat_q30[1] = (int32_t)gaf_outputs.gmrv_quat_q14[1] << 16;
				edmp_outputs.gmrv_quat_q30[2] = (int32_t)gaf_outputs.gmrv_quat_q14[2] << 16;
				edmp_outputs.gmrv_quat_q30[3] = (int32_t)gaf_outputs.gmrv_quat_q14[3] << 16;
				edmp_outputs.gmrv_heading_q27 = (int32_t)gaf_outputs.gmrv_heading_q11 << 16;
			}
			edmp_outputs.rv_quat_valid = gaf_outputs.rv_quat_valid && gaf_outputs.rv_heading_valid;
			if (edmp_outputs.rv_quat_valid) {
				edmp_outputs.rv_quat_q30[0] = (int32_t)gaf_outputs.rv_quat_q14[0] << 16;
				edmp_outputs.rv_quat_q30[1] = (int32_t)gaf_outputs.rv_quat_q14[1] << 16;
				edmp_outputs.rv_quat_q30[2] = (int32_t)gaf_outputs.rv_quat_q14[2] << 16;
				edmp_outputs.rv_quat_q30[3] = (int32_t)gaf_outputs.rv_quat_q14[3] << 16;
				edmp_outputs.rv_heading_q27 = (int32_t)gaf_outputs.rv_heading_q11 << 16;
			}

			edmp_outputs.gyr_bias_valid = gaf_outputs.gyr_bias_valid;
			if (edmp_outputs.gyr_bias_valid) {
				edmp_outputs.gyr_bias_q16[0] = (int32_t)gaf_outputs.gyr_bias_q12[0] << 4;
				edmp_outputs.gyr_bias_q16[1] = (int32_t)gaf_outputs.gyr_bias_q12[1] << 4;
				edmp_outputs.gyr_bias_q16[2] = (int32_t)gaf_outputs.gyr_bias_q12[2] << 4;
			}
			if (gyro_is_on) { /*no need to update gyro if it was OFF*/
				if (high_res_en) {
					edmp_outputs.gyr_cal_q16[0] =
					    (int32_t)event->gyro[0] * RAW_GYR_SCALE_HR - edmp_outputs.gyr_bias_q16[0];
					edmp_outputs.gyr_cal_q16[1] =
					    (int32_t)event->gyro[1] * RAW_GYR_SCALE_HR - edmp_outputs.gyr_bias_q16[1];
					edmp_outputs.gyr_cal_q16[2] =
					    (int32_t)event->gyro[2] * RAW_GYR_SCALE_HR - edmp_outputs.gyr_bias_q16[2];
				} else {
					edmp_outputs.gyr_cal_q16[0] =
					    (int32_t)event->gyro[0] * RAW_GYR_SCALE - edmp_outputs.gyr_bias_q16[0];
					edmp_outputs.gyr_cal_q16[1] =
					    (int32_t)event->gyro[1] * RAW_GYR_SCALE - edmp_outputs.gyr_bias_q16[1];
					edmp_outputs.gyr_cal_q16[2] =
					    (int32_t)event->gyro[2] * RAW_GYR_SCALE - edmp_outputs.gyr_bias_q16[2];
				}
			}

			edmp_outputs.gyr_flags_valid = gaf_outputs.gyr_flags_valid;
			if (edmp_outputs.gyr_flags_valid) {
				edmp_outputs.gyr_accuracy_flag = gaf_outputs.gyr_accuracy_flag;
				edmp_outputs.stationary_flag   = gaf_outputs.stationary_flag;
			}

			edmp_outputs.mag_bias_valid = gaf_outputs.mag_bias_valid;
			if (edmp_outputs.mag_bias_valid) {
				edmp_outputs.mag_bias_q16[0] = (int32_t)gaf_outputs.mag_bias_q16[0];
				edmp_outputs.mag_bias_q16[1] = (int32_t)gaf_outputs.mag_bias_q16[1];
				edmp_outputs.mag_bias_q16[2] = (int32_t)gaf_outputs.mag_bias_q16[2];
				if (freeze_mag_bias) {
					/* If there was a request to use frozen biases, overwrite the edmp outputs */
					edmp_outputs.mag_bias_q16[0] = frozen_bias_mag[0];
					edmp_outputs.mag_bias_q16[1] = frozen_bias_mag[1];
					edmp_outputs.mag_bias_q16[2] = frozen_bias_mag[2];
				}
				edmp_outputs.mag_accuracy_flag = gaf_outputs.mag_accuracy_flag;
				edmp_outputs.mag_anomaly       = gaf_outputs.mag_anomalies;
			}

			edmp_outputs.rmag_valid = gaf_outputs.rmag_valid;
			if (edmp_outputs.rmag_valid) {
				edmp_outputs.raw_mag[0] = gaf_outputs.rmag[0];
				edmp_outputs.raw_mag[1] = gaf_outputs.rmag[1];
				edmp_outputs.raw_mag[2] = gaf_outputs.rmag[2];
			}

			edmp_outputs.mrm_state_valid = gaf_outputs.mrm_state_valid;
			if (edmp_outputs.mrm_state_valid) {
				edmp_outputs.mrm_state = gaf_outputs.mrm_state;
				if (mrm_event.INV_GAF_MRM_CHG)
					edmp_outputs.mrm_evt_chg_st = 1;
				if (mrm_event.INV_GAF_MRM_RUN)
					edmp_outputs.mrm_evt_exe_mrm = 1;
				if (mrm_event.INV_GAF_MRM_RUN)
					edmp_outputs.mrm_evt_exc_thr = 1;
				mrm_event.INV_GAF_MRM_CHG = 0;
				mrm_event.INV_GAF_MRM_RUN = 0;
				mrm_event.INV_GAF_MRM_THR = 0;
			}

			if (edmp_outputs.mag_bias_valid && edmp_outputs.rmag_valid) {
				edmp_outputs.mag_cal_q16[0] =
				    (int32_t)edmp_outputs.raw_mag[0] * RAW_MAG_SCALE - edmp_outputs.mag_bias_q16[0];
				edmp_outputs.mag_cal_q16[1] =
				    (int32_t)edmp_outputs.raw_mag[1] * RAW_MAG_SCALE - edmp_outputs.mag_bias_q16[1];
				edmp_outputs.mag_cal_q16[2] =
				    (int32_t)edmp_outputs.raw_mag[2] * RAW_MAG_SCALE - edmp_outputs.mag_bias_q16[2];
			}

			edmp_outputs.temp_degC_q16 =
			    (25 * (1UL << 16)) +
			    ((int32_t)event->temperature * 32768); /* 25 + (raw_temp / 2) converted to q16 */
			edmp_outputs.temperature_valid = 1;

			/* If we were to follow edmp output, we'd do:
			 * ```
			 * notify_data(timestamp, &edmp_outputs, event->temperature);
			 * ```
			 * Instead, we maintain a faster ODR for anything that is AG based by 
			 * notifying every time and clearing other flags after each `notify_data()`.
			 */
			memset(&gaf_outputs, 0, sizeof(gaf_outputs));
		}
	}

	notify_raw_data(timestamp, input_mask, event->accel, event->gyro, event->temperature);
	/* Only push edmp data if FIFO push was enabled (otherwise it will keep reporting the last data from edmp that gradually gets out of data) */
	if (fifo_push_en) {
		notify_data(timestamp, &edmp_outputs, event->temperature);
	}
	/* Clear all flags not related to racc/rgyr data to report EDMP events at EDMP ODR, and racc/rgyr/acc/gyr at acc/gyr ODR */
	edmp_outputs.mag_bias_valid  = 0;
	edmp_outputs.rmag_valid      = 0;
	edmp_outputs.mrm_state_valid = 0;
	edmp_outputs.mrm_evt_chg_st  = 0;
	edmp_outputs.mrm_evt_exe_mrm = 0;
	edmp_outputs.mrm_evt_exc_thr = 0;
	edmp_outputs.grv_quat_valid  = 0;
	edmp_outputs.gmrv_quat_valid = 0;
	edmp_outputs.rv_quat_valid   = 0;
	if (0 == gyro_is_on) {
		/* In case gyroscope is OFF, we clear valid flags associate with it as well since it could be a remain from previous opmode */
		edmp_outputs.gyr_bias_valid  = 0;
		edmp_outputs.gyr_flags_valid = 0;
	}
}

static int stop_algo()
{
	int rc = 0;

	rc |= inv_imu_edmp_disable(&imu_dev);

	/* Get latest computed gyro and mag bias */
	rc |=
	    inv_imu_edmp_get_gaf_gyr_bias(&imu_dev, gyr_bias_q12, &gyr_bias_temperature, &gyr_accuracy);
	rc |= inv_imu_edmp_get_gaf_mag_bias(&imu_dev, mag_bias_q16, &mag_accuracy);

	return rc;
}

static int set_operation_mode(const op_mode_t *op_mode)
{
	int rc = 0;

	dmp_odr_us     = op_mode->dmp_sensor_odr_us;
	gaf_pdr_us     = op_mode->gaf_pdr_us;
	mag_odr_us     = op_mode->mag.mag_odr_us;
	fusion_enabled = op_mode->fusion_enabled;
	gyro_is_on     = op_mode->gyr.pm == PWR_MGMT0_GYRO_MODE_OFF ? 0 : 1;
	mag_is_on      = op_mode->mag.is_on;

	rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_OFF);
	rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_OFF);
	/* Flush FIFO to make sure there is no old data */
	rc |= inv_imu_flush_fifo(&imu_dev);
	int1_timestamp = 0;
	int1_flag      = 0;
	SI_CHECK_RC(rc);

	if (mag_is_on) {
		if (invn_mag_init(&imu_dev))
			mag_is_on = 0;
	}

	switch (dmp_odr_us) {
	case 20 * 1000:
		rc |= inv_imu_edmp_set_frequency(&imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ);
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
		rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_50_HZ);
		break;
	case 10 * 1000:
		rc |= inv_imu_edmp_set_frequency(&imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ);
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
		rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_100_HZ);
		break;
	case 5 * 1000:
		rc |= inv_imu_edmp_set_frequency(&imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ);
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_200_HZ);
		rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_200_HZ);
		break;
	case 2500:
		rc |= inv_imu_edmp_set_frequency(&imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ);
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
		rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_400_HZ);
		break;
	case 1250:
		rc |= inv_imu_edmp_set_frequency(&imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ);
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_800_HZ);
		rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_800_HZ);
		break;
	default:
		INV_MSG(INV_MSG_LEVEL_ERROR, "Unknown dmp_odr_us %d, force to default 200Hz", dmp_odr_us);
		dmp_odr_us = 5 * 1000;
		rc |= inv_imu_edmp_set_frequency(&imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ);
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_200_HZ);
		rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_200_HZ);
		break;
	}

	switch (op_mode->acc.lpf.bw) {
	case 0:
		rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER);
		break;
	case 4:
		rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
		break;
	case 8:
		rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8);
		break;
	case 16:
		rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_16);
		break;
	case 32:
		rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_32);
		break;
	case 64:
		rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_64);
		break;
	case 128:
		rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_128);
		break;
	default:
		break;
	}
	switch (op_mode->gyr.lpf.bw) {
	case 0:
		rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_NO_FILTER);
		break;
	case 4:
		rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);
		break;
	case 8:
		rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8);
		break;
	case 16:
		rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_16);
		break;
	case 32:
		rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_32);
		break;
	case 64:
		rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_64);
		break;
	case 128:
		rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_128);
		break;
	default:
		break;
	}

	switch (op_mode->acc.lpf.avg) {
	case 1:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1);
		break;
	case 2:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_2);
		break;
	case 4:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_4);
		break;
	case 5:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_5);
		break;
	case 7:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_7);
		break;
	case 8:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_8);
		break;
	case 10:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_10);
		break;
	case 11:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_11);
		break;
	case 16:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_16);
		break;
	case 18:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_18);
		break;
	case 20:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_20);
		break;
	case 32:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_32);
		break;
	case 64:
		rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_64);
		break;
	default:
		break;
	}
	switch (op_mode->gyr.lpf.avg) {
	case 1:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_1);
		break;
	case 2:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_2);
		break;
	case 4:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_4);
		break;
	case 5:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_5);
		break;
	case 7:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_7);
		break;
	case 8:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_8);
		break;
	case 10:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_10);
		break;
	case 11:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_11);
		break;
	case 16:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_16);
		break;
	case 18:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_18);
		break;
	case 20:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_20);
		break;
	case 32:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_32);
		break;
	case 64:
		rc |= inv_imu_set_gyro_lp_avg(&imu_dev, IPREG_SYS1_REG_170_GYRO_LP_AVG_64);
		break;
	default:
		break;
	}

	/* Force clock configuration to fit I2C master need to have either RCOSC or PLL running */
	if ((op_mode->acc.pm == PWR_MGMT0_ACCEL_MODE_LP) && (gyro_is_on == 0))
		rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
	else
		rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);

	if (op_mode->acc.pm == PWR_MGMT0_ACCEL_MODE_LN)
		rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
	else
		rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);

	if (gyro_is_on) {
		if (op_mode->gyr.pm == PWR_MGMT0_GYRO_MODE_LN)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
		else
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LP);
		/* Wait for gyro startup time */
		si_sleep_us(GYR_STARTUP_TIME_US);
	}
	SI_CHECK_RC(rc);

	return rc;
}

static int start_algo()
{
	int                                 rc = 0;
	inv_imu_edmp_gaf_parameters_t       gaf_params;
	inv_imu_edmp_powersave_parameters_t apex_parameters;

	/* Clear global variable when new algorithm execution (different opmode for instance) starts */
	(void)memset(&edmp_outputs, 0, sizeof(edmp_outputs));

	/* Initialize APEX for GAF */
	rc |= inv_imu_edmp_init(&imu_dev);
	SI_CHECK_RC(rc);

	/* Configure GAF parameters */
	rc |= inv_imu_edmp_get_gaf_parameters(&imu_dev, &gaf_params);
	gaf_params.pdr_us        = gaf_pdr_us;
	gaf_params.run_spherical = fusion_enabled;
	if (mag_is_on)
		gaf_params.mag_dt_us = mag_odr_us;
	else
		gaf_params.mag_dt_us = 0;
	rc |= inv_imu_read_reg(&imu_dev, SW_PLL1_TRIM, 1, (uint8_t *)&gaf_params.clock_variation);

	rc |= inv_imu_edmp_set_gaf_acc_bias(&imu_dev, acc_bias_q16);
	/* Adjust gyro and mag bias, it must be done before inv_imu_edmp_gaf_set_parameters() so that EDMP_ONDEMAND_ENABLE_GAF_BIAS service is executed */
	rc |= inv_imu_edmp_set_gaf_gyr_bias(&imu_dev, gyr_bias_q12, gyr_bias_temperature, gyr_accuracy);
	/* Force quat to converge once new biases are estimated.
	 * Required for scenario such as:
	 * - Biases are estimated with a magnet attached to the device
	 * - GAF is stopped or device rebooted and biases saved in NVM
	 * - Magnet is detached from the device
	 * - GAF starts and loads previous biases (manifestly wrong).
	 * - Once new biases are estimated, quaternion wouldn't converge toward proper heading 
	 *   if initial accuracy flag was different from 0.
	 */
	mag_accuracy = 0;
	rc |= inv_imu_edmp_set_gaf_mag_bias(&imu_dev, mag_bias_q16, mag_accuracy);

	rc |= inv_imu_edmp_set_gaf_parameters(&imu_dev, &gaf_params);
	SI_CHECK_RC(rc);
	INV_MSG(INV_MSG_LEVEL_INFO,
	        "eDMP parameter clock_variation is 0x%hhx corresponding to %+f%% error",
	        gaf_params.clock_variation, ((float)gaf_params.clock_variation / 25.40f));

	/* Configure GAF 6 or 9 axis : AG, AM or AGM*/
	rc |= inv_imu_edmp_set_gaf_mode(&imu_dev, gyro_is_on, mag_is_on);

	/* Configure APEX parameters for power-save mode (WoM needed as well) */
	rc |= inv_imu_edmp_get_powersave_parameters(&imu_dev, &apex_parameters);
	if (power_save_en) {
		apex_parameters.power_save_en = INV_IMU_ENABLE;
		rc |= inv_imu_adv_configure_wom(&imu_dev, DEFAULT_WOM_THS_MG, DEFAULT_WOM_THS_MG,
		                                DEFAULT_WOM_THS_MG, TMST_WOM_CONFIG_WOM_INT_MODE_ANDED,
		                                TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
		rc |= inv_imu_adv_enable_wom(&imu_dev);
	} else {
		apex_parameters.power_save_en = INV_IMU_DISABLE;
		rc |= inv_imu_adv_disable_wom(&imu_dev);
	}
	rc |= inv_imu_edmp_set_powersave_parameters(&imu_dev, &apex_parameters);
	SI_CHECK_RC(rc);

	/* Set EDMP mounting matrix */
	rc |= inv_imu_edmp_set_mounting_matrix(&imu_dev, mounting_matrix);
	if (mag_is_on) {
		/* Configure properly soft iron matrix in eDMP image */
		rc |= inv_imu_edmp_set_gaf_soft_iron_cor_matrix(&imu_dev, soft_iron_matrix);
		rc |= inv_imu_edmp_enable_gaf_soft_iron_cor(&imu_dev);
	}
	SI_CHECK_RC(rc);

	rc |= invn_mag_load_ram_image(&imu_dev, INVN_MAG_USECASE_IMG_OVER_SIF);
	if (mrm_auto_is_on) {
		rc |= invn_mag_enable_automrm(&imu_dev);
	} else {
		rc |= invn_mag_disable_automrm(&imu_dev);
	}

	if (fifo_push_en)
		rc |= inv_imu_edmp_start_gaf_fifo_push(&imu_dev);
	else
		rc |= inv_imu_edmp_stop_gaf_fifo_push(&imu_dev);

	/* Reset FIFO */
	rc |= inv_imu_adv_reset_fifo(&imu_dev);

	/* Reset interrupt flag */
	si_disable_irq();
	int1_flag = 0;
	si_enable_irq();

	/* Enable GAF */
	rc |= inv_imu_edmp_enable_gaf(&imu_dev);
	SI_CHECK_RC(rc);

	/* Enable eDMP */
	rc |= inv_imu_edmp_enable(&imu_dev);

	/* Let dmp see Accel data ready on ISR0. */
	rc |= inv_imu_edmp_unmask_int_src(&imu_dev, INV_IMU_EDMP_INT0,
	                                  EDMP_INT_SRC_ACCEL_DRDY_MASK | EDMP_INT_SRC_GYRO_DRDY_MASK);

	return rc;
}

/* IMU interrupt handler. */
static void int_cb(void *context, unsigned int int_num)
{
	(void)context;

	if (int_num == SI_GPIO_INT1) {
		int1_timestamp = si_get_time_us();
		int1_flag      = 1;
	}
}

/* Get command from user through UART */
static int on_command_received_cb(inv_edmp_all_algo_commands cmd, void *params,
                                  uint32_t *param_size)
{
	int      rc   = 0;
	int32_t *data = (int32_t *)params;

	switch (cmd) {
	case RAW_ACC_TOGGLE:
		/* Toggle accel raw data printing */
		accel_en = !accel_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s accel data printing.", accel_en ? "Enabling" : "Disabling");
		break;
	case RAW_GYR_TOGGLE:
		/* Toggle gyro raw data printing */
		gyro_en = !gyro_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s gyro data printing.", gyro_en ? "Enabling" : "Disabling");
		break;
	case START_STOP_FIFO:
		/* Toggle FIFO USAGE */
		fifo_push_en = (uint8_t)*data;
		if (fifo_push_en)
			rc |= inv_imu_edmp_start_gaf_fifo_push(&imu_dev);
		else
			rc |= inv_imu_edmp_stop_gaf_fifo_push(&imu_dev);
		INV_MSG(INV_MSG_LEVEL_INFO, "%s FIFO push from eDMP.",
		        fifo_push_en ? "Enabling" : "Disabling");
		break;
	case SET_OPMODE:
		if ((uint32_t)*data >= (sizeof(supported_cfg) / sizeof(supported_cfg[0]))) {
			INV_MSG(INV_MSG_LEVEL_ERROR, "Unexpected opmode configuration value %d", *data);
			return INV_ERROR_BAD_ARG;
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "Set OPMODE %d", *data);
			rc |= stop_algo();
			current_opmode = *data;
			rc |= set_operation_mode(&supported_cfg[*data]);
			if (high_res_en && fusion_enabled) {
				INV_MSG(
				    INV_MSG_LEVEL_WARNING,
				    "Gyro high resolution reconstruction not compatible with GAF fusion enabled, forcing high resolution to disable");
				high_res_en = 0;
			}
			rc |= start_algo();
			if (rc == 0) {
				if (supported_cfg[*data].gyr.pm != PWR_MGMT0_GYRO_MODE_OFF)
					INV_MSG(INV_MSG_LEVEL_INFO, "AG %dHz",
					        (1000 * 1000) / supported_cfg[*data].dmp_sensor_odr_us);
				else
					INV_MSG(INV_MSG_LEVEL_INFO, "A %dHz Gyro OFF",
					        (1000 * 1000) / supported_cfg[*data].dmp_sensor_odr_us);
				if (supported_cfg[*data].fusion_enabled)
					INV_MSG(INV_MSG_LEVEL_INFO, "GAF %dHz",
					        (1000 * 1000) / supported_cfg[*data].gaf_pdr_us);
				else
					INV_MSG(INV_MSG_LEVEL_INFO, "HRC %dHz",
					        (1000 * 1000) / supported_cfg[*data].gaf_pdr_us);
				if (supported_cfg[*data].mag.is_on)
					INV_MSG(INV_MSG_LEVEL_INFO, "Mag %dHz",
					        (1000 * 1000) / supported_cfg[*data].mag.mag_odr_us);
				else
					INV_MSG(INV_MSG_LEVEL_INFO, "Mag OFF");
			}
		}
		break;
	case TOGGLE_GYRO_ON:
		if (gyro_is_on) {
			INV_MSG(INV_MSG_LEVEL_INFO, "Gyro OFF");
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_OFF);
			gyro_is_on = 0;
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "Gyro in operation mode original state");
			rc |= inv_imu_set_gyro_mode(&imu_dev, supported_cfg[current_opmode].gyr.pm);
			gyro_is_on = 1;
		}
		rc |= inv_imu_edmp_set_gaf_mode(&imu_dev, gyro_is_on, mag_is_on);
		SI_CHECK_RC(rc);
		break;
	case RUN_MRM:
		INV_MSG(INV_MSG_LEVEL_INFO, "Run MRM");
		rc |= invn_mag_run_mrm(&imu_dev);
		SI_CHECK_RC(rc);
		break;
	case SET_MATRIX:
		INV_MSG(INV_MSG_LEVEL_INFO, "Set new mag matrix");
		rc |= stop_algo();
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 3; j++) {
				soft_iron_matrix[i][j] = data[i * 3 + j];
			}
		}
		rc |= start_algo();
		SI_CHECK_RC(rc);
		break;
	case GET_MATRIX:
		INV_MSG(INV_MSG_LEVEL_INFO, "Get mag matrix");
		*param_size = sizeof(soft_iron_matrix);
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 3; j++) {
				data[i * 3 + j] = soft_iron_matrix[i][j];
			}
		}
		SI_CHECK_RC(rc);
		break;
	case SET_HIGHRES_EN:
		INV_MSG(INV_MSG_LEVEL_INFO, "%s gyro high resolution", *data ? "Enable" : "Disable");
		high_res_en = *data;
		if (high_res_en && fusion_enabled) {
			INV_MSG(
			    INV_MSG_LEVEL_WARNING,
			    "Gyro high resolution reconstruction not compatible with GAF fusion enabled, forcing high resolution to disable");
			high_res_en = 0;
		}
		SI_CHECK_RC(rc);
		break;
	case GET_HIGHRES_EN:
		INV_MSG(INV_MSG_LEVEL_INFO, "Get gyro high resolution config");
		*param_size = sizeof(high_res_en);
		data[0]     = high_res_en;
		SI_CHECK_RC(rc);
		break;
	case FREEZE_MAG_BIAS:
		INV_MSG(INV_MSG_LEVEL_INFO, "%s mag bias", *data ? "Freezing" : "Unfreezing");
		freeze_mag_bias = *data;
		if (*data) {
			frozen_bias_mag[0] = edmp_outputs.mag_bias_q16[0];
			frozen_bias_mag[1] = edmp_outputs.mag_bias_q16[1];
			frozen_bias_mag[2] = edmp_outputs.mag_bias_q16[2];
		}
		break;
	case ALGORITHM_RESET:
		INV_MSG(INV_MSG_LEVEL_INFO, "Reset algo and biases");
		rc |= stop_algo();
		/* Reset bias to 0 */
		gyr_bias_q12[0]      = 0;
		gyr_bias_q12[1]      = 0;
		gyr_bias_q12[2]      = 0;
		gyr_accuracy         = 0;
		gyr_bias_temperature = GAF_DEFAULT_TEMPERATURE_INIT_Q16;
		mag_bias_q16[0]      = 0;
		mag_bias_q16[1]      = 0;
		mag_bias_q16[2]      = 0;
		mag_accuracy         = 0;
		rc |= start_algo();
		SI_CHECK_RC(rc);
		break;
	case ENABLE_AUTO_MRM:
		INV_MSG(INV_MSG_LEVEL_INFO, "Auto MRM is ON");
		mrm_auto_is_on = 1;
		rc |= invn_mag_enable_automrm(&imu_dev);
		SI_CHECK_RC(rc);
		break;
	case DISABLE_AUTO_MRM:
		INV_MSG(INV_MSG_LEVEL_INFO, "Auto MRM is OFF");
		mrm_auto_is_on = 0;
		rc |= invn_mag_disable_automrm(&imu_dev);
		SI_CHECK_RC(rc);
		break;
	case POWER_SAVE_ENABLE:
		power_save_en = 1;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Power Save mode.",
		        power_save_en ? "Enabling" : "Disabling");
		rc |= stop_algo();
		rc |= start_algo();
		SI_CHECK_RC(rc);
		break;
	case POWER_SAVE_DISABLE:
		power_save_en = 0;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Power Save mode.",
		        power_save_en ? "Enabling" : "Disabling");
		rc |= stop_algo();
		rc |= start_algo();
		SI_CHECK_RC(rc);
		break;
	case SAVE_BIAS_NVM:
		INV_MSG(INV_MSG_LEVEL_INFO, "Saving sensor biases to NVM");
		rc |= inv_imu_edmp_get_gaf_gyr_bias(&imu_dev, gyr_bias_q12, &gyr_bias_temperature,
		                                    &gyr_accuracy);
		rc |= inv_imu_edmp_get_gaf_mag_bias(&imu_dev, mag_bias_q16, &mag_accuracy);
		rc |= store_imu_biases(mag_bias_q16, mag_accuracy, gyr_bias_q12, gyr_accuracy,
		                       gyr_bias_temperature);
		break;
	case GAF_ENABLE:
	case GAF_DISABLE:
		/* GAF isn't really enabled/disabled as it is the core feature
		 * of this example. It can end up in this switch when dynamic
		 * protocol based quaternion enabling/disabling occur. Silently
		 * consume these commands instead of ending in the default clause.
		 */
		rc = 0;
		break;
	default:
		INV_MSG(INV_MSG_LEVEL_INFO, "Unknown command : %c", (char)cmd);
		break;
	}

	return rc;
}

static int init_imu_biases(int32_t m_bias_q16[3], uint8_t *const m_accuracy, int16_t g_bias_q12[3],
                           uint8_t *const g_accuracy, int32_t *const t_q16)
{
	int rc;
	/* NVM allows for up to 84B to be stored */
	uint8_t sensor_bias[84] = { 0 };
	rc                      = si_flash_storage_read(sensor_bias);
	if (0 == rc) {
		/* But we only need 6B for gyroscope biases, and 4B for gyroscope biases' temperature and 12B for mag biases*/
		(void)memcpy(g_bias_q12, sensor_bias, 3 * sizeof(g_bias_q12[0]));
		(void)memcpy(t_q16, &sensor_bias[3 * sizeof(g_bias_q12[0])], sizeof(*t_q16));
		(void)memcpy(m_bias_q16, &sensor_bias[3 * sizeof(g_bias_q12[0]) + sizeof(*t_q16)],
		             3 * sizeof(m_bias_q16[0]));

		/* Having previously saved biases gives a better confidence */
		*g_accuracy = 3;
		INV_MSG(INV_MSG_LEVEL_INFO, "Loading calibration from flash:");
		INV_MSG(INV_MSG_LEVEL_INFO, "   - Gyro:  [%f %f %f]dps at %f degC",
		        (float)g_bias_q12[0] / (1 << 12), (float)g_bias_q12[1] / (1 << 12),
		        (float)g_bias_q12[2] / (1 << 12), (float)*t_q16 / (1 << 16));
		*m_accuracy = 3;
		INV_MSG(INV_MSG_LEVEL_INFO, "   - Mag:  [%f %f %f]uT", (float)m_bias_q16[0] / (1 << 16),
		        (float)m_bias_q16[1] / (1 << 16), (float)m_bias_q16[2] / (1 << 16));
	} else {
		/* rc is FLASH_HEADER_MISSING_RC (or 1) if data couldn't be found */
		memset(g_bias_q12, 0, 3 * sizeof(g_bias_q12[0]));
		*g_accuracy = 0;
		*t_q16      = GAF_DEFAULT_TEMPERATURE_INIT_Q16;
		memset(m_bias_q16, 0, 3 * sizeof(m_bias_q16[0]));
		*m_accuracy = 0;
		rc          = INV_IMU_OK;
	}

	return rc;
}

static int store_imu_biases(const int32_t m_bias_q16[3], const uint8_t m_accuracy,
                            const int16_t g_bias_q12[3], const uint8_t g_accuracy,
                            const int32_t t_q16)
{
	int rc = INV_IMU_OK;
	/* NVM allows for up to 84B to be stored */
	uint8_t sensor_bias[84] = { 0 };

	si_flash_storage_read(sensor_bias);

	if (3 == g_accuracy) {
		/* Only save biases that are correctly calibrated */
		(void)memcpy(sensor_bias, g_bias_q12, 3 * sizeof(g_bias_q12[0]));
		(void)memcpy(&sensor_bias[3 * sizeof(g_bias_q12[0])], &t_q16, sizeof(t_q16));
		INV_MSG(INV_MSG_LEVEL_INFO, "Saving calibration in flash:");
		INV_MSG(INV_MSG_LEVEL_INFO, "   - Gyro:  [%f %f %f]dps at %f degC",
		        (float)g_bias_q12[0] / (1 << 12), (float)g_bias_q12[1] / (1 << 12),
		        (float)g_bias_q12[2] / (1 << 12), (float)t_q16 / (1 << 16));
	} else {
		INV_MSG(
		    INV_MSG_LEVEL_WARNING,
		    "Gyroscope biases were not saved due to insufficient accuracy level (was %d, must be 3)",
		    (int32_t)g_accuracy);
	}
	if (3 == m_accuracy) {
		/* Only save biases that are correctly calibrated */
		(void)memcpy(&sensor_bias[3 * sizeof(g_bias_q12[0]) + sizeof(t_q16)], m_bias_q16,
		             3 * sizeof(m_bias_q16[0]));
		INV_MSG(INV_MSG_LEVEL_INFO, "Saving calibration in flash:");
		INV_MSG(INV_MSG_LEVEL_INFO, "   - Mag:  [%f %f %f]uT", (float)m_bias_q16[0] / (1 << 16),
		        (float)m_bias_q16[1] / (1 << 16), (float)m_bias_q16[2] / (1 << 16));
	} else {
		INV_MSG(
		    INV_MSG_LEVEL_WARNING,
		    "Magnetometer biases were not saved due to insufficient accuracy level (was %d, must be 3)",
		    (int32_t)m_accuracy);
	}

	if ((3 == g_accuracy) || (3 == m_accuracy))
		rc |= si_flash_storage_write(sensor_bias);

	return rc;
}