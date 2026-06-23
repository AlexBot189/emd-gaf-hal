/*
 * linux_main.c - eMD GAF Linux Userspace Entry Point
 *
 * Port of MCU edmp_gaf.c to Linux userspace.
 * All MCU functions and logic preserved; only HAL calls (si_*) replaced.
 *
 * Build:  mkdir build && cd build && cmake .. && make
 * Run:    ./emd-gaf -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

/* ── eMD IMU Drivers ── */
#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"

/* ── Magnetometer drivers ── */
#include "invn_mag.h"

/* ── HAL (replaces MCU system_interface.h) ── */
#include "system_interface.h"

/* ── Frontend (inv_edmp_gaf_outputs_t, MASK_ defines, INV_MSG) ── */
#include "frontend.h"

/*
 * Select communication link: I2C
 */
#define SERIF_TYPE UI_I2C

#define ACCEL_FSR_ENUM (ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G)
#define ACCEL_FSR_G    (4)
#define RAW_ACC_SCALE  (ACCEL_FSR_G /* gee */ * 2 /* / 32768 * (1<<16)) */)

/* High FSR not supported by ICM45608, use ±2000dps */
#define GYRO_FSR_ENUM (GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS)
#define GYRO_FSR_DPS  (2000)
#define RAW_GYR_SCALE    (GYRO_FSR_DPS /* dps */ * 2 /* / (1<<15) * (1<<16)) */)
#define RAW_GYR_SCALE_HR (4000 /* dps */ / 8 /* / (1<<19) * (1<<16)) */)

#define RAW_MAG_SCALE (4915) /* 0.075 * (2 << 16) */

/*
 * WOM threshold value in mg.
 * 1g/256 resolution (wom_th = mg * 256 / 1000)
 */
#define DEFAULT_WOM_THS_MG 52 >> 2 /* 52 mg */

/*
 * Handy defines to describe power modes
 */
#define LOW_POWER_MODE 0
#define LOW_NOISE_MODE 1

/* MASK_ definitions (from original frontend.h) */
#define MASK_NOTIFY_RAW_ACC_DATA 0x01
#define MASK_NOTIFY_RAW_GYR_DATA 0x02

/* ── Default config ── */
#define DEFAULT_I2C_DEV   "/dev/i2c-3"
#define DEFAULT_GPIO_CHIP "gpiochip4"
#define DEFAULT_GPIO_LINE 2
#define DEFAULT_IMU_ADDR  0x68

/* ── Operating modes (10 modes, from MCU edmp_gaf.c) ── */
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
	    /* 0: HRC ALN GLN BYPASS 200 HZ MAG 100 HZ HRC 100Hz */
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
	    /* 1: HRC ALP GLP AVG 1x 100 HZ MAG 50 HZ HRC 50Hz */
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
	    /* 2: HRC ALP AVG 1x 100 HZ GYRO OFF MAG 50 HZ HRC 50Hz */
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
	    /* 3: ALN GLN BYPASS 400 HZ MAG 50 HZ GAF 200 HZ */
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
	    /* 4: ALP GLP AVG 1x 100 HZ MAG 50 HZ GAF 50 HZ */
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
	    /* 5: ALN GLN BYPASS 100 HZ MAG 50 HZ GAF 50 HZ */
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
	    /* 6: ALN GLN BYPASS 400 HZ MAG 50 HZ GAF 50 HZ */
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
	    /* 7: ALN GLN BYPASS 800 HZ MAG 50 HZ GAF 50 HZ */
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
	    /* 8: ALP GLP AVG 1x 50 HZ MAG OFF GAF 50 HZ */
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
	    /* 9: ALP AVG 1x 100 HZ GYRO OFF MAG 50 HZ GAF 50 HZ */
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

/* ── Static variables (from MCU edmp_gaf.c) ── */
static inv_imu_device_t  imu_dev;                     /* Driver structure */
static volatile int      int1_flag;                   /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp;              /* Store timestamp when int from IMU fires */
static uint64_t timestamp;                            /* Store int1_timestamp when IMU IRQ is being processed in main loop */
static uint32_t dmp_odr_us;                           /* Gyro and DMP ODR in us */
static uint32_t mag_odr_us;                           /* Mag ODR in us */
static uint32_t gaf_pdr_us;                           /* GAF PDR in us */
static uint8_t  fusion_enabled;                       /* 1 if eDMP outputs quaternion, 0 if only calibration is run by eDMP and only bias are output */
static int      power_save_en;                        /* Indicates power save mode state */

/* Default matrix to be used: identity */
static int8_t mounting_matrix[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

/*
 * Soft-iron matrix applied to mag data in EDMP
 * Align mag axis to IMU
 */
static int32_t soft_iron_matrix[3][3] = {
	{ (1 << 30), 0, 0 },
	{ 0, (1 << 30), 0 },
	{ 0, 0, (1 << 30) },
};

static uint8_t fifo_data[FIFO_MIRRORING_SIZE];       /* Memory where to store FIFO data */

/* Static variables for command interface */
static uint8_t accel_en;                              /* Indicates accel state */
static uint8_t gyro_en;                               /* Indicates gyro state */

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

static uint8_t current_opmode;

/* ── Runtime flag ── */
static volatile int g_running = 1;

/* ── Forward declarations ── */
static int  setup_imu(void);
static int  stop_algo(void);
static int  set_operation_mode(const op_mode_t *op_mode);
static int  start_algo(void);
static void sensor_event_cb(inv_imu_sensor_event_t *event);
static void int_cb(void *context, unsigned int int_num);
static int  init_imu_biases(int32_t m_bias_q16[3], uint8_t *const m_accuracy,
                            int16_t g_bias_q12[3], uint8_t *const g_accuracy,
                            int32_t *const t_q16);
static int  store_imu_biases(const int32_t m_bias_q16[3], const uint8_t m_accuracy,
                             const int16_t g_bias_q12[3], const uint8_t g_accuracy,
                             const int32_t t_q16);
static void notify_data(uint64_t time, const inv_edmp_gaf_outputs_t *gaf_outputs,
                        const int16_t rtemp_data);
static void notify_raw_data(uint64_t time, uint8_t input_mask,
                            const int16_t accel_data[3],
                            const int16_t gyro_data[3],
                            const int16_t temp_data);

/* ── Signal handler ── */
static void sig_handler(int sig)
{
	(void)sig;
	g_running = 0;
}

/* ── Usage ── */
static void usage(const char *prog)
{
	printf("Usage: %s [options]\n", prog);
	printf("  -i <dev>     I2C device   (default: %s)\n", DEFAULT_I2C_DEV);
	printf("  -g <chip>    GPIO chip    (default: %s)\n", DEFAULT_GPIO_CHIP);
	printf("  -l <line>    GPIO line    (default: %d)\n", DEFAULT_GPIO_LINE);
	printf("  -a <addr>    I2C addr     (default: 0x%02x)\n", DEFAULT_IMU_ADDR);
	printf("  -m <mode>    opmode 0-9   (default: 0)\n");
	printf("  -h           help\n");
}

/* ── Main ── */
int main(int argc, char *argv[])
{
	int rc = 0;
	const char *i2c_dev    = DEFAULT_I2C_DEV;
	const char *gpio_chip  = DEFAULT_GPIO_CHIP;
	unsigned int gpio_line = DEFAULT_GPIO_LINE;
	uint8_t     imu_addr   = DEFAULT_IMU_ADDR;
	int opt;

	while ((opt = getopt(argc, argv, "i:g:l:a:m:h")) != -1) {
		switch (opt) {
		case 'i': i2c_dev    = optarg; break;
		case 'g': gpio_chip  = optarg; break;
		case 'l': gpio_line  = (unsigned int)atoi(optarg); break;
		case 'a': imu_addr   = (uint8_t)strtoul(optarg, NULL, 0); break;
		case 'm': current_opmode = (uint8_t)atoi(optarg); break;
		case 'h': usage(argv[0]); return 0;
		default:  usage(argv[0]); return 1;
		}
	}

	if (current_opmode >= (sizeof(supported_cfg) / sizeof(supported_cfg[0]))) {
		fprintf(stderr, "Invalid opmode %d (0-%zu)\n", current_opmode,
		        sizeof(supported_cfg) / sizeof(supported_cfg[0]) - 1);
		return 1;
	}

	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	accel_en      = 0;
	gyro_en       = 0;
	power_save_en = 0;

	/* Accelerometer bias are 0 by default */
	acc_bias_q16[0] = 0;
	acc_bias_q16[1] = 0;
	acc_bias_q16[2] = 0;

	/* ── 1. Init HAL (replaces si_board_init, si_io_imu_init, si_flash_storage_init, si_init_timers) ── */
	rc |= emd_hal_init(i2c_dev, imu_addr, gpio_chip, gpio_line);
	SI_CHECK_RC(rc);

	/* ── 2. Configure GPIO to call int_cb when INT1 fires (replaces si_init_gpio_int) ── */
	rc |= emd_hal_gpio_init(1, (emd_gpio_cb_t)int_cb);
	SI_CHECK_RC(rc);

	fprintf(stderr, "[I] ###\n");
	fprintf(stderr, "[I] ### Example EDMP GAF (Linux port)\n");
	fprintf(stderr, "[I] ###\n");
	fprintf(stderr, "[I] I2C: %s (addr=0x%02x), GPIO: line %u, Mode: %d\n",
	        i2c_dev, imu_addr, gpio_chip, gpio_line, current_opmode);

	high_res_en    = 1; /* By default high resolution data are built */
	mrm_auto_is_on = 1; /* By default MRM is enabled */
	fifo_push_en   = 1; /* By default FIFO PUSH is enabled */
	freeze_mag_bias = 0; /* By default, ensure mag bias are taken into account */

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

	/* ── 3. setup_imu() ── */
	rc |= setup_imu();
	SI_CHECK_RC(rc);

	/* ── 4. Configure sensors ── */
	rc |= set_operation_mode(&supported_cfg[current_opmode]);
	rc |= start_algo();
	SI_CHECK_RC(rc);

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	fprintf(stderr, "[I] Entering main loop (Ctrl+C to stop)...\n");

	/* ── 5. Main loop: GPIO poll + FIFO processing ── */
	do {
		int ret = emd_hal_gpio_wait(100); /* 100ms timeout */
		if (ret < 0) break;
		if (ret == 0) continue; /* timeout, check g_running */

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
	} while (rc == 0 && g_running);

	/* ── 6. Cleanup ── */
	fprintf(stderr, "[I] Stopping algorithm...\n");
	stop_algo();
	emd_hal_deinit();

	return rc;
}

/* ── setup_imu() — from MCU edmp_gaf.c, HAL calls replaced ── */
static int setup_imu(void)
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

/* ── sensor_event_cb() — from MCU edmp_gaf.c ── */
static void sensor_event_cb(inv_imu_sensor_event_t *event)
{
	static int16_t prev_rgyr[3] = { 0 };
	uint8_t        input_mask   = 0;

	if (event->sensor_mask & (1 << INV_SENSOR_ACCEL)) {
		/* Align IMU data with EDMP input data */
		inv_imu_remap_data(event->accel, mounting_matrix);
		if (accel_en) {
			input_mask |= MASK_NOTIFY_RAW_ACC_DATA;
		}
	}

	if (event->sensor_mask & (1 << INV_SENSOR_GYRO)) {
		/* Align IMU data with EDMP input data */
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
			fprintf(stderr, "[E] Error when rebuilding GAF output, unknown FIFO frame received\n");
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
			if (gyro_is_on) {
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
				if (mrm_event.INV_GAF_MRM_THR)
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
			    ((int32_t)event->temperature * 32768);
			edmp_outputs.temperature_valid = 1;

			memset(&gaf_outputs, 0, sizeof(gaf_outputs));
		}
	}

	notify_raw_data(timestamp, input_mask, event->accel, event->gyro, event->temperature);
	/* Only push edmp data if FIFO push was enabled */
	if (fifo_push_en) {
		notify_data(timestamp, &edmp_outputs, event->temperature);
	}
	/* Clear all flags not related to racc/rgyr data */
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
		edmp_outputs.gyr_bias_valid  = 0;
		edmp_outputs.gyr_flags_valid = 0;
	}
}

/* ── stop_algo() — from MCU edmp_gaf.c ── */
static int stop_algo(void)
{
	int rc = 0;

	rc |= inv_imu_edmp_disable(&imu_dev);

	/* Get latest computed gyro and mag bias */
	rc |= inv_imu_edmp_get_gaf_gyr_bias(&imu_dev, gyr_bias_q12, &gyr_bias_temperature, &gyr_accuracy);
	rc |= inv_imu_edmp_get_gaf_mag_bias(&imu_dev, mag_bias_q16, &mag_accuracy);

	return rc;
}

/* ── set_operation_mode() — from MCU edmp_gaf.c ── */
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
		fprintf(stderr, "[E] Unknown dmp_odr_us %d, force to default 200Hz\n", (int)dmp_odr_us);
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

	/* Force clock configuration to fit I2C master need */
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

/* ── start_algo() — from MCU edmp_gaf.c ── */
static int start_algo(void)
{
	int                                 rc = 0;
	inv_imu_edmp_gaf_parameters_t       gaf_params;
	inv_imu_edmp_powersave_parameters_t apex_parameters;

	/* Clear global variable when new algorithm execution starts */
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
	rc |= inv_imu_edmp_set_gaf_gyr_bias(&imu_dev, gyr_bias_q12, gyr_bias_temperature, gyr_accuracy);
	mag_accuracy = 0;
	rc |= inv_imu_edmp_set_gaf_mag_bias(&imu_dev, mag_bias_q16, mag_accuracy);

	rc |= inv_imu_edmp_set_gaf_parameters(&imu_dev, &gaf_params);
	SI_CHECK_RC(rc);
	fprintf(stderr, "[I] eDMP parameter clock_variation is 0x%hhx corresponding to %+f%% error\n",
	        gaf_params.clock_variation, ((float)gaf_params.clock_variation / 25.40f));

	/* Configure GAF 6 or 9 axis : AG, AM or AGM */
	rc |= inv_imu_edmp_set_gaf_mode(&imu_dev, gyro_is_on, mag_is_on);

	/* Configure APEX parameters for power-save mode */
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

	/* ── Enable GAF ── */
	rc |= inv_imu_edmp_enable_gaf(&imu_dev);
	SI_CHECK_RC(rc);

	/* ── Enable eDMP ── */
	rc |= inv_imu_edmp_enable(&imu_dev);

	/* ── Let dmp see Accel data ready on ISR0 ── */
	rc |= inv_imu_edmp_unmask_int_src(&imu_dev, INV_IMU_EDMP_INT0,
	                                  EDMP_INT_SRC_ACCEL_DRDY_MASK | EDMP_INT_SRC_GYRO_DRDY_MASK);

	return rc;
}

/* ── int_cb() — from MCU edmp_gaf.c ── */
static void int_cb(void *context, unsigned int int_num)
{
	(void)context;

	if (int_num == 1) { /* SI_GPIO_INT1 */
		int1_timestamp = si_get_time_us();
		int1_flag      = 1;
	}
}

/* ── init_imu_biases() — from MCU edmp_gaf.c ── */
static int init_imu_biases(int32_t m_bias_q16[3], uint8_t *const m_accuracy,
                           int16_t g_bias_q12[3], uint8_t *const g_accuracy,
                           int32_t *const t_q16)
{
	int rc;
	uint8_t sensor_bias[84] = { 0 };
	rc = emd_hal_storage_read(sensor_bias, 84);
	if (0 == rc) {
		(void)memcpy(g_bias_q12, sensor_bias, 3 * sizeof(g_bias_q12[0]));
		(void)memcpy(t_q16, &sensor_bias[3 * sizeof(g_bias_q12[0])], sizeof(*t_q16));
		(void)memcpy(m_bias_q16, &sensor_bias[3 * sizeof(g_bias_q12[0]) + sizeof(*t_q16)],
		             3 * sizeof(m_bias_q16[0]));

		*g_accuracy = 3;
		fprintf(stderr, "[I] Loading calibration from flash:\n");
		fprintf(stderr, "[I]    - Gyro:  [%f %f %f]dps at %f degC\n",
		        (float)g_bias_q12[0] / (1 << 12), (float)g_bias_q12[1] / (1 << 12),
		        (float)g_bias_q12[2] / (1 << 12), (float)*t_q16 / (1 << 16));
		*m_accuracy = 3;
		fprintf(stderr, "[I]    - Mag:  [%f %f %f]uT\n",
		        (float)m_bias_q16[0] / (1 << 16), (float)m_bias_q16[1] / (1 << 16),
		        (float)m_bias_q16[2] / (1 << 16));
	} else {
		memset(g_bias_q12, 0, 3 * sizeof(g_bias_q12[0]));
		*g_accuracy = 0;
		*t_q16      = GAF_DEFAULT_TEMPERATURE_INIT_Q16;
		memset(m_bias_q16, 0, 3 * sizeof(m_bias_q16[0]));
		*m_accuracy = 0;
		rc          = INV_IMU_OK;
	}

	return rc;
}

/* ── store_imu_biases() — from MCU edmp_gaf.c ── */
static int store_imu_biases(const int32_t m_bias_q16[3], const uint8_t m_accuracy,
                            const int16_t g_bias_q12[3], const uint8_t g_accuracy,
                            const int32_t t_q16)
{
	int rc = INV_IMU_OK;
	uint8_t sensor_bias[84] = { 0 };

	emd_hal_storage_read(sensor_bias, 84);

	if (3 == g_accuracy) {
		(void)memcpy(sensor_bias, g_bias_q12, 3 * sizeof(g_bias_q12[0]));
		(void)memcpy(&sensor_bias[3 * sizeof(g_bias_q12[0])], &t_q16, sizeof(t_q16));
		fprintf(stderr, "[I] Saving calibration in flash:\n");
		fprintf(stderr, "[I]    - Gyro:  [%f %f %f]dps at %f degC\n",
		        (float)g_bias_q12[0] / (1 << 12), (float)g_bias_q12[1] / (1 << 12),
		        (float)g_bias_q12[2] / (1 << 12), (float)t_q16 / (1 << 16));
	} else {
		fprintf(stderr, "[W] Gyroscope biases were not saved due to insufficient accuracy level (was %d, must be 3)\n",
		        (int32_t)g_accuracy);
	}
	if (3 == m_accuracy) {
		(void)memcpy(&sensor_bias[3 * sizeof(g_bias_q12[0]) + sizeof(t_q16)], m_bias_q16,
		             3 * sizeof(m_bias_q16[0]));
		fprintf(stderr, "[I] Saving calibration in flash:\n");
		fprintf(stderr, "[I]    - Mag:  [%f %f %f]uT\n",
		        (float)m_bias_q16[0] / (1 << 16), (float)m_bias_q16[1] / (1 << 16),
		        (float)m_bias_q16[2] / (1 << 16));
	} else {
		fprintf(stderr, "[W] Magnetometer biases were not saved due to insufficient accuracy level (was %d, must be 3)\n",
		        (int32_t)m_accuracy);
	}

	if ((3 == g_accuracy) || (3 == m_accuracy))
		rc |= emd_hal_storage_write(sensor_bias, 84);

	return rc;
}

/* ── notify_data() — stdout printf, replaces MCU UART frontend ── */
static void notify_data(uint64_t time, const inv_edmp_gaf_outputs_t *gaf_outputs,
                        const int16_t rtemp_data)
{
	(void)rtemp_data;

	if (gaf_outputs->grv_quat_valid) {
		printf("DATA:ts=%llu GRV_Q30=[%d,%d,%d,%d]\n",
		       (unsigned long long)time,
		       gaf_outputs->grv_quat_q30[0], gaf_outputs->grv_quat_q30[1],
		       gaf_outputs->grv_quat_q30[2], gaf_outputs->grv_quat_q30[3]);
	}

	if (gaf_outputs->gmrv_quat_valid) {
		printf("DATA:ts=%llu GMRV_Q30=[%d,%d,%d,%d] heading_q27=%d\n",
		       (unsigned long long)time,
		       gaf_outputs->gmrv_quat_q30[0], gaf_outputs->gmrv_quat_q30[1],
		       gaf_outputs->gmrv_quat_q30[2], gaf_outputs->gmrv_quat_q30[3],
		       gaf_outputs->gmrv_heading_q27);
	}

	if (gaf_outputs->rv_quat_valid) {
		printf("DATA:ts=%llu RV_Q30=[%d,%d,%d,%d] heading_q27=%d\n",
		       (unsigned long long)time,
		       gaf_outputs->rv_quat_q30[0], gaf_outputs->rv_quat_q30[1],
		       gaf_outputs->rv_quat_q30[2], gaf_outputs->rv_quat_q30[3],
		       gaf_outputs->rv_heading_q27);
	}

	if (gaf_outputs->acc_cal_valid) {
		printf("DATA:ts=%llu ACC_CAL_Q16=[%d,%d,%d]\n",
		       (unsigned long long)time,
		       gaf_outputs->acc_cal_q16[0], gaf_outputs->acc_cal_q16[1],
		       gaf_outputs->acc_cal_q16[2]);
	}

	if (gaf_outputs->gyr_bias_valid || gaf_outputs->gyr_flags_valid) {
		printf("DATA:ts=%llu GYR_CAL_Q16=[%d,%d,%d] GYR_BIAS_Q16=[%d,%d,%d] stationary=%d accuracy=%d\n",
		       (unsigned long long)time,
		       gaf_outputs->gyr_cal_q16[0], gaf_outputs->gyr_cal_q16[1],
		       gaf_outputs->gyr_cal_q16[2],
		       gaf_outputs->gyr_bias_q16[0], gaf_outputs->gyr_bias_q16[1],
		       gaf_outputs->gyr_bias_q16[2],
		       (int32_t)gaf_outputs->stationary_flag,
		       (int32_t)gaf_outputs->gyr_accuracy_flag);
	}

	if (gaf_outputs->rmag_valid) {
		printf("DATA:ts=%llu RMAG=[%d,%d,%d]\n",
		       (unsigned long long)time,
		       (int32_t)gaf_outputs->raw_mag[0], (int32_t)gaf_outputs->raw_mag[1],
		       (int32_t)gaf_outputs->raw_mag[2]);
	}

	if (gaf_outputs->mag_bias_valid && gaf_outputs->rmag_valid) {
		printf("DATA:ts=%llu MAG_CAL_Q16=[%d,%d,%d] MAG_BIAS_Q16=[%d,%d,%d] anomalies=%d accuracy=%d\n",
		       (unsigned long long)time,
		       gaf_outputs->mag_cal_q16[0], gaf_outputs->mag_cal_q16[1],
		       gaf_outputs->mag_cal_q16[2],
		       gaf_outputs->mag_bias_q16[0], gaf_outputs->mag_bias_q16[1],
		       gaf_outputs->mag_bias_q16[2],
		       (int32_t)gaf_outputs->mag_anomaly,
		       (int32_t)gaf_outputs->mag_accuracy_flag);
	}

	if (gaf_outputs->mrm_state_valid) {
		printf("DATA:ts=%llu MRM_STATE=%d chg=%d exe=%d thr=%d\n",
		       (unsigned long long)time,
		       (uint32_t)gaf_outputs->mrm_state,
		       (uint32_t)gaf_outputs->mrm_evt_chg_st,
		       (uint32_t)gaf_outputs->mrm_evt_exe_mrm,
		       (uint32_t)gaf_outputs->mrm_evt_exc_thr);
	}

	if (gaf_outputs->temperature_valid) {
		printf("DATA:ts=%llu TEMP_Q16=%d\n",
		       (unsigned long long)time,
		       gaf_outputs->temp_degC_q16);
	}
}

/* ── notify_raw_data() — stdout printf, replaces MCU UART frontend ── */
static void notify_raw_data(uint64_t time, uint8_t input_mask,
                            const int16_t accel_data[3],
                            const int16_t gyro_data[3],
                            const int16_t temp_data)
{
	if (input_mask & MASK_NOTIFY_RAW_ACC_DATA)
		printf("RAW:ts=%llu RAcc=[%d,%d,%d]\n",
		       (unsigned long long)time,
		       (uint32_t)accel_data[0], (uint32_t)accel_data[1], (uint32_t)accel_data[2]);

	if (input_mask & MASK_NOTIFY_RAW_GYR_DATA)
		printf("RAW:ts=%llu RGyr=[%d,%d,%d]\n",
		       (unsigned long long)time,
		       (uint32_t)gyro_data[0], (uint32_t)gyro_data[1], (uint32_t)gyro_data[2]);

	if ((input_mask & MASK_NOTIFY_RAW_ACC_DATA) || (input_mask & MASK_NOTIFY_RAW_GYR_DATA))
		printf("RAW:ts=%llu RTemp=%d\n",
		       (unsigned long long)time, (uint32_t)temp_data);
}
