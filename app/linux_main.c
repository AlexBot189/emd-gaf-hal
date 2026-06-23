/*
 * linux_main.c - eMD GAF Linux Userspace Entry Point
 *
 * Replaces MCU main() from edmp_gaf.c.
 * 
 * Build:  mkdir build && cd build && cmake .. && make
 * Run:    ./emd-gaf /dev/i2c-3 gpiochip4 2
 *         (I2C3 bus, GPIO4 chip, line PA2=2)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

/* eMD drivers */
#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_wearable.h"

/* HAL */
#include "system_interface.h"

/* App */
#include "invn_mag.h"
#include "frontend.h"

/* ── Default config ── */
#define DEFAULT_I2C_DEV   "/dev/i2c-3"
#define DEFAULT_GPIO_CHIP "gpiochip4"
#define DEFAULT_GPIO_LINE 2        /* PA2 */
#define DEFAULT_IMU_ADDR  0x68

/* ── Operating modes (matching MCU supported_cfg[]) ── */
typedef struct {
    uint32_t dmp_sensor_odr_us;
    uint32_t gaf_pdr_us;
    uint8_t  fusion_enabled;
    uint8_t  acc_mode;    /* 0=OFF, 2=LP, 3=LN */
    uint8_t  gyr_mode;
    uint32_t acc_lpf;
    uint32_t gyr_lpf;
    uint8_t  mag_on;
    uint32_t mag_odr_us;
} op_mode_t;

static const op_mode_t supported_cfg[] = {
    /* 0: LN 200Hz AGM PDR=10000us */
    { 5000, 10000, 0, 3, 3, 0, 0, 1, 10000 },
    /* 1: LP 100Hz AGM PDR=20000us */
    { 10000, 20000, 0, 2, 2, 1, 1, 1, 20000 },
    /* 2: LP 100Hz accel-only PDR=20000us */
    { 10000, 20000, 0, 2, 0, 1, 1, 1, 20000 },
    /* 3: LN 400Hz GAF 200Hz */
    { 2500, 5000, 1, 3, 3, 0, 0, 1, 20000 },
    /* 4: LP 100Hz GAF 50Hz */
    { 10000, 20000, 1, 2, 2, 1, 1, 1, 20000 },
    /* 5: LN 100Hz GAF 50Hz (recommended) */
    { 10000, 20000, 1, 3, 3, 0, 0, 1, 20000 },
};

/* ── Global state ── */
static volatile int      g_running     = 1;
static volatile int      g_int1_flag   = 0;
static volatile uint64_t g_int1_ts     = 0;
static inv_imu_device_t  g_imu_dev;
static inv_edmp_gaf_outputs_t g_edmp_out;
static int               g_opmode      = 5;  /* default: LN 100Hz GAF 50Hz */

/* ── INT1 callback ── */
static void int_cb(void *context, unsigned int int_num)
{
    (void)context;
    (void)int_num;
    g_int1_ts    = si_get_time_us();
    g_int1_flag  = 1;
}

/* ── Sensor event callback ── */
static void sensor_event_cb(inv_imu_sensor_event_t *event)
{
    /* GAF fusion output */
    if (event->es0_en && event->es1_en) {
        inv_imu_edmp_gaf_outputs_t gaf_out;
        int ret = inv_imu_edmp_gaf_decode_fifo(&g_imu_dev,
                        (const uint8_t *)event->es0,
                        (const uint8_t *)event->es1, &gaf_out);
        if (ret == 0 && gaf_out.frame_complete) {
            memcpy(&g_edmp_out, &gaf_out, sizeof(gaf_out));
            /* Output quaternion */
            if (gaf_out.rv_quat_valid) {
                printf("QUAT: %d %d %d %d\n",
                    gaf_out.rv_quat_q14[0],
                    gaf_out.rv_quat_q14[1],
                    gaf_out.rv_quat_q14[2],
                    gaf_out.rv_quat_q14[3]);
            }
        }
    }
}

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
    printf("  -i <dev>     I2C device (default: %s)\n", DEFAULT_I2C_DEV);
    printf("  -g <chip>    GPIO chip  (default: %s)\n", DEFAULT_GPIO_CHIP);
    printf("  -l <line>    GPIO line  (default: %d)\n", DEFAULT_GPIO_LINE);
    printf("  -a <addr>    I2C addr   (default: 0x%02x)\n", DEFAULT_IMU_ADDR);
    printf("  -m <mode>    opmode 0-5 (default: %d)\n", g_opmode);
    printf("  -h           help\n");
}

/* ── Main ── */
int main(int argc, char *argv[])
{
    const char *i2c_dev    = DEFAULT_I2C_DEV;
    const char *gpio_chip  = DEFAULT_GPIO_CHIP;
    unsigned int gpio_line = DEFAULT_GPIO_LINE;
    uint8_t     imu_addr   = DEFAULT_IMU_ADDR;
    int opt, rc;

    while ((opt = getopt(argc, argv, "i:g:l:a:m:h")) != -1) {
        switch (opt) {
        case 'i': i2c_dev   = optarg; break;
        case 'g': gpio_chip = optarg; break;
        case 'l': gpio_line = atoi(optarg); break;
        case 'a': imu_addr  = (uint8_t)strtoul(optarg, NULL, 0); break;
        case 'm': g_opmode  = atoi(optarg); break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (g_opmode >= (int)(sizeof(supported_cfg)/sizeof(supported_cfg[0]))) {
        fprintf(stderr, "Invalid opmode %d (0-%zu)\n", g_opmode,
                sizeof(supported_cfg)/sizeof(supported_cfg[0])-1);
        return 1;
    }
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("EMD GAF Linux userspace driver\n");
    printf("  I2C: %s (addr=0x%02x)\n", i2c_dev, imu_addr);
    printf("  GPIO: %s line %u\n", gpio_chip, gpio_line);
    printf("  Mode: %d (dmp_odr=%u us, pdr=%u us, fusion=%d, mag=%d)\n",
           g_opmode,
           supported_cfg[g_opmode].dmp_sensor_odr_us,
           supported_cfg[g_opmode].gaf_pdr_us,
           supported_cfg[g_opmode].fusion_enabled,
           supported_cfg[g_opmode].mag_on);

    /* ── Init HAL ── */
    rc = emd_hal_init(i2c_dev, imu_addr, gpio_chip, gpio_line);
    if (rc) { fprintf(stderr, "HAL init failed\n"); return 1; }

    rc = si_init_gpio_int(1, int_cb);
    if (rc) { fprintf(stderr, "GPIO init failed\n"); return 1; }

    /* ── Init IMU transport ── */
    memset(&g_imu_dev, 0, sizeof(g_imu_dev));
    g_imu_dev.transport.read_reg   = si_io_imu_read_reg;
    g_imu_dev.transport.write_reg  = si_io_imu_write_reg;
    g_imu_dev.transport.serif_type = UI_I2C;
    g_imu_dev.transport.sleep_us   = si_sleep_us;
    g_imu_dev.adv_var.sensor_event_cb = sensor_event_cb;

    si_sleep_us(3000);
    rc = inv_imu_adv_init(&g_imu_dev);
    if (rc) { fprintf(stderr, "IMU init failed: %d\n", rc); return 1; }
    printf("IMU: WHOAMI OK\n");

    /* TODO: full IMU setup + GAF init */
    /* This is a skeleton - full init sequence from edmp_gaf.c's
     * setup_imu() + set_operation_mode() + start_algo() goes here */

    /* ── Main loop ── */
    printf("Entering main loop (Ctrl+C to stop)...\n");
    while (g_running) {
        int ret = emd_hal_gpio_wait(100); /* 100ms timeout */
        if (ret < 0) break;
        if (ret == 0) continue; /* timeout, check g_running */

        if (g_int1_flag) {
            uint64_t ts = g_int1_ts;
            g_int1_flag = 0;

            /* Read interrupt status */
            inv_imu_int_state_t int_state;
            inv_imu_adv_get_int_status(&g_imu_dev, INV_IMU_INT1, &int_state);

            if (int_state.INV_FIFO_THS) {
                /* Read + parse FIFO
                 * inv_imu_adv_get_data_from_fifo(&g_imu_dev, fifo_buf, &pkt_cnt);
                 * inv_imu_adv_parse_fifo_data(&g_imu_dev, fifo_buf, pkt_cnt);
                 *   → triggers sensor_event_cb() for each packet */
            }
            (void)ts;
        }
    }

    printf("Stopping...\n");
    emd_hal_deinit();
    return 0;
}
