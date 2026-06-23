/*
 * emd_hal.c - eMD HAL Linux Userspace Implementation (Self-contained)
 *
 * Hardware:
 *   I2C3, 400kHz, ICM45608 addr 0x68
 *   INT1: GPIO4_PA2, IRQ_TYPE_LEVEL_LOW
 *
 * Zero external dependencies beyond POSIX + standard C.
 * I2C: raw ioctl(I2C_RDWR) with locally-defined structs
 * GPIO: sysfs /sys/class/gpio export + poll on value
 */
#include "emd_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <poll.h>

/* ── I2C structures (from linux/i2c-dev.h, self-defined) ── */
#define I2C_SLAVE        0x0703
#define I2C_RDWR         0x0707
#define I2C_M_RD         0x0001

struct i2c_msg_local {
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t  *buf;
};

struct i2c_rdwr_ioctl_data_local {
    struct i2c_msg_local *msgs;
    uint32_t nmsgs;
};

/* ── Private state ── */
static int              g_i2c_fd       = -1;
static uint8_t          g_imu_addr     = 0x68;

static emd_gpio_cb_t          g_gpio_cb       = NULL;
static void                  *g_gpio_context  = NULL;
static unsigned int            g_gpio_num      = 0;
static int              g_gpio_fd      = -1;
static unsigned int      g_gpio_line     = 0;

static pthread_mutex_t  g_irq_mutex    = PTHREAD_MUTEX_INITIALIZER;
static int              g_initialized  = 0;

static const char      *g_bias_file    = "./imu_bias.bin";

/* ── Helper: sysfs write ── */
static int sysfs_write(const char *path, const char *val)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    int n = write(fd, val, strlen(val));
    close(fd);
    return (n > 0) ? 0 : -1;
}

/* ═══════════════════════════════════════════════════════════════════
 * I2C Operations (raw ioctl, no linux/i2c-dev.h needed)
 * ═══════════════════════════════════════════════════════════════════ */

int emd_hal_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
    struct i2c_msg_local msgs[2];
    struct i2c_rdwr_ioctl_data_local ioctl_data;

    msgs[0].addr  = g_imu_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = g_imu_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = len;
    msgs[1].buf   = buf;

    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(g_i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        fprintf(stderr, "EMD_HAL: I2C read reg=0x%02x len=%u failed: %s\n",
                reg, len, strerror(errno));
        return -1;
    }
    return 0;
}

int emd_hal_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len)
{
    struct i2c_msg_local msgs[1];
    struct i2c_rdwr_ioctl_data_local ioctl_data;
    uint8_t tx[1 + len];

    tx[0] = reg;
    if (len > 0 && buf)
        memcpy(tx + 1, buf, len);

    msgs[0].addr  = g_imu_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1 + len;
    msgs[0].buf   = tx;

    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 1;

    if (ioctl(g_i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        fprintf(stderr, "EMD_HAL: I2C write reg=0x%02x len=%u failed: %s\n",
                reg, len, strerror(errno));
        return -1;
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════
 * GPIO Interrupt (sysfs: export/edge/value + poll)
 * ═══════════════════════════════════════════════════════════════════ */

int emd_hal_gpio_init(unsigned int int_num, emd_gpio_cb_t cb)
{
    char path[128];
    char val[32];

    g_gpio_num = int_num;
    g_gpio_cb  = cb;

    /* Export GPIO */
    snprintf(path, sizeof(path), "/sys/class/gpio/export");
    snprintf(val, sizeof(val), "%u", g_gpio_line);
    if (sysfs_write(path, val) < 0) {
        /* Already exported, that's OK */
    }

    /* Set direction: input */
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%u/direction", g_gpio_line);
    sysfs_write(path, "in");

    /* Set edge: DTS says IRQ_TYPE_LEVEL_LOW but ICM45608 INT is
     * active low, pulsing, so we catch falling edge. */
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%u/edge", g_gpio_line);
    sysfs_write(path, "falling");

    /* Open value file for poll */
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%u/value", g_gpio_line);
    g_gpio_fd = open(path, O_RDONLY);
    if (g_gpio_fd < 0) {
        fprintf(stderr, "EMD_HAL: open %s failed: %s\n", path, strerror(errno));
        return -1;
    }

    /* Dummy read to clear initial state */
    char c;
    read(g_gpio_fd, &c, 1);

    printf("EMD_HAL: GPIO %u ready (sysfs)\n", g_gpio_line);
    return 0;
}

int emd_hal_gpio_wait(int timeout_ms)
{
    struct pollfd pfd;
    int ret;

    if (g_gpio_fd < 0)
        return -1;

    pfd.fd      = g_gpio_fd;
    pfd.events  = POLLPRI | POLLERR;
    pfd.revents = 0;

    /* Seek to start before poll */
    lseek(g_gpio_fd, 0, SEEK_SET);

    ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        if (errno == EINTR) return 0;
        return -1;
    }
    if (ret == 0)
        return 0; /* timeout */

    /* Read to clear interrupt */
    char c;
    lseek(g_gpio_fd, 0, SEEK_SET);
    read(g_gpio_fd, &c, 1);

    if (g_gpio_cb)
        g_gpio_cb(g_gpio_context, g_gpio_num);

    return 1;
}

/* ═══════════════════════════════════════════════════════════════════
 * Timer
 * ═══════════════════════════════════════════════════════════════════ */

void emd_hal_sleep_us(uint32_t us)
{
    usleep(us);
}

uint64_t emd_hal_get_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

/* ═══════════════════════════════════════════════════════════════════
 * IRQ Control (mutex)
 * ═══════════════════════════════════════════════════════════════════ */

void emd_hal_disable_irq(void)
{
    pthread_mutex_lock(&g_irq_mutex);
}

void emd_hal_enable_irq(void)
{
    pthread_mutex_unlock(&g_irq_mutex);
}

/* ═══════════════════════════════════════════════════════════════════
 * Storage
 * ═══════════════════════════════════════════════════════════════════ */

int emd_hal_storage_read(uint8_t *data, uint32_t size)
{
    FILE *f = fopen(g_bias_file, "rb");
    if (!f)
        return -1;
    size_t n = fread(data, 1, size, f);
    fclose(f);
    return (n == size) ? 0 : -1;
}

int emd_hal_storage_write(const uint8_t *data, uint32_t size)
{
    FILE *f = fopen(g_bias_file, "wb");
    if (!f)
        return -1;
    size_t n = fwrite(data, 1, size, f);
    fclose(f);
    return (n == size) ? 0 : -1;
}

/* ═══════════════════════════════════════════════════════════════════
 * Init / Deinit
 * ═══════════════════════════════════════════════════════════════════ */

int emd_hal_init(const char *i2c_dev, uint8_t imu_addr, unsigned int gpio_line)
{
    if (g_initialized)
        return 0;

    g_gpio_line = gpio_line;

    /* ── I2C ── */
    g_i2c_fd = open(i2c_dev, O_RDWR);
    if (g_i2c_fd < 0) {
        fprintf(stderr, "EMD_HAL: open %s failed: %s\n",
                i2c_dev, strerror(errno));
        return -1;
    }
    g_imu_addr = imu_addr;
    printf("EMD_HAL: I2C %s ready (addr=0x%02x)\n", i2c_dev, imu_addr);

    g_initialized = 1;
    return 0;
}

void emd_hal_deinit(void)
{
    if (g_gpio_fd >= 0) {
        close(g_gpio_fd);
        g_gpio_fd = -1;
    }
    /* Unexport GPIO */
    char val[32];
    snprintf(val, sizeof(val), "%u", g_gpio_line);
    sysfs_write("/sys/class/gpio/unexport", val);
    if (g_i2c_fd >= 0) {
        close(g_i2c_fd);
        g_i2c_fd = -1;
    }
    g_initialized = 0;
}
