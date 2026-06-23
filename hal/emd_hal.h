/*
 * emd_hal.h - eMD Hardware Abstraction Layer for Linux Userspace
 *
 * Maps MCU system_interface.h functions to Linux userspace:
 *   - I2C:       /dev/i2c-N  (i2c-dev)
 *   - GPIO INT:  /dev/gpiochipN (libgpiod)
 *   - Timer:     clock_gettime / usleep
 *   - Storage:   file I/O (bias persistence)
 *
 * Copyright (c) 2026 张君宝
 */
#ifndef EMD_HAL_H
#define EMD_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Board Init ── */
int emd_hal_init(const char *i2c_dev, uint8_t imu_addr,
                 const char *gpio_chip, unsigned int gpio_line);

/* ── IMU I2C I/O ── */
int emd_hal_read_reg(uint8_t reg, uint8_t *buf, uint32_t len);
int emd_hal_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len);

/* ── GPIO Interrupt ── */
typedef void (*emd_gpio_cb_t)(void *context, unsigned int int_num);
int emd_hal_gpio_init(unsigned int int_num, emd_gpio_cb_t cb);
int emd_hal_gpio_wait(int timeout_ms);  /* poll for INT1, returns 1 on event */

/* ── Timer ── */
void    emd_hal_sleep_us(uint32_t us);
uint64_t emd_hal_get_time_us(void);

/* ── IRQ Control (mutex for critical sections) ── */
void emd_hal_disable_irq(void);
void emd_hal_enable_irq(void);

/* ── Storage (bias persistence) ── */
int emd_hal_storage_read(uint8_t *data, uint32_t size);
int emd_hal_storage_write(const uint8_t *data, uint32_t size);

/* ── Cleanup ── */
void emd_hal_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EMD_HAL_H */
