/*
 * system_interface.h - eMD SDK → Linux HAL bridge
 *
 * Maps MCU system_interface.h functions to emd_hal.h
 * The eMD drivers call si_* functions; we implement them
 * as wrappers around our Linux HAL.
 */
#ifndef SYSTEM_INTERFACE_H
#define SYSTEM_INTERFACE_H

#include "emd_hal.h"
#include "imu/inv_imu_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Board ── */
static inline int si_board_init(void) { return 0; }

/* ── IMU I/O (maps to our HAL) ── */
static inline int si_io_imu_init(inv_imu_serif_type_t serif_type) {
    (void)serif_type;
    return 0; /* already done in emd_hal_init() */
}

static inline int si_io_imu_read_reg(uint8_t reg, uint8_t *buf, uint32_t len) {
    return emd_hal_read_reg(reg, buf, len);
}

static inline int si_io_imu_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len) {
    return emd_hal_write_reg(reg, buf, len);
}

/* ── GPIO Interrupt ── */
#define SI_GPIO_INT1 1

static inline int si_init_gpio_int(unsigned int int_num,
                                   void (*int_cb)(void *context, unsigned int int_num)) {
    return emd_hal_gpio_init(int_num, (emd_gpio_cb_t)int_cb);
}

/* ── Timer ── */
static inline int  si_init_timers(void) { return 0; }
static inline void si_sleep_us(uint32_t us) { emd_hal_sleep_us(us); }

static inline uint64_t si_get_time_us(void) {
    return emd_hal_get_time_us();
}

/* ── IRQ Control ── */
static inline void si_disable_irq(void) { emd_hal_disable_irq(); }
static inline void si_enable_irq(void)  { emd_hal_enable_irq(); }

/* ── Storage ── */
static inline int si_flash_storage_read(uint8_t *data) {
    return emd_hal_storage_read(data, 84);
}

static inline int si_flash_storage_write(const uint8_t *data) {
    return emd_hal_storage_write(data, 84);
}

/* ── Unused / Stub ── */
static inline int  si_init_clkin(void)   { return 0; }
static inline int  si_uninit_clkin(void) { return 0; }
static inline int  si_config_uart_for_print(int id, int level) { (void)id; (void)level; return 0; }
static inline int  si_config_uart_for_bin(int id)   { (void)id; return 0; }
static inline int  si_get_uart_command(int id, char *cmd) { (void)id; *cmd=0; return 0; }
static inline int  si_io_akm_init(void *s)    { (void)s; return -1; }
static inline int  si_io_akm_read_reg(void *s, uint8_t r, uint8_t *b, uint32_t l) { (void)s;(void)r;(void)b;(void)l; return -1; }
static inline int  si_io_akm_write_reg(void *s, uint8_t r, const uint8_t *b, uint32_t l) { (void)s;(void)r;(void)b;(void)l; return -1; }
static inline int  si_start_gpio_fsync(uint32_t f, void (*cb)(void*)) { (void)f;(void)cb; return 0; }
static inline int  si_stop_gpio_fsync(void) { return 0; }
static inline void si_toggle_gpio_fsync(void) {}

#define SI_CHECK_RC(rc) do { \
    if (rc) { fprintf(stderr, "SI error %d at %s:%d\n", rc, __FILE__, __LINE__); return rc; } \
} while(0)

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_INTERFACE_H */
