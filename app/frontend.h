/*
 * frontend.h - Minimal frontend for Linux userspace
 *
 * Maps MCU UART frontend to stdout.
 * INV_MSG → printf
 * notify_data → stdout hex dump
 */
#ifndef FRONTEND_H
#define FRONTEND_H

#include <stdio.h>
#include <stdint.h>

/* ── Message levels ── */
#define INV_MSG_LEVEL_ERROR   0
#define INV_MSG_LEVEL_WARNING 1
#define INV_MSG_LEVEL_INFO    2
#define INV_MSG_LEVEL_DEBUG   3
#define INV_MSG_LEVEL_VERBOSE 4

#define INV_MSG(level, fmt, ...) \
    fprintf(stderr, "[%d] " fmt "\n", level, ##__VA_ARGS__)

/* ── Frontend config flags ── */
#define FRONTEND_CONFIG_GAF          (1 << 0)
#define FRONTEND_CONFIG_GAF_RMAG     (1 << 1)
#define FRONTEND_CONFIG_GAF_CFG      (1 << 2)
#define FRONTEND_CONFIG_B2S          (1 << 3)

/* ── API ── */
static inline int init_frontend(uint32_t config, void *cmd_cb) {
    (void)config; (void)cmd_cb;
    return 0;
}

static inline void notify_data(uint64_t ts, void *outputs_ptr) {
    (void)ts;
    /* edmp_outputs already printed in sensor_event_cb */
    if (outputs_ptr) {
        /* Could add structured output here */
    }
}

static inline void notify_raw_data(uint64_t ts, int16_t *acc, int16_t *gyr, int16_t temp) {
    fprintf(stderr, "RAW: ts=%llu acc=[%d %d %d] gyr=[%d %d %d] temp=%d\n",
            (unsigned long long)ts,
            acc ? acc[0] : 0, acc ? acc[1] : 0, acc ? acc[2] : 0,
            gyr ? gyr[0] : 0, gyr ? gyr[1] : 0, gyr ? gyr[2] : 0,
            temp);
}

#endif /* FRONTEND_H */
