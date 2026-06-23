/**
 * @file emd_gaf_types.h
 * @brief eMD GAF HAL — 公开数据类型
 *
 * 定义融合输出结构体 emd_output_t 和 IMU 原始数据结构体。
 * 用于 libemd_gaf.so 的跨模块接口。
 *
 * Copyright (c) 2026 张君宝
 */
#ifndef EMD_GAF_TYPES_H
#define EMD_GAF_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 9 轴融合输出, 包括四元数、航向角、校准传感器数据
 *
 * 数据来源: ICM45608 eDMP GAF 引擎
 * get_output() 非阻塞读取最近一次融合结果
 */
typedef struct {
    /** 9 轴 (accel + gyro + mag) 四元数, 归一化到 [-1, 1] */
    float quat_w, quat_x, quat_y, quat_z;
    /** 航向角 (度), 基于 9 轴四元数解算 */
    float heading_deg;
    /** 校准后加速度 (g, 1 g = 9.8 m/s²) */
    float accel_x, accel_y, accel_z;
    /** 校准后角速度 (dps, 度/秒) */
    float gyro_x, gyro_y, gyro_z;
    /** 校准后磁力计 (uT, 微特斯拉) */
    float mag_x, mag_y, mag_z;
    /** 温度 (°C) */
    float temp_c;
    /** 静止检测: 0=运动, 2=静止 */
    int   stationary;
    /** 陀螺仪零偏校准精度: 0=未校准, 3=精校准 */
    int   gyr_accuracy;
    /** 磁力计校准精度: 0=未校准, 3=精校准 */
    int   mag_accuracy;
    /** 时间戳 (微秒, CLOCK_MONOTONIC) */
    uint64_t timestamp_us;
} emd_output_t;

/**
 * @brief 原始 IMU 数据 (非阻塞读取)
 *
 * 提供未经过 eDMP 融合的加速度计和陀螺仪原始值,
 * 采样率跟随 IMU ODR (200~800 Hz, 取决于工作模式)。
 */
typedef struct {
    /** 原始加速度 (g) */
    float accel_x, accel_y, accel_z;
    /** 原始角速度 (dps) */
    float gyro_x, gyro_y, gyro_z;
    /** 时间戳 (微秒, CLOCK_MONOTONIC) */
    uint64_t timestamp_us;
} emd_imu_data_t;

#ifdef __cplusplus
}
#endif

#endif /* EMD_GAF_TYPES_H */
