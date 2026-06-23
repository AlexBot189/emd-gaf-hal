/**
 * @file emd_gaf.h
 * @brief eMD GAF HAL — ICM45608 9 轴融合动态库公共 API
 *
 * ## 概述
 *
 *   libemd_gaf.so 封装了 InvenSense ICM45608 eDMP GAF (Gyro-Assisted Fusion)
 *   引擎的 Linux userspace 驱动。提供 9 轴 IMU 数据的采集和融合,
 *   输出包括四元数、航向角、校准后的传感器数据和静止检测标志。
 *
 * ## 最小示例 (复制即用)
 *
 * @code
 *   #include "emd_gaf.h"
 *
 *   emd_gaf_t *gaf = emd_gaf_create();
 *   emd_gaf_init(gaf, "/dev/i2c-3", "gpiochip4", 2, 5);
 *   emd_gaf_start(gaf);
 *
 *   usleep(1000000);  // 等待融合收敛
 *
 *   emd_output_t out;
 *   while (running) {
 *       emd_gaf_get_output(gaf, &out);
 *       printf("heading=%.1f° quat=(%.3f,%.3f,%.3f,%.3f) stationary=%d\n",
 *              out.heading_deg, out.quat_w, out.quat_x, out.quat_y, out.quat_z,
 *              out.stationary);
 *       usleep(10000);
 *   }
 *
 *   emd_gaf_stop(gaf);
 *   emd_gaf_destroy(gaf);
 * @endcode
 *
 * ## 线程模型
 *
 *   ┌──────────────────────────┐     ┌────────────────────────┐
 *   │  后台采集线程 (libemd_gaf) │     │  用户线程 (你的代码)     │
 *   │                          │     │                        │
 *   │  GPIO poll 等待 INT1     │     │ emd_gaf_get_output()   │
 *   │  I2C FIFO 读取 + 解码    │ ──→ │   → 只读缓存 (非阻塞)   │
 *   │  更新输出缓存 (mutex)    │     │ emd_gaf_get_imu()      │
 *   │                          │     │   → 只读缓存 (非阻塞)   │
 *   └──────────────────────────┘     └────────────────────────┘
 *
 *   用户线程 "不触 I2C/GPIO" — 只读缓存。
 *   get_output/get_imu 只需一次 memcpy + mutex lock, ~百纳秒量级。
 *
 * ## 操作模式 (op_mode 0-9)
 *
 *   | 模式 | 描述                    | ODR    | 融合 |
 *   |------|-------------------------|--------|------|
 *   | 0    | HRC ALN GLN, MAG 100Hz  | 200Hz  | 否   |
 *   | 1    | HRC ALP GLP, MAG 50Hz   | 100Hz  | 否   |
 *   | 2    | HRC ALP, GYRO OFF       | 100Hz  | 否   |
 *   | 3    | ALN GLN, MAG 50Hz       | 400Hz  | 是   |
 *   | 4    | ALP GLP, MAG 50Hz       | 100Hz  | 是   |
 *   | 5    | ALN GLN, MAG 50Hz       | 100Hz  | 是   |
 *   | 6    | ALN GLN, MAG 50Hz       | 400Hz  | 是   |
 *   | 7    | ALN GLN, MAG 50Hz       | 800Hz  | 是   |
 *   | 8    | ALP GLP, MAG OFF        | 50Hz   | 是   |
 *   | 9    | ALP, GYRO OFF, MAG 50Hz | 100Hz  | 是   |
 *
 * Copyright (c) 2026 张君宝
 */
#ifndef EMD_GAF_H
#define EMD_GAF_H

#include "emd_gaf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 不透明句柄 ── */
typedef struct emd_gaf emd_gaf_t;

/* ═══════════════════════════════════════════════════════════════════
 * 1. 生命周期
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 创建 GAF 实例
 *
 * 分配内部数据结构，返回不透明句柄。
 * 一个进程通常只需一个实例。
 *
 * @return 成功返回非 NULL, 内存不足返回 NULL。
 */
emd_gaf_t *emd_gaf_create(void);

/**
 * @brief 销毁 GAF 实例, 释放所有资源
 *
 * 自动停止后台线程 (如果已启动), 释放 I2C/GPIO 资源,
 * 保存 IMU 校准偏置到文件。
 *
 * @param handle GAF 实例
 */
void emd_gaf_destroy(emd_gaf_t *handle);

/**
 * @brief 初始化 IMU 并配置 HAL
 *
 * 打开 I2C 设备, 请求 GPIO 中断线, 初始化 ICM45608,
 * 设置中断和 FIFO 配置。
 *
 * 必须在 emd_gaf_start() 之前调用。
 *
 * @param handle    GAF 实例
 * @param i2c_dev   I2C 设备路径, 如 "/dev/i2c-3"
 * @param gpio_chip GPIO 芯片名, 如 "gpiochip4" 或 "/dev/gpiochip4"
 * @param gpio_line GPIO 中断线编号, 如 2 (对应 INT1)
 * @param op_mode   操作模式 0-9, 见上方模式表
 * @return 0=成功; <0=失败
 */
int emd_gaf_init(emd_gaf_t *handle, const char *i2c_dev,
                 const char *gpio_chip, unsigned int gpio_line,
                 int op_mode);

/* ═══════════════════════════════════════════════════════════════════
 * 2. 采集控制
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 启动后台采集线程
 *
 * 创建 pthread, 在后台执行主循环:
 *   GPIO poll → FIFO 读取 → 解码 → 更新输出缓存
 *
 * @param handle GAF 实例
 * @return 0=成功; <0=失败
 */
int emd_gaf_start(emd_gaf_t *handle);

/**
 * @brief 停止后台采集线程
 *
 * 设置退出标志, pthread_join 等待线程结束,
 * 然后执行 stop_algo() 取出最新偏置并保存。
 *
 * @param handle GAF 实例
 * @return 0=成功
 */
int emd_gaf_stop(emd_gaf_t *handle);

/* ═══════════════════════════════════════════════════════════════════
 * 3. 数据读取 (非阻塞, 线程安全)
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 获取最近一次 9 轴融合输出 (非阻塞)
 *
 * 直接从后台线程维护的缓存中拷贝, 不触发 I/O。
 * 延迟: 一次 memcpy + pthread_mutex lock ~ 百纳秒。
 * 可在高频 RT 线程中调用。
 *
 * @param handle GAF 实例
 * @param output [out] 输出数据, 调用者分配
 * @return 0=有更新数据; 1=无新数据 (缓存未更新); <0=错误
 *
 * @code
 * emd_output_t out;
 * if (emd_gaf_get_output(gaf, &out) == 0) {
 *     printf("heading=%.1f°\n", out.heading_deg);
 * }
 * @endcode
 */
int emd_gaf_get_output(emd_gaf_t *handle, emd_output_t *output);

/**
 * @brief 获取最近一次原始 IMU 数据 (非阻塞)
 *
 * 返回加速度计和陀螺仪的原始数据 (未经 eDMP 融合)。
 *
 * @param handle GAF 实例
 * @param accel  [out] 加速度数据 (g), 调用者分配, 可为 NULL
 * @param gyro   [out] 角速度数据 (dps), 调用者分配, 可为 NULL
 * @return 0=成功; <0=错误
 */
int emd_gaf_get_imu(emd_gaf_t *handle, emd_imu_data_t *accel, emd_imu_data_t *gyro);

/**
 * @brief 查询后台线程是否在运行
 *
 * @return 1=运行中; 0=已停止
 */
int emd_gaf_is_running(emd_gaf_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* EMD_GAF_H */
