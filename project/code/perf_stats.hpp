#ifndef PERF_STATS_HPP
#define PERF_STATS_HPP

#include "app_config.h"

#if ENABLE_PERF_STATS

#include <cstdint>

/**
 * @brief 性能统计模块初始化
 * @return 无
 * @sample perf_stats_init();
 */
void perf_stats_init(void);

/**
 * @brief 标记本帧图像处理开始（image_get 成功后调用）
 * @return 无
 * @sample perf_stats_image_frame_begin();
 */
void perf_stats_image_frame_begin(void);

/**
 * @brief 标记本帧图像处理结束（Image_Error_Get 后调用）
 * @return 无
 * @sample perf_stats_image_frame_end();
 */
void perf_stats_image_frame_end(void);

/**
 * @brief 速度环 PIT 回调节拍（期望周期约 2ms）
 * @return 无
 * @sample perf_stats_tick_speed_loop();
 */
void perf_stats_tick_speed_loop(void);

/**
 * @brief 角速度环 PIT 回调节拍（期望周期约 6ms）
 * @return 无
 * @sample perf_stats_tick_yaw_spd_loop();
 */
void perf_stats_tick_yaw_spd_loop(void);

/**
 * @brief 角度环 PIT 回调节拍（期望周期约 18ms）
 * @return 无
 * @sample perf_stats_tick_yaw_loop();
 */
void perf_stats_tick_yaw_loop(void);

/**
 * @brief 主循环末尾调用，按 PERF_STATS_PRINT_INTERVAL_MS 节流打印汇总
 * @return 无
 * @sample perf_stats_poll_print();
 */
void perf_stats_poll_print(void);

#if ENABLE_PERF_IMAGE_STAGES

/** image_process 子阶段 ID */
enum PerfImageStage : int
{
    PERF_STAGE_BIN = 0,
    PERF_STAGE_TRACK,
    PERF_STAGE_CROSS_RING,
    PERF_STAGE_FITTING,
    PERF_STAGE_COUNT
};

/**
 * @brief 记录 image_process 子阶段耗时（微秒累加，打印窗口内取平均）
 * @param stage [in] PERF_STAGE_* 枚举
 * @param us    [in] 本阶段耗时（微秒）
 * @return 无
 * @sample perf_stats_image_stage_add(PERF_STAGE_BIN, dt_us);
 */
void perf_stats_image_stage_add(int stage, uint64_t us);

#else

static inline void perf_stats_image_stage_add(int /*stage*/, uint64_t /*us*/) {}

#endif /* ENABLE_PERF_IMAGE_STAGES */

#else

static inline void perf_stats_init(void) {}
static inline void perf_stats_image_frame_begin(void) {}
static inline void perf_stats_image_frame_end(void) {}
static inline void perf_stats_tick_speed_loop(void) {}
static inline void perf_stats_tick_yaw_spd_loop(void) {}
static inline void perf_stats_tick_yaw_loop(void) {}
static inline void perf_stats_poll_print(void) {}
static inline void perf_stats_image_stage_add(int /*stage*/, uint64_t /*us*/) {}

#endif /* ENABLE_PERF_STATS */

#endif /* PERF_STATS_HPP */
