#include "perf_stats.hpp"

#if ENABLE_PERF_STATS

#include <atomic>
#include <cstdio>
#include <mutex>
#include <time.h>

namespace {

/**
 * @brief 取单调时钟微秒时间戳
 * @return 自系统启动以来的微秒数
 */
static uint64_t perf_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/** 回调间隔统计窗口 */
struct PeriodWindow
{
    std::mutex   mtx;
    uint64_t     last_us      = 0;
    uint32_t     sample_count = 0;
    double       period_sum   = 0.0;
    float        period_min   = 0.0f;
    float        period_max   = 0.0f;
    bool         has_sample   = false;
};

/** 图像帧统计窗口 */
struct ImageWindow
{
    std::mutex   mtx;
    uint64_t     frame_begin_us = 0;
    uint32_t     frame_count    = 0;
    double       frame_time_sum = 0.0;
    float        frame_time_max = 0.0f;
    bool         has_frame      = false;
};

static PeriodWindow s_speed;
static PeriodWindow s_yaw_spd;
static PeriodWindow s_yaw;
static ImageWindow  s_image;

#if ENABLE_PERF_IMAGE_STAGES
static double       s_stage_sum[PERF_STAGE_COUNT] = {};
static uint32_t     s_stage_frames[PERF_STAGE_COUNT] = {};
#endif

static std::atomic<uint64_t> s_print_last_us{0};

/**
 * @brief 记录一次回调间隔样本
 * @param win  [in,out] 统计窗口
 * @param now  [in]     当前微秒时间戳
 * @return 无
 */
static void period_window_tick(PeriodWindow &win, uint64_t now)
{
    std::lock_guard<std::mutex> lock(win.mtx);

    if (win.last_us != 0)
    {
        const float period_ms = (float)(now - win.last_us) / 1000.0f;
        if (!win.has_sample)
        {
            win.period_min = period_ms;
            win.period_max = period_ms;
            win.has_sample = true;
        }
        else
        {
            if (period_ms < win.period_min) win.period_min = period_ms;
            if (period_ms > win.period_max) win.period_max = period_ms;
        }
        win.period_sum   += (double)period_ms;
        win.sample_count += 1;
    }
    win.last_us = now;
}

/**
 * @brief 重置回调间隔窗口
 * @param win [in,out] 统计窗口
 * @return 无
 */
static void period_window_reset(PeriodWindow &win)
{
    std::lock_guard<std::mutex> lock(win.mtx);
    win.sample_count = 0;
    win.period_sum   = 0.0;
    win.period_min   = 0.0f;
    win.period_max   = 0.0f;
    win.has_sample   = false;
    /* 保留 last_us，避免跨窗口首样本异常 */
}

/**
 * @brief 重置图像帧窗口
 * @return 无
 */
static void image_window_reset(void)
{
    std::lock_guard<std::mutex> lock(s_image.mtx);
    s_image.frame_count    = 0;
    s_image.frame_time_sum = 0.0;
    s_image.frame_time_max = 0.0f;
    s_image.has_frame      = false;
    s_image.frame_begin_us = 0;
}

#if ENABLE_PERF_IMAGE_STAGES
static void image_stage_reset(void)
{
    for (int i = 0; i < PERF_STAGE_COUNT; ++i)
    {
        s_stage_sum[i]    = 0.0;
        s_stage_frames[i] = 0;
    }
}
#endif

/**
 * @brief 读取并打印单个回调通道统计
 * @param label [in] 通道名称
 * @param win   [in,out] 统计窗口
 * @return 无
 */
static void print_period_channel(const char *label, PeriodWindow &win)
{
    std::lock_guard<std::mutex> lock(win.mtx);

    if (!win.has_sample || win.sample_count == 0)
    {
        printf("%s: n/a", label);
        return;
    }

    const float avg_ms = (float)(win.period_sum / (double)win.sample_count);
    printf("%s: %.2fms (min=%.1f max=%.1f)",
           label, avg_ms, win.period_min, win.period_max);
}

} /* namespace */

/**
 * @brief 性能统计模块初始化
 * @return 无
 * @sample perf_stats_init();
 */
void perf_stats_init(void)
{
    s_print_last_us.store(perf_now_us(), std::memory_order_relaxed);
    period_window_reset(s_speed);
    period_window_reset(s_yaw_spd);
    period_window_reset(s_yaw);
    image_window_reset();
#if ENABLE_PERF_IMAGE_STAGES
    image_stage_reset();
#endif
}

/**
 * @brief 记录 image_process 子阶段耗时（微秒累加）
 * @param stage [in] PERF_STAGE_* 枚举
 * @param us    [in] 本阶段耗时（微秒）
 * @return 无
 */
void perf_stats_image_stage_add(int stage, uint64_t us)
{
#if ENABLE_PERF_IMAGE_STAGES
    if (stage < 0 || stage >= PERF_STAGE_COUNT)
        return;
    s_stage_sum[stage] += (double)us;
    s_stage_frames[stage] += 1;
#else
    (void)stage;
    (void)us;
#endif
}

/**
 * @brief 标记本帧图像处理开始（image_get 成功后调用）
 * @return 无
 * @sample perf_stats_image_frame_begin();
 */
void perf_stats_image_frame_begin(void)
{
    const uint64_t now = perf_now_us();
    std::lock_guard<std::mutex> lock(s_image.mtx);
    s_image.frame_begin_us = now;
}

/**
 * @brief 标记本帧图像处理结束（Image_Error_Get 后调用）
 * @return 无
 * @sample perf_stats_image_frame_end();
 */
void perf_stats_image_frame_end(void)
{
    const uint64_t now = perf_now_us();

    std::lock_guard<std::mutex> lock(s_image.mtx);
    if (s_image.frame_begin_us == 0)
        return;

    const float frame_ms = (float)(now - s_image.frame_begin_us) / 1000.0f;
    s_image.frame_count    += 1;
    s_image.frame_time_sum += (double)frame_ms;
    if (!s_image.has_frame || frame_ms > s_image.frame_time_max)
        s_image.frame_time_max = frame_ms;
    s_image.has_frame = true;
    s_image.frame_begin_us = 0;
}

/**
 * @brief 速度环 PIT 回调节拍（期望周期约 2ms）
 * @return 无
 * @sample perf_stats_tick_speed_loop();
 */
void perf_stats_tick_speed_loop(void)
{
    period_window_tick(s_speed, perf_now_us());
}

/**
 * @brief 角速度环 PIT 回调节拍（期望周期约 6ms）
 * @return 无
 * @sample perf_stats_tick_yaw_spd_loop();
 */
void perf_stats_tick_yaw_spd_loop(void)
{
    period_window_tick(s_yaw_spd, perf_now_us());
}

/**
 * @brief 角度环 PIT 回调节拍（期望周期约 18ms）
 * @return 无
 * @sample perf_stats_tick_yaw_loop();
 */
void perf_stats_tick_yaw_loop(void)
{
    period_window_tick(s_yaw, perf_now_us());
}

/**
 * @brief 主循环末尾调用，按 PERF_STATS_PRINT_INTERVAL_MS 节流打印汇总
 * @return 无
 * @sample perf_stats_poll_print();
 * @note  输出示例：[perf] img: 38.2 fps avg=26.1ms max=41.3ms | speed: ...
 */
void perf_stats_poll_print(void)
{
    const uint64_t now = perf_now_us();
    const uint64_t last = s_print_last_us.load(std::memory_order_relaxed);
    const uint64_t interval_us =
        (uint64_t)PERF_STATS_PRINT_INTERVAL_MS * 1000ULL;

    if (now - last < interval_us)
        return;

    s_print_last_us.store(now, std::memory_order_relaxed);

    float img_fps     = 0.0f;
    float img_avg_ms  = 0.0f;
    float img_max_ms  = 0.0f;
    bool  img_valid   = false;

    {
        std::lock_guard<std::mutex> lock(s_image.mtx);
        if (s_image.has_frame && s_image.frame_count > 0)
        {
            img_valid  = true;
            img_fps    = (float)s_image.frame_count * 1000.0f / (float)PERF_STATS_PRINT_INTERVAL_MS;
            img_avg_ms = (float)(s_image.frame_time_sum / (double)s_image.frame_count);
            img_max_ms = s_image.frame_time_max;
        }
    }

    printf("[perf] ");
    if (img_valid)
        printf("img: %.1f fps  avg=%.1fms max=%.1fms | ", img_fps, img_avg_ms, img_max_ms);
    else
        printf("img: n/a | ");

    print_period_channel("speed", s_speed);
    printf(" | ");
    print_period_channel("yaw_spd", s_yaw_spd);
    printf(" | ");
    print_period_channel("yaw", s_yaw);
    printf("\n");

#if ENABLE_PERF_IMAGE_STAGES
    {
        bool any = false;
        for (int i = 0; i < PERF_STAGE_COUNT; ++i)
        {
            if (s_stage_frames[i] > 0) { any = true; break; }
        }
        if (any)
        {
            printf("[perf] stages(ms): bin=%.2f track=%.2f cross_ring=%.2f fit=%.2f\n",
                   s_stage_frames[PERF_STAGE_BIN] ?
                       (float)(s_stage_sum[PERF_STAGE_BIN] / (double)s_stage_frames[PERF_STAGE_BIN] / 1000.0) : 0.f,
                   s_stage_frames[PERF_STAGE_TRACK] ?
                       (float)(s_stage_sum[PERF_STAGE_TRACK] / (double)s_stage_frames[PERF_STAGE_TRACK] / 1000.0) : 0.f,
                   s_stage_frames[PERF_STAGE_CROSS_RING] ?
                       (float)(s_stage_sum[PERF_STAGE_CROSS_RING] / (double)s_stage_frames[PERF_STAGE_CROSS_RING] / 1000.0) : 0.f,
                   s_stage_frames[PERF_STAGE_FITTING] ?
                       (float)(s_stage_sum[PERF_STAGE_FITTING] / (double)s_stage_frames[PERF_STAGE_FITTING] / 1000.0) : 0.f);
        }
    }
#endif
    fflush(stdout);

    image_window_reset();
#if ENABLE_PERF_IMAGE_STAGES
    image_stage_reset();
#endif
    period_window_reset(s_speed);
    period_window_reset(s_yaw_spd);
    period_window_reset(s_yaw);
}

#endif /* ENABLE_PERF_STATS */
