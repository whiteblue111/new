#include "zf_common_headfile.hpp"
#include "imu0.hpp"
#include <cstdio>
#include <cmath>
#include <sys/time.h>

/* ============ 设备对象 ============ */
static zf_device_imu imu_dev;

/* ============ 全局状态 ============ */
volatile float g_yaw_speed = 0.0f;
volatile float g_angle_yaw = 0.0f;
volatile bool g_imu_ready = false;

/* ============ 模块私有 ============ */
static float s_zero_sum = 0.0f;
static float s_zero_point = 0.0f;
static int s_zero_count = 0;
static float s_last_yaw_speed = 0.0f;   // 一阶LPF前一次输出（deg/s）

volatile uint32_t g_imu_call_cnt = 0; // 当前1秒内调用次数
volatile uint32_t g_imu_call_hz = 0;  // 上一秒统计值

/* ============ 初始化 ============ */
void imu_init(void)
{
    imu_dev.init();

    if (DEV_IMU660RA == imu_dev.imu_type)
        printf("[imu] 检测到 IMU660RA\n");
    else
    {
        printf("[imu] ❌ 未检测到 IMU！\n");
        return;
    }

    s_zero_sum = 0.0f;
    s_zero_point = 0.0f;
    s_zero_count = 0;
    s_last_yaw_speed = 0.0f;
    g_yaw_speed = 0.0f;
    g_angle_yaw = 0.0f;
    g_imu_ready = false;

    printf("[imu] 开始零漂校准，请保持静止...\n");
}

/* ============  ============ */
void imu_read(void)//2000
{
    g_imu_call_cnt++;
    /* 1. 读取原始值 */
    int16 raw_z = imu_dev.get_gyro_z();

    /* 2. 零漂校准状态机（与参考代码一致） */
    if (s_zero_count < IMU_ZERO_COUNT)
    {
        s_zero_sum += (float)raw_z;
        s_zero_count++;
        g_yaw_speed = 0.0f;
        return; // 校准期间不积分
    }
    else if (s_zero_count == IMU_ZERO_COUNT)
    {
        s_zero_point = s_zero_sum / (float)IMU_ZERO_COUNT;
        s_zero_count++;
        g_imu_ready = true;
        g_angle_yaw = 0.0f;
        s_last_yaw_speed = 0.0f;
        printf("[imu] 零漂校准完成: %.2f\n", s_zero_point);
        return;
    }
    float dt = IMU_DT_MS * 0.001f;
    float raw_minus_zero = (float)raw_z - s_zero_point;
    float gain = (raw_minus_zero >= 0.0f) ? IMU_GYRO_GAIN_POS : IMU_GYRO_GAIN_NEG;
    float cur_yaw_dps = raw_minus_zero / IMU_GYRO_LSB_PER_DPS * gain;

    /* 一阶低通滤波 (参考 IMUHandler::update 的 LPF 思路，只对 Z 轴) */
    float filt_yaw_dps = IMU_GYRO_ALPHA * cur_yaw_dps
                       + (1.0f - IMU_GYRO_ALPHA) * s_last_yaw_speed;
    s_last_yaw_speed = filt_yaw_dps;

    g_yaw_speed  = filt_yaw_dps;          // 单位：deg/s（滤波后）
    g_angle_yaw += filt_yaw_dps * dt;     // 单位：deg

    /* 1 Hz 打印（IMU_DT_MS=6ms → 每 (1000/IMU_DT_MS) 次约 1 秒） */
    static uint32_t print_cnt = 0;
    if (++print_cnt >= (1000U / IMU_DT_MS))
    {
        print_cnt = 0;
        printf("[imu] 原始: %6d, 角速度: %7.2f dps, 角度: %7.2f deg\n",
               raw_z, g_yaw_speed, g_angle_yaw);
    }
}

/* ============ 工具函数 ============ */
void imu_reset_angle(void)
{
    g_angle_yaw = 0.0f;
}

float imu_get_yaw_speed(void)
{
    return g_yaw_speed;
}

float imu_get_angle(void)
{
    return g_angle_yaw;
}

/**
 * @brief 测试 imu_read() 实际调用频率（用于验证陀螺仪采样率是否达到 6ms PIT 期望的 ~166Hz）
 * @return 无；统计结果写入全局 g_imu_call_hz，并每秒 printf 一次
 * @sample 在 main() 的 while(1) 主循环中每次迭代调用 imu_test_call_hz();
 * @note 非阻塞实现，使用 gettimeofday 计时；首次调用仅初始化基准时间戳，
 *       之后每经过 ≥1s 输出一次 Hz 并刷新基准；
 *       避免在 PIT 中断回调里调用 printf，因此放在主循环执行。
 */
void imu_test_call_hz(void)
{
    static struct timeval last_tv = {0, 0};
    static uint32_t last_cnt = 0;
    struct timeval now;
    gettimeofday(&now, nullptr);

    if (last_tv.tv_sec == 0 && last_tv.tv_usec == 0) {
        last_tv = now;
        last_cnt = g_imu_call_cnt;
        return;
    }

    double elapsed = (now.tv_sec - last_tv.tv_sec)
                   + (now.tv_usec - last_tv.tv_usec) * 1e-6;
    if (elapsed >= 1.0) {
        uint32_t delta = g_imu_call_cnt - last_cnt;
        g_imu_call_hz = (uint32_t)(delta / elapsed);
        printf("[imu] 陀螺仪调用频率: %u Hz (%u 次 / %.3f s)\n",
               g_imu_call_hz, delta, elapsed);
        last_tv = now;
        last_cnt = g_imu_call_cnt;
    }
}