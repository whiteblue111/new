#ifndef IMU0_HPP  
#define IMU0_HPP  
  
#include "zf_common_headfile.hpp"  
  
/* ============ 配置 ============ */  
#define IMU_ZERO_COUNT          500    // 零漂采样次数（500×6ms ≈ 3秒）  
#define IMU_DT_MS               6      // 角速度环 6ms 采样周期  
#define IMU_GYRO_LSB_PER_DPS    16.4f  // IMU660RA ±2000 dps 量程下的灵敏度 (LSB/(deg/s))  
#define IMU_GYRO_ALPHA          0.5f   // X轴陀螺仪一阶低通滤波系数 (0~1，越小越平滑)  
#define IMU_GYRO_GAIN_POS       2.36f // 逆时针(raw_minus_zero>=0)方向 dps 增益，实测360°时单独调  
#define IMU_GYRO_GAIN_NEG       2.64f  // 顺时针(raw_minus_zero<0) 方向 dps 增益，实测360°时单独调  
  
/* ============ 全局变量 ============ */  
extern volatile float g_yaw_speed;     // 校准+滤波后的 yaw 角速度（deg/s）  
extern volatile float g_angle_yaw;     // 积分航向角（deg，环岛用）  
extern volatile bool  g_imu_ready;     // 零漂校准完成标志  

extern volatile uint32_t g_imu_call_cnt;      // 当前1秒内调用次数  
extern volatile uint32_t g_imu_call_hz;
  
/* ============ 接口 ============ */  
void  imu_init(void);  
void  imu_read(void);         // 6ms 角速度环回调中调用  
void  imu_reset_angle(void);  
float imu_get_yaw_speed(void);  
float imu_get_angle(void);  
void  imu_test_call_hz(void); // 主循环调用，每秒打印 imu_read 实际触发频率  
  
#endif  