#ifndef MOTOR_HPP 
#define MOTOR_HPP  
#include "zf_common_headfile.hpp"  
#include "pid.hpp"  
  
/* ====================== 引脚配置 ====================== */  
#define PWM_1_PATH ZF_PWM_MOTOR_1  
#define DIR_1_PATH ZF_GPIO_MOTOR_1  
#define PWM_2_PATH ZF_PWM_MOTOR_2  
#define DIR_2_PATH ZF_GPIO_MOTOR_2  
  
/* ====================== 参数配置 ====================== */  
#define MAX_DUTY             30  
#define MAX_PWM              3000 
#define ENCODER_QUAD_1_PATH  ZF_ENCODER_QUAD_1  
#define ENCODER_QUAD_2_PATH  ZF_ENCODER_QUAD_2  
#define ENCODER_LINE_NUM     2340.0f  
#define WHEEL_RADIUS         0.0325f  
#define DT_SPEED             0.002f  
/*=======================死区============================*/
#define DEAD_L               1500
#define DEAD_R               1500

// 渐进式死区补偿参数：PID输出在 [0, RAMP_WIDTH] 范围内线性渐变补偿量
// RAMP_WIDTH 越大过渡越平滑（但低速响应稍慢），建议设为死区值的 0.5~1.0 倍
#define MOTOR_DEAD_RAMP_WIDTH       1500.0f
  
/* ====================== 对外状态 ====================== */  
extern volatile float g_speed_l;  
extern volatile float g_speed_r;  
extern volatile float g_speed;  
extern volatile int16 g_enc_l;  
extern volatile int16 g_enc_r;  
  
/* ====================== 控制参数 ====================== */  
extern volatile float g_target_speed;   
extern volatile float g_u_yaw;  
extern float img_err;
  
/* ====================== PID结构体 ====================== */  
extern PID_Inc_Datatypedef pid_speed_l;  // 速度环：增量式  
extern PID_Inc_Datatypedef pid_speed_r;  // 速度环：增量式  
extern PID_Pos_Datatypedef pid_dir;      // 角度环：位置式  
  
/* ====================== 接口函数 ====================== */  
void motor_init();  
void motor_stop();  
void motor_set_lr(int duty_l, int duty_r);  
void update_direction();                // 角度环（位置式）  
void speed_reset();                     // 目标速度变更时重置速度环状态  
void pit_callback_speed();              // 注册给 pit_timer（速度环增量式）  
void run_speed_loop();                  // 主循环手动调用（调试用）  
void speed_parallel_5ms();
void get_enconder();
int get_dist ();
void yaw_callback_speed();
typedef struct motor_param_t {
    float enc;//原始数据
    float encb;//滤波
    float enc_total;
    float spd;//实际速度
    float dist;//实际距离
    float target_encoder;
    float enc_last;
    float target_speed;
    int32_t duty;         //Motor PWM duty
} motor_param_t;
extern motor_param_t motor_l,motor_r;
static int32_t apply_smooth_deadzone(int32_t duty, int32_t deadzone, float ramp_width);
extern volatile uint32_t g_speed_loop_cnt;  
void speed_cascaded_5ms();  
void yaw_loop();  
void yaw_spd_loop();


#endif 
