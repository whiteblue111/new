#include "motor.hpp"  
#include "pid.hpp"  
#include "imu0.hpp"  
#include "perf_stats.hpp"
#include "zf_common_headfile.hpp"  
#include <fstream>
#include <string>
  
/* ====================== 设备对象（模块私有） ====================== */  
static zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR); // 右  
static zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR); // 左  
static zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);  
static zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);  
static struct pwm_info drv8701e_pwm_1_info;  
static struct pwm_info drv8701e_pwm_2_info;  
  
#define MOTOR1_PWM_DUTY_MAX (drv8701e_pwm_1_info.duty_max)  
#define MOTOR2_PWM_DUTY_MAX (drv8701e_pwm_2_info.duty_max)  
  
static zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH); // 左  
static zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH); // 右  
/* ====================== 全局状态定义 ====================== */  
motor_param_t motor_l,motor_r;  
float loss_pass = 0.4;//编码器低通滤波系数  
volatile float g_speed_l = 0.0f;  
volatile float g_speed_r = 0.0f;  
volatile float g_speed   = 0.0f;  
volatile int16 g_enc_l   = 0;  
volatile int16 g_enc_r   = 0;  
//编码器累计度数  
volatile int16 total_enc_l = 0;  
volatile int16 total_enc_r = 0;  
volatile float g_target_yaw_spd = 0.0f;  

  
/* ====================== PID结构体定义 ====================== */  
/*  
 * 速度环：增量式PID  
 *  
 * 角度环：位置式PID  
 */  
// PID_Inc_Datatypedef pid_speed_l = PID_INC_INIT(89.25f, 11.38f, 0.02f);  
// PID_Inc_Datatypedef pid_speed_r = PID_INC_INIT(89.25f, 11.38f, 0.02f); 
// PID_Pos_Datatypedef pid_dir     = PID_POS_INIT(0.0f, 0.0f,  0.0f, 0.0f);  
// PD_Double  pd_yaw               = PD_DOUBLE_INIT(0.03f, 0.0f, 0.0f, 0.018f);  
  
/* VOFA 在线调参后请同步写回此处 P/I/D，见 project/code/VOFA使用说明.md */
PID_Inc_Datatypedef pid_speed_l = PID_INC_INIT(1000.00f, 10.00f, 0.0f);  //5.00 15.00 0.0
PID_Inc_Datatypedef pid_speed_r = PID_INC_INIT(1000.00f, 10.00f, 0.0f); 
// PID_Pos_Datatypedef pid_dir     = PID_POS_INIT(0.0f, 0.0f,  0.0f, 0.0f);  
PID_Pos_Datatypedef pid_yaw_spd = PID_POS_INIT(0.012f, 0.0f, 0.00f, 0.0f);   //角速度环位置式PD，P=0.015 I=0 D=0.006
PD_Double  pd_yaw               = PD_DOUBLE_INIT(5.2f, 1.5f, 1.0f, 0.00f); //双pd角度环 5.0 1.0 0.5 0.002
/* ====================== 增量式速度环内部累计占空比 ====================== */  
static float duty_l_out = 0.0f;  
static float duty_r_out = 0.0f;  
/* ====================== motor_init ====================== */  
void motor_init()  
{  
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);  
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);  
}  
  
/* ====================== motor_stop ====================== */  
void motor_stop()  
{  
    drv8701e_pwm_1.set_duty(0);  
    drv8701e_pwm_2.set_duty(0);  
}  
  
/* ====================== motor_set_lr ====================== */  
void motor_set_lr(int duty_l, int duty_r)  
{  
    duty_l = apply_smooth_deadzone(duty_l,DEAD_L,MOTOR_DEAD_RAMP_WIDTH);  
    duty_r = apply_smooth_deadzone(duty_r,DEAD_R,MOTOR_DEAD_RAMP_WIDTH);  
    duty_l = limit_int(duty_l, -MAX_PWM, MAX_PWM);  
    duty_r = limit_int(duty_r, -MAX_PWM, MAX_PWM);  
  
  
  
    /* 左轮（电机2） */  
    if (duty_l >= 0)  
    {  
        drv8701e_dir_2.set_level(1);  
        drv8701e_pwm_2.set_duty(duty_l );  
    }  
    else  
    {  
        drv8701e_dir_2.set_level(0);  
        drv8701e_pwm_2.set_duty(-duty_l);  
    }  
  
    /* 右轮（电机1） */  
    if (duty_r >= 0)  
    {  
        drv8701e_dir_1.set_level(1);  
        drv8701e_pwm_1.set_duty(duty_r );  
    }  
    else  
    {  
        drv8701e_dir_1.set_level(0);  
        drv8701e_pwm_1.set_duty((-duty_r) );  
    }  
}  
/*--------------------------------------------编码器读取-------------------------------------------------*/  
void get_enconder()  
{  
    motor_l.enc = -encoder_quad_1.get_count();  //编码器完全相同，所以一正一负  
    motor_r.enc =  encoder_quad_2.get_count(); 
    motor_l.spd =  ((float)motor_l.enc / ENCODER_LINE_NUM) * 2.0f * 3.1415926f / DT_SPEED* WHEEL_RADIUS;
    motor_r.spd =  ((float)motor_r.enc / ENCODER_LINE_NUM) * 2.0f * 3.1415926f / DT_SPEED* WHEEL_RADIUS;

    
    motor_l.enc_last = motor_l.enc;  
    motor_r.enc_last = motor_r.enc;  

    motor_l.enc_total+= motor_l.enc;  
    motor_r.enc_total+= motor_r.enc;

    motor_l.dist = motor_l.enc_total * 2.0f * 3.1415926f * WHEEL_RADIUS / ENCODER_LINE_NUM;
    motor_r.dist = motor_r.enc_total * 2.0f * 3.1415926f * WHEEL_RADIUS / ENCODER_LINE_NUM;


    encoder_quad_1.clear_count();  
    encoder_quad_2.clear_count();  
}  
/*-------------------------------------------读取距离---------------------------- ------------------------*/  
int get_dist ()  
{  
    return ((motor_l.enc_total+motor_r.enc_total) / 2);  
}  
  
/* ====================== 速度环（增量式，并环，5ms） ====================== */  
 void speed_parallel_5ms()  
{  
    get_enconder();  
 
    g_speed_l = motor_l.spd;  
    g_speed_r = motor_r.spd;  

    g_speed   = 0.5f * (g_speed_l + g_speed_r);  
    duty_l_out += PID_Inc(&pid_speed_l, g_target_speed - g_speed_l);  
    duty_r_out += PID_Inc(&pid_speed_r, g_target_speed - g_speed_r);  
    duty_l_out = limit_float(duty_l_out, -MAX_PWM, MAX_PWM);  
    duty_r_out = limit_float(duty_r_out, -MAX_PWM, MAX_PWM);  
    // int duty_l = (int)limit_float(duty_l_out - g_u_yaw, -MAX_DUTY, MAX_DUTY);  
    // int duty_r = (int)limit_float(duty_r_out + g_u_yaw, -MAX_DUTY, MAX_DUTY);  
    motor_set_lr(duty_l_out, duty_r_out);  
    // g_speed_loop_cnt++;//统计pid计算时间  
  
}  
 /* ====================== 速度环（增量式，串环，5ms） ====================== */  
 void speed_cascaded_5ms()  
 {  
    static int dbg_speed_cb_cnt = 0;
    dbg_speed_cb_cnt++;


    get_enconder();  
    /* 计算线速度 m/s */  

    g_speed_l = motor_l.spd;  
    g_speed_r = motor_r.spd;  
    // g_speed_l = motor_l.enc;  
    // g_speed_r = motor_r.enc;  
    g_speed   = 0.5f * (g_speed_l + g_speed_r);  
    duty_l_out += PID_Inc(&pid_speed_l, g_target_speed - g_u_yaw - g_speed_l);  
    duty_r_out += PID_Inc(&pid_speed_r, g_target_speed + g_u_yaw - g_speed_r);  
    duty_l_out = limit_float(duty_l_out, -MAX_PWM, MAX_PWM);  
    duty_r_out = limit_float(duty_r_out, -MAX_PWM, MAX_PWM);  
    // duty_l_out = limit_float(3000, -MAX_PWM, MAX_PWM);  
    // duty_r_out = limit_float(3000, -MAX_PWM, MAX_PWM);  


    motor_set_lr((int)duty_l_out, (int)duty_r_out);  
    // g_speed_loop_cnt++;//统计pid计算时间    
 }  
/* ====================== 角度环（位置式，主循环调用） ====================== */  
void update_direction()  
{  
    /* 位置式PID直接输出g_u_yaw */  
    // img_err_get();
    g_u_yaw = PID_Pos(&pid_dir, img_err);  
    g_u_yaw = limit_float(g_u_yaw, -12.0f, 12.0f);  
}  
/**
 * @brief 角度外环（18ms）：图像偏差经双 PD 得到角速度给定
 * @return 无
 * @note   不调用 imu_read，使用 6ms 角速度环已更新的 g_yaw_speed；公式与 pd_yaw 不变
 */
void yaw_loop()
{
    if (!g_imu_ready)
        return;
    g_target_yaw_spd = PD_Loc_Ctrl_2PD(&pd_yaw, img_err, g_yaw_speed);
    g_target_yaw_spd = limit_float(g_target_yaw_spd, -180.0f, 180.0f);
}

/**
 * @brief 角速度中环（6ms）：位置式 PD，跟踪外环给定
 * @return 无
 * @note   输出 g_u_yaw 作为左右轮差速，单位与速度环目标叠加一致（m/s）
 */
void yaw_spd_loop()
{
    imu_read();
    if (!g_imu_ready)
    {
        g_u_yaw = 0.0f;
        return;
    }
    // g_target_yaw_spd = 10.0f;
    const float err_w = g_target_yaw_spd - g_yaw_speed;
    g_u_yaw = PID_Pos(&pid_yaw_spd, err_w);
    g_u_yaw = limit_float(g_u_yaw, -5.0f, 5.0f);
}

/* ====================== 速度环重置（目标速度变更时调用） ====================== */  
void speed_reset()  
{  
    PID_Inc_Reset(&pid_speed_l);  
    PID_Inc_Reset(&pid_speed_r);  
    PID_Pos_Reset(&pid_yaw_spd);
    duty_l_out = 0.0f;  
    duty_r_out = 0.0f;  
}  

/**
 * @brief 2ms PIT 回调：仅内环速度控制
 * @return 无
 */
void pit_callback_speed(void)
{
    perf_stats_tick_speed_loop();
    speed_cascaded_5ms();
}

/**
 * @brief 6ms PIT 回调：陀螺仪采样 + 角速度环
 * @return 无
 */
void pit_callback_yaw_spd(void)
{
    perf_stats_tick_yaw_spd_loop();
    yaw_spd_loop();
}

/**
 * @brief 18ms PIT 回调：角度外环
 * @return 无
 */
void pit_callback_yaw(void)
{
    perf_stats_tick_yaw_loop();
    yaw_loop();
}
  
/* ====================== run_speed_loop（主循环手动调用，调试用） ====================== */  
void run_speed_loop()  
{  
    speed_cascaded_5ms();  
}  

/**
 * @brief 角度环回调（兼容旧接口，等同 pit_callback_yaw）
 * @return 无
 */
void yaw_callback_speed(void)
{
    pit_callback_yaw();
}  
// ============================ 渐进式死区补偿 ============================  
/**  
 * @brief 渐进式死区补偿，消除低速前馈跳变导致的高频抖动  
 * PID输出绝对值从0到ramp_width范围内，补偿量从0平滑增加到deadzone（smoothstep）  
 * 超过ramp_width后补偿满值，全程无阶跃跳变  
 */  
static int32_t apply_smooth_deadzone(int32_t duty, int32_t deadzone, float ramp_width)  
{  
    float abs_duty = fabsf((float)duty);  
    if (abs_duty < 1.0f) return 0;  // PID输出≈0，电机不动  
  
    // 防御性保护，避免除0和异常参数  
    if (ramp_width < 1.0f) {  
        ramp_width = 1.0f;  
    }  
  
    // 平滑比例：smoothstep曲线（0处斜率为0），比线性更不易触发低速振荡  
    float t = abs_duty / ramp_width;  
    if (t > 1.0f) t = 1.0f;  
    float scale = t * t * (3.0f - 2.0f * t);  
  
    // 四舍五入，降低整型截断带来的量化抖动  
    int32_t compensation = (int32_t)((float)deadzone * scale + 0.5f);  
  
    if (duty > 0) return duty + compensation;  
    else          return duty - compensation;  
}  

