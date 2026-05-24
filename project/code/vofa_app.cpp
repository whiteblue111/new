/**
 * @file vofa_app.cpp
 * @brief VOFA+ 与智能车工程的绑定层
 */
#include "vofa_app.hpp"
#include "app_config.h"
#include "motor.hpp"
#include "imu0.hpp"
#include "math.hpp"

extern "C" {
#include "vofa_client.h"
}
#define VOFA_USE_TCP 1
#define VOFA_TCP_IP "192.168.137.1"
#define VOFA_TCP_PORT 1347
#define VOFA_SERIAL_DEVICE "/dev/ttyACM0"

static bool s_vofa_ready = false;

bool vofa_app_init(void)
{
    if (VOFA_Client_Init() == 0)
    {
#if defined(VOFA_USE_TCP) && (VOFA_USE_TCP == 1)
        printf("[vofa] init failed (TCP %s:%d, 请确认 VOFA+ 已开 TCP 服务端)\n",
               VOFA_TCP_IP, VOFA_TCP_PORT);
#else
        printf("[vofa] init failed (serial %s, check device/permissions)\n",
               VOFA_SERIAL_DEVICE);
#endif
        s_vofa_ready = false;
        return false;
    }
    s_vofa_ready = true;
#if defined(VOFA_USE_TCP) && (VOFA_USE_TCP == 1)
    printf("[vofa] init ok, tcp=%s:%d\n", VOFA_TCP_IP, VOFA_TCP_PORT);
#else
    printf("[vofa] init ok, serial=%s\n", VOFA_SERIAL_DEVICE);
#endif
    return true;
}

bool vofa_app_is_ready(void)
{
    return s_vofa_ready && (init_failed_flag == 0);
}

/**
 * @brief 将 VOFA 接收通道映射到控制参数
 * @return 无
 * @note  RX0=目标速度 RX2=速度P RX3=角速度环P RX5=速度I RX6=角速度环D
 *        YAW_SPD_TUNE_MODE=1 时 RX1=g_target_yaw_spd；否则 RX1=pd_yaw.P1 RX4=pd_yaw.D1
 *        改 RX2/RX5 后 speed_reset()；改 RX3/RX6 后 PID_Pos_Reset(pid_yaw_spd)
 */
static void vofa_apply_rx_params(void)
{
    bool speed_pid_changed = false;
    bool yaw_spd_pid_changed = false;

    if (vofa_rev_new_data_flag[0])
    {
        g_target_speed = vofa_rev_data[0];
        vofa_rev_new_data_flag[0] = 0;
    }
    if (vofa_rev_new_data_flag[1])
    {
#if YAW_SPD_TUNE_MODE
        g_target_yaw_spd = limit_float(vofa_rev_data[1], -180.0f, 180.0f);
#else
        pd_yaw.P1 = vofa_rev_data[1];
#endif
        vofa_rev_new_data_flag[1] = 0;
    }
    if (vofa_rev_new_data_flag[2])
    {
        pid_speed_l.P = vofa_rev_data[2];
        pid_speed_r.P = vofa_rev_data[2];
        vofa_rev_new_data_flag[2] = 0;
        speed_pid_changed = true;
    }
    if (vofa_rev_new_data_flag[3])
    {
        pid_yaw_spd.P = vofa_rev_data[3];
        vofa_rev_new_data_flag[3] = 0;
        yaw_spd_pid_changed = true;
    }
#if !YAW_SPD_TUNE_MODE
    if (vofa_rev_new_data_flag[4])
    {
        pd_yaw.D1 = vofa_rev_data[4];
        vofa_rev_new_data_flag[4] = 0;
    }
#endif
    if (vofa_rev_new_data_flag[5])
    {
        pid_speed_l.I = vofa_rev_data[5];
        pid_speed_r.I = vofa_rev_data[5];
        vofa_rev_new_data_flag[5] = 0;
        speed_pid_changed = true;
    }
    if (vofa_rev_new_data_flag[6])
    {
        pid_yaw_spd.D = vofa_rev_data[6];
        vofa_rev_new_data_flag[6] = 0;
        yaw_spd_pid_changed = true;
    }

    if (speed_pid_changed)
        speed_reset();
    if (yaw_spd_pid_changed)
        PID_Pos_Reset(&pid_yaw_spd);
}

void vofa_app_poll(void)
{
    if (!vofa_app_is_ready())
        return;
    VOFA_Receiver_Callback();
    vofa_apply_rx_params();
}

void vofa_app_send_waveform(void)
{
    if (!vofa_app_is_ready())
        return;

    VOFA_Set_Float_Data(0, img_err);
    VOFA_Set_Float_Data(1, g_target_speed);
    VOFA_Set_Float_Data(2, g_u_yaw);
    VOFA_Set_Float_Data(3, g_target_yaw_spd);
    VOFA_Set_Float_Data(4, g_yaw_speed);
    VOFA_Set_Float_Data(5, g_speed_l);
    VOFA_Set_Float_Data(6, g_speed_r);
    VOFA_Set_Float_Data(7, g_speed);
    VOFA_Send_Datas(VOFA_TX_CH_NUM);
}

void vofa_app_shutdown(void)
{
    vofa_port_close();
    s_vofa_ready = false;
}
