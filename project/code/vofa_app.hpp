/**
 * @file vofa_app.hpp
 * @brief VOFA+ 与智能车工程的绑定层（LS2K0300）
 *
 * 上位机 VOFA+ 设置：
 *   - 数据引擎：JustFloat
 *   - 串口波特率：115200（与 VOFA_SERIAL_BAUDRATE 一致）
 *   - 调参命令 Hex：A6 <通道> %% <通道>（见 libraries/zf_components/vofa_plus/examples）
 */
#ifndef VOFA_APP_HPP
#define VOFA_APP_HPP

#include "zf_common_headfile.hpp"

/** 发送波形通道数 */
#define VOFA_TX_CH_NUM  8

/**
 * @brief 初始化 VOFA 客户端（打开串口 + 协议）
 * @return true=成功，false=串口不可用（主程序可继续跑，仅无 VOFA）
 */
bool vofa_app_init(void);

/**
 * @brief 轮询接收并应用上位机下发的参数
 * @return 无
 * @note  主循环每帧调用；RX 映射见 VOFA使用说明.md
 */
void vofa_app_poll(void);

/**
 * @brief 发送当前调试量到 VOFA+ 波形
 * @return 无
 * @note  TX0=img_err TX1=target_speed TX2=u_yaw TX3=target_yaw_spd TX4=yaw_speed
 *        TX5=speed_l TX6=speed_r TX7=speed_avg
 *        RX0=目标速度 RX2=速度P RX3=角速度环P RX5=速度I RX6=角速度环D
 *        YAW_SPD_TUNE_MODE=1 时 RX1=目标角速度(deg/s)；否则 RX1=角度P RX4=角度D
 */
void vofa_app_send_waveform(void);

/**
 * @brief VOFA 是否已成功初始化
 * @return true=可用
 */
bool vofa_app_is_ready(void);

/**
 * @brief 关闭 VOFA 串口
 * @return 无
 */
void vofa_app_shutdown(void);

#endif /* VOFA_APP_HPP */
