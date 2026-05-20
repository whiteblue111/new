#ifndef MY_KEY_HPP
#define MY_KEY_HPP

#include "zf_common_headfile.hpp"

/**
 * @brief 初始化按键 GPIO（KEY_1）
 * @return 无
 * @sample my_key_init();
 */
void my_key_init(void);

/**
 * @brief 轮询 KEY_1，按下边沿时复位十字/环岛状态机
 * @return 无
 * @sample my_key_poll();
 * @note  需在 image_process() 之后调用；含 3 帧消抖
 */
void my_key_poll(void);

#endif /* MY_KEY_HPP */
