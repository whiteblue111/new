#ifndef MY_KEY_HPP
#define MY_KEY_HPP

#include "zf_common_headfile.hpp"

/**
 * @brief 初始化按键 GPIO（KEY_1/2/3）
 * @return 无
 * @sample my_key_init();
 */
void my_key_init(void);

/**
 * @brief 轮询 KEY_1/2/3
 * @return 无
 * @sample my_key_poll();
 * @note  KEY_1 复位十字/环岛；KEY_2 单击按三态循环切换显示模式；KEY_3 保存 vision 64×64 到 /home/root/picture/；需在 image_process() 之后调用
 */
void my_key_poll(void);

#endif /* MY_KEY_HPP */
