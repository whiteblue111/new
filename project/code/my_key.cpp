/**
 * @file my_key.cpp
 * @brief 主板按键：KEY_1 按下时复位十字/环岛状态机
 */

#include "my_key.hpp"
#include "image.hpp"

#define MY_KEY_RESET_PATH   ZF_GPIO_KEY_1
#define MY_KEY_DEBOUNCE_CNT 3

static zf_driver_gpio s_key1(MY_KEY_RESET_PATH, O_RDWR);

static bool     s_prev_pressed = false;
static bool     s_stable_pressed = false;
static int      s_stable_cnt = 0;
static bool     s_inited = false;

/**
 * @brief 判断按键是否处于按下态
 * @param lv  get_level() 返回值（0/1 或 ASCII '0'/'1'）
 * @return    true 表示按下
 */
static bool key_is_pressed(uint8 lv)
{
    return (lv == 0 || lv == '0');
}

/**
 * @brief 初始化按键 GPIO（KEY_1）
 * @return 无
 */
void my_key_init(void)
{
    s_prev_pressed = false;
    s_stable_pressed = false;
    s_stable_cnt = 0;
    s_inited = true;
}

/**
 * @brief 轮询 KEY_1，按下边沿时复位十字/环岛状态机
 * @return 无
 * @sample my_key_poll();
 * @note  连续 MY_KEY_DEBOUNCE_CNT 帧电平一致才更新稳定态，避免抖动重复触发
 */
void my_key_poll(void)
{
    if (!s_inited) return;

    const bool raw_pressed = key_is_pressed(s_key1.get_level());

    if (raw_pressed == s_stable_pressed)
    {
        s_stable_cnt = 0;
    }
    else
    {
        s_stable_cnt++;
        if (s_stable_cnt >= MY_KEY_DEBOUNCE_CNT)
        {
            s_stable_pressed = raw_pressed;
            s_stable_cnt = 0;
        }
    }

    if (!s_prev_pressed && s_stable_pressed)
    {
        track_elements_reset();
    }

    s_prev_pressed = s_stable_pressed;
}
