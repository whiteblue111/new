/**
 * @file my_key.cpp
 * @brief 主板按键：KEY_1 复位状态机，KEY_2 切换显示，KEY_3 保存 vision 64×64 ROI
 */

#include "my_key.hpp"
#include "image.hpp"
#include "display.hpp"
#include "vision.hpp"

#ifndef ENABLE_VISION_BRICK
#define ENABLE_VISION_BRICK 1
#endif

#define MY_KEY_RESET_PATH    ZF_GPIO_KEY_1
#define MY_KEY_MODE_PATH     ZF_GPIO_KEY_2
#define MY_KEY_SNAPSHOT_PATH ZF_GPIO_KEY_3
#define MY_KEY_DEBOUNCE_CNT  3

static zf_driver_gpio s_key1(MY_KEY_RESET_PATH,    O_RDWR);
static zf_driver_gpio s_key2(MY_KEY_MODE_PATH,     O_RDWR);
static zf_driver_gpio s_key3(MY_KEY_SNAPSHOT_PATH, O_RDWR);

static bool     s_prev_pressed = false;
static bool     s_stable_pressed = false;
static int      s_stable_cnt = 0;
static bool     s_inited = false;

static bool     s_prev_pressed_2   = false;
static bool     s_stable_pressed_2 = false;
static int      s_stable_cnt_2     = 0;

static bool     s_prev_pressed_3   = false;
static bool     s_stable_pressed_3 = false;
static int      s_stable_cnt_3     = 0;

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
 * @brief 初始化按键 GPIO（KEY_1 复位、KEY_2 显示模式、KEY_3 保存 ROI）
 * @param 无 无
 * @return 无
 */
void my_key_init(void)
{
    s_prev_pressed = false;
    s_stable_pressed = false;
    s_stable_cnt = 0;

    s_prev_pressed_2 = false;
    s_stable_pressed_2 = false;
    s_stable_cnt_2 = 0;

    s_prev_pressed_3 = false;
    s_stable_pressed_3 = false;
    s_stable_cnt_3 = 0;

    s_inited = true;
}

/**
 * @brief 轮询 KEY_1 / KEY_2 / KEY_3
 * @param 无 无
 * @return 无
 * @sample my_key_poll();
 * @note  KEY_1 边沿复位巡线状态机；KEY_2 按下沿执行三态显示循环；KEY_3 边沿保存 64×64 ROI 到 /home/root/picture/
 */
void my_key_poll(void)
{
    if (!s_inited) return;

    /* ----- KEY_1：复位十字/环岛状态机 ----- */
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

    /* ----- KEY_2：切换显示屏模式（三态循环） ----- */
    const bool raw_pressed_2 = key_is_pressed(s_key2.get_level());
    if (raw_pressed_2 == s_stable_pressed_2)
    {
        s_stable_cnt_2 = 0;
    }
    else
    {
        s_stable_cnt_2++;
        if (s_stable_cnt_2 >= MY_KEY_DEBOUNCE_CNT)
        {
            s_stable_pressed_2 = raw_pressed_2;
            s_stable_cnt_2 = 0;
        }
    }
    if (!s_prev_pressed_2 && s_stable_pressed_2)
    {
        display_toggle_mode();
    }
    s_prev_pressed_2 = s_stable_pressed_2;

    /* ----- KEY_3：保存 vision 64×64 ROI 到 /home/root/picture/ ----- */
    const bool raw_pressed_3 = key_is_pressed(s_key3.get_level());
    if (raw_pressed_3 == s_stable_pressed_3)
    {
        s_stable_cnt_3 = 0;
    }
    else
    {
        s_stable_cnt_3++;
        if (s_stable_cnt_3 >= MY_KEY_DEBOUNCE_CNT)
        {
            s_stable_pressed_3 = raw_pressed_3;
            s_stable_cnt_3 = 0;
        }
    }
    if (!s_prev_pressed_3 && s_stable_pressed_3)
    {
#if ENABLE_VISION_BRICK
        vision_save_last_roi_to_picture();
#else
        printf("[my_key] 视觉未编译，无法保存 ROI\n");
#endif
    }
    s_prev_pressed_3 = s_stable_pressed_3;
}
