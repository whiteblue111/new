/**
 * @file main.cpp
 * @brief 龙芯 LS2K0300 智能车主程序（走马观碑）
 *
 * 巡线流水线现已对齐 temp_repo：
 *   image_get() → image_process() → Image_Error_Get() → img_err → motor.cpp
 *
 * 逐飞助手数据契约：
 *   - 图像：每 5 帧发一次 320×240 灰度图（image_send_full）
 *   - 边线：每 5 帧发一次 XY_BOUNDARY 3 路（左边线 / 中线 / 右边线），单路 180 点
 *           坐标 uint16，超过当前帧实际点数的位置零填充
 */

#include "zf_common_headfile.hpp"
#include "imgproc.hpp"
#include "image.hpp"
#include "elements.hpp"
#include "display.hpp"
#include "my_key.hpp"
#include "motor.hpp"
#include "vision.hpp"
#include "redbrick.hpp"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace cv;

#define TRACK_DEBUG_LEVEL 2

/** 1=恢复 5ms 速度环 + 6ms 角度环；验证阶段建议低速 g_target_speed */
#define ENABLE_MOTOR_CLOSED_LOOP 1

/** 1=主循环调用 NCNN 碑识别 + 红砖避障状态机 */
#define ENABLE_VISION_BRICK      1


/* ====================== 模型 / 设备对象 ====================== */
ncnn::Net my_net;

zf_driver_tcp_client tcp_client_dev;
zf_device_ips200     ips200;

/* ====================== 定时器 ====================== */
volatile uint32_t g_speed_loop_cnt = 0;
volatile uint32_t g_last_speed_hz  = 0;
float yaw_diff = 0.0f;
zf_driver_pit pit_timer;
zf_driver_pit fps_timer;
zf_driver_pit img_timer;

RedBlockAvoider g_brick_avoider;

/* ====================== 图像发送缓冲 ======================
 * 320×240 整幅灰度图，行优先，连续 76800 字节。
 * image_process() 完成后从 gray_img 拷入，由 seekfree_assistant_camera_send()
 * 一次性发送给逐飞助手。
 */
uint8 rgb_image[240][320];
uint8 gray_image[240][320];
uint8 gray_cut_image[130][320];
uint8 bin_cut_image[130][320];
uint8 bin_bird_image[130][320];

int my_fps = 0;

/* ====================== 边线发送缓冲（XY_BOUNDARY 平铺） ======================
 * 每路最多 POINTS_MAX_LEN = 180 点；超过当前帧实际点数的位置零填充
 * left  : pointsEdgeLeft  （原图坐标，迷宫法直接输出）
 * mid   : CenterEdge      （t_CenterEdge 反透视回原图）
 * right : pointsEdgeRight
 */
uint16 dot_x_left  [POINTS_MAX_LEN] = {0};
uint16 dot_y_left  [POINTS_MAX_LEN] = {0};
uint16 dot_x_mid   [POINTS_MAX_LEN] = {0};
uint16 dot_y_mid   [POINTS_MAX_LEN] = {0};
uint16 dot_x_right [POINTS_MAX_LEN] = {0};
uint16 dot_y_right [POINTS_MAX_LEN] = {0};

/* ====================== 偏差量 ====================== */
float img_err = 0.0f;

/* ====================== 控制量（motor.cpp 在用） ====================== */
volatile float g_target_speed = 1.0f;
volatile float g_u_yaw        = 0.0f;


/* ====================== TCP 包装 ====================== */
uint32 tcp_send_wrap(const uint8 *buf, uint32 len){ return tcp_client_dev.send_data(buf, len); }
uint32 tcp_read_wrap(uint8 *buf, uint32 len)      { return tcp_client_dev.read_data(buf, len); }

extern seekfree_assistant_oscilloscope_struct seekfree_assistant_oscilloscope_data;
#if (1 == SEEKFREE_ASSISTANT_SET_PARAMETR_ENABLE)
extern float  seekfree_assistant_parameter[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];
extern vuint8 seekfree_assistant_parameter_update_flag[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT];
#endif


/**
 * @brief 把 vector<POINT> 平铺到逐飞助手的 XY_BOUNDARY 缓冲区
 * @param pts   POINT 序列
 * @param size  实际点数
 * @param x_buf 横坐标缓冲（uint16，长度 POINTS_MAX_LEN）
 * @param y_buf 纵坐标缓冲（uint16，长度 POINTS_MAX_LEN）
 * @note   超过 size 的位置零填充；坐标会按 [0, COLSIMAGE) × [0, ROWSIMAGE) 限幅
 */
static inline void flatten_points(const std::vector<POINT> &pts, int size,
                                  uint16 *x_buf, uint16 *y_buf)
{
    int n = (size < POINTS_MAX_LEN) ? size : POINTS_MAX_LEN;
    
    if (n == 0) {
        // 如果压根没有点，全置 0 即可
        for (int i = 0; i < POINTS_MAX_LEN; i++) { x_buf[i] = 0; y_buf[i] = 0; }
        return;
    }

    for (int i = 0; i < n; i++)
    {
        int x = pts[i].x;
        int y = pts[i].y;
        if (x < 0) x = 0;
        else if (x >= COLSIMAGE) x = COLSIMAGE - 1;
        if (y < 0) y = 0;
        else if (y >= ROWSIMAGE) y = ROWSIMAGE - 1;
        x_buf[i] = (uint16)x;
        y_buf[i] = (uint16)y;
    }
    
    // 【修改】：使用最后一个有效坐标去填充余下的部分，避免连线飞回 (0,0)
    for (int i = n; i < POINTS_MAX_LEN; i++) { 
        x_buf[i] = x_buf[n - 1]; 
        y_buf[i] = y_buf[n - 1]; 
    }
}


/* ====================== 退出处理 ====================== */
void sigint_handler(int)
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序退出，执行清理操作\n");
    pit_timer.stop();
    fps_timer.stop();
    img_timer.stop();
    motor_stop();
}


/**
 * @brief 主函数入口
 * @return 0 = 正常退出
 */
int main()
{
    motor_init();
    imu_init();
    ips200.init(FB_PATH);
    display_init(&ips200);
    my_key_init();
    vision_init();

    /* ---------- 退出注册 ---------- */
    atexit(cleanup);
    signal(SIGINT,  sigint_handler);
    signal(SIGPIPE, SIG_IGN);

    /* ---------- TCP 连接 + 逐飞助手 ---------- */
    bool tcp_ok = (tcp_client_dev.init(SERVER_IP, PORT) == 0);
    if (tcp_ok)
    {
        seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);
        seekfree_assistant_camera_information_config(
            SEEKFREE_ASSISTANT_GRAY, gray_cut_image[0], 320, 130);
        /* XY_BOUNDARY：3 条边线，每条 POINTS_MAX_LEN 点 */
        seekfree_assistant_camera_boundary_config(
            XY_BOUNDARY, POINTS_MAX_LEN,
            dot_x_left, dot_x_mid, dot_x_right,
            dot_y_left, dot_y_mid, dot_y_right);
    }

    /* ---------- 摄像头初始化 ---------- */
    lq_camera_ex camera(COLSIMAGE, ROWSIMAGE, 60, LQ_CAMERA_0CPU_MJPG, "/dev/video0");
    if (!camera.is_cam_opened())
    {
        printf("[main] 龙邱摄像头初始化失败！\n");
        return -1;
    }

#if ENABLE_MOTOR_CLOSED_LOOP
    g_target_speed = 0.5f;
    pit_timer.init_ms(5, pit_callback_speed);
    img_timer.init_ms(6, yaw_callback_speed);
    printf("[main] 闭环已开: 速度环 5ms + 角度环 6ms, target=%.2f\n", g_target_speed);
#else
    printf("[main] 开环图像验证模式（未启动电机定时器）\n");
#endif

    printf("[main] 初始化完成，开始主循环\n");

    /* ====================== 主循环 ====================== */
    while (1)
    {
        if (!image_get(camera, rgb_img, gray_img, gray_cut_img)) continue;

#if ENABLE_VISION_BRICK
        process_car_vision(rgb_img);
        g_brick_avoider.process(rgb_img, false);
#endif

        /* -------- 巡线主流水线 -------- */
        image_process();
        img_err = Image_Error_Get();
        my_key_poll();

#if ENABLE_VISION_BRICK
        if (g_brick_avoider.get_state() == RB_STATE_AVOIDING
         || g_brick_avoider.get_state() == RB_STATE_RETURNING)
        {
            img_err += g_brick_avoider.get_avoid_offset() * 0.08f;
        }
#endif

        static int disp_cnt = 0;
        if (++disp_cnt % 3 == 0)
        {
            display_show_track();
#if TRACK_DEBUG_LEVEL >= 1
            display_show_debug_hud_phase_c();
            display_show_debug_hud_phase_d();
#endif
        }
#if TRACK_DEBUG_LEVEL >= 2
        track_debug_print_phase_c_throttled();
        track_debug_print_phase_d_throttled();
#endif

        /* -------- 每 5 帧发一次给逐飞助手 -------- */
        static int send_cnt = 0;
        bool send_now = tcp_ok && (++send_cnt % 5 == 0);
        if (send_now)
        {
            /* 320×120 灰度图 */
            if (gray_cut_img.rows == 130 && gray_cut_img.cols == 320)
            {
                if (gray_cut_img.isContinuous())
                {
                    memcpy(gray_cut_image[0], gray_cut_img.data, 320 * 130);
                }
                else
                {
                    for (int r = 0; r < 130; ++r)
                    {
                        memcpy(gray_cut_image[0] + r * 320, gray_cut_img.ptr<uint8_t>(r), 320);
                    }
                }
            }
            /* 边线（XY_BOUNDARY 平铺） */
            flatten_points(pointsEdgeLeft,  pointsEdgeLeft_size,  dot_x_left,  dot_y_left);
            flatten_points(CenterEdge,      CenterEdge_size,      dot_x_mid,   dot_y_mid);
            flatten_points(pointsEdgeRight, pointsEdgeRight_size, dot_x_right, dot_y_right);
        }

        imu_test_call_hz();

        if (send_now)
            seekfree_assistant_camera_send();
    }

    return 0;
}
