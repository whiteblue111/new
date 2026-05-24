#ifndef VISION_HPP
#define VISION_HPP
#include "zf_common_headfile.hpp"
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>
#include <vector>
#include <string>

// 声明外部的 NCNN 网络对象 (真身在 main.cpp 里定义)
extern ncnn::Net my_net;

// ==========================================
// 常量定义
// ==========================================
const int   MODEL_INPUT_WIDTH = 64;
const float RED_ROW_RATIO = 0.05f;     
const int   RED_CONFIRM_ROWS = 2;      
const int   CROP_EDGE_PADDING = 5; 

extern int block_w;
extern int block_h;

extern bool  g_is_bypassing_binoculars;  // 是否正在执行望远镜绕行
extern int   g_bypass_timer;             // 绕行持续帧数计时器
extern const int BYPASS_MAX_FRAMES;      // 绕行持续时间（假设30帧/秒，80帧大约2.5秒，根据你的车速调）
extern const float BYPASS_OFFSET;        // 向左绕行的偏移量（像素），越大绕得越宽

// ==========================================
// 红色目标检测与裁切类声明
// ==========================================
class RedRectDetector {
public:
    cv::Rect target_rect;
    RedRectDetector() {}

    // 核心函数：HSV过滤 + 目标裁切
    bool model_roi_cut(cv::Mat& img, cv::Mat& roi, bool is_draw = true);

private:
    // 直接在 BGR 色彩空间下判断红色 (极速版)
    inline bool is_red_bgr(const cv::Vec3b& bgr) {
        int b = bgr[0];
        int g = bgr[1];
        int r = bgr[2];

        return (r > 80) && ((r - g) > 40) && ((r - b) > 40);
    }
    bool find_red_block(cv::Mat& img, int& block_top, int& block_bottom, int& x_left, int& x_right);
};

// ==========================================
// 全局视觉流水线接口
// ==========================================
// ==========================================
// 视觉模块初始化（加载 NCNN 模型、配置线程/精度）
// ==========================================
void vision_init(void);
void process_car_vision(cv::Mat& frame);

/** 主循环阶段探针 ID（ENABLE_TERMINAL_DEBUG 时由 main 写入） */
extern volatile sig_atomic_t g_dbg_stage_id;


#endif // VISION_MODULE_HPP