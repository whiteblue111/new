#ifndef VISION_HPP
#define VISION_HPP
#include "app_config.h"
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
const int   RED_TOP_GAP_LIMIT = 1;     /* 顶部判定：连续 N 行非红即停，避免与图片区红色（如急救包红十字）粘连 */

/* 图片梯形相对红块的尺寸比例（走马观碑） */
const float PIC_W_RATIO = 1.0f;  /* 图片宽 ≈ block_w，保留宏便于以后扩展 */
const float PIC_H_RATIO = 2.2f;  /* 图片高 = PIC_H_RATIO × block_h，沿碑体两条斜边向上延伸 */

extern int block_w;
extern int block_h;

extern bool  g_is_bypassing_binoculars;  // 是否正在执行望远镜绕行（历史 API，已不再决定中线）
extern int   g_bypass_timer;             // 绕行持续帧数计时器（历史 API）
extern const int BYPASS_MAX_FRAMES;      // 绕行持续时间（历史 API）
extern const float BYPASS_OFFSET;        // 向左绕行的偏移量（历史 API）

/**
 * @brief 模型识别结果对应的中线偏移动作
 * @note  STRAIGHT 不 override 中线；LEFT 用左线 + VISION_BYPASS_SHIFT_PX 列偏移；
 *        RIGHT 用右线 - VISION_BYPASS_SHIFT_PX 列偏移。供 image.cpp fitting() 读取。
 */
enum VisionBypassAction {
    VBA_STRAIGHT = 0,
    VBA_LEFT     = 1,
    VBA_RIGHT    = 2
};

/** 当前由模型识别推出的中线偏移动作；fitting() 在红砖 NORMAL 时读取 */
extern VisionBypassAction g_vision_bypass_action;

/** 视觉绕行的列方向偏移量（俯视图像素），左线 +shift，右线 -shift */
constexpr int VISION_BYPASS_SHIFT_PX = 5;

/** model_roi_cut 连续失败超过该帧数后清零视觉绕行动作 */
constexpr int VISION_LOST_FRAMES     = 10;

/* ====================== 视觉状态缓存（供 display_show_vision 读取） ====================== */
extern cv::Mat g_last_roi;          /* 最近一次 model_roi_cut 成功输出的 64×64 BGR */
extern cv::Mat g_last_saved_roi;    /* KEY_3 最近一次保存成功的 64×64 ROI 深拷贝 */
extern int     g_last_pred_index;   /* 最近一次稳定确认的标签索引，-1 表示无 */
extern float   g_last_pred_prob;    /* 对应置信度（NCNN out 原始 logit） */

/**
 * @brief 最近一帧 model_roi_cut 的执行结果
 * @note  -1 = 初始未运行；0 = OK；1 = find_red_block 失败；2 = block_w/h 超阈值；3 = 图片梯形越界
 */
extern int g_roi_cut_reject_reason;

/** KEY_3 拍照结果（供 display_show_vision 屏显） */
enum VisionSnapshotResult {
    VSNAP_NONE = 0,
    VSNAP_OK,
    VSNAP_FAIL_NO_ROI,
    VSNAP_FAIL_DIR,
    VSNAP_FAIL_IO,
};

extern VisionSnapshotResult g_vision_snapshot_result;
extern int                  g_vision_snapshot_ttl; /* 剩余 display_show_vision 刷新次数，0=不显示 */

/**
 * @brief 取 6 类标签数组（Ambulance / Armored vehicle / Binoculars / Grenade / Guns / medical）
 * @return 标签数组常量引用
 * @sample auto& labels = vision_labels(); printf("%s", labels[i].c_str());
 */
const std::vector<std::string>& vision_labels(void);

// ==========================================
// 红色目标检测与裁切类声明
// ==========================================
class RedRectDetector {
public:
    cv::Rect    target_rect;          /* 图片梯形的轴对齐外接矩形（兼容老 HUD/调试） */
    cv::Point2f picture_quad[4];      /* 图片梯形 4 角，顺序 TL,TR,BR,BL（图像坐标系） */
    RedRectDetector() {}

    // 核心函数：红块 4 角定位 + 透视梯形裁切 → 64×64 ROI
    bool model_roi_cut(cv::Mat& img, cv::Mat& roi, bool is_draw = true);

private:
    // 直接在 BGR 色彩空间下判断红色 (极速版)
    inline bool is_red_bgr(const cv::Vec3b& bgr) {
        int b = bgr[0];
        int g = bgr[1];
        int r = bgr[2];

        return (r > 80) && ((r - g) > 40) && ((r - b) > 40);
    }

    /* 找到红块并输出 4 个倾斜角点，顺序固定为 TL_r, TR_r, BR_r, BL_r */
    bool find_red_block(cv::Mat& img, cv::Point2f corners[4]);

    /* 把 minAreaRect 返回的 4 点强制排成 TL,TR,BR,BL */
    void sort_quad_clockwise_from_topleft(const cv::Point2f raw[4], cv::Point2f out[4]);

    /* 八邻域漫水填充用的访问标记；首次或尺寸不匹配时 create()，每帧 setTo(0) 复用 */
    cv::Mat visited_;
};

// ==========================================
// 全局视觉流水线接口
// ==========================================
// ==========================================
// 视觉模块初始化（加载 NCNN 模型、配置线程/精度）
// ==========================================
void vision_init(void);
void process_car_vision(cv::Mat& frame);

/**
 * @brief 将最近一次 vision 裁切的 64×64 ROI 保存为 JPG
 * @return  true 表示写入成功；false 表示 ROI 无效、目录创建失败或 imwrite 失败
 * @sample  KEY_3 按下边沿 → vision_save_last_roi_to_picture();
 * @note    目标目录 /home/root/picture/；无有效 g_last_roi 时不写入并打印警告
 */
bool vision_save_last_roi_to_picture(void);

/** 主循环阶段探针 ID（ENABLE_TERMINAL_DEBUG 时由 main 写入） */
extern volatile sig_atomic_t g_dbg_stage_id;


#endif // VISION_MODULE_HPP