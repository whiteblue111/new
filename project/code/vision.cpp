#include "app_config.h"
#include "image.hpp"
#include "vision.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>
#include <cstdio>
#include <cstring>
#include <sys/time.h>
#include <sys/stat.h>
#include <errno.h>
#include <csignal>

using namespace std;
using namespace cv;

int red_area = 0; // 全局变量，记录当前检测到的红色区域面积
float aspect_ratio = 0.0f; // 全局变量，记录当前检测到

volatile sig_atomic_t g_dbg_stage_id = 0;
bool  g_is_bypassing_binoculars = false;
int   g_bypass_timer            = 0;
const int   BYPASS_MAX_FRAMES   = 80;
const float BYPASS_OFFSET       = 30.0f;

/* ====================== 视觉绕行状态机（供 image.cpp fitting() 读取） ====================== */
VisionBypassAction g_vision_bypass_action = VBA_STRAIGHT;

/* model_roi_cut 连续失败帧计数；超过 VISION_LOST_FRAMES 视为红块离开视野 */
static int s_no_roi_frames = 0;

/* 3 帧确认机制状态（提到文件级，便于丢失红块时统一复位） */
static int s_confirmed_index = -2; /* 已生效的"官方结果"，-2 表示未初始化 */
static int s_candidate_index = -2; /* 当前考察期的"候选结果" */
static int s_consecutive_cnt = 0;  /* 候选结果连续出现帧数 */

/* NCNN 推理冷却截止时间（毫秒时间戳）；0 表示未在冷却 */
static long long s_infer_cooldown_until_ms = 0;

/**
 * @brief 获取当前单调毫秒时间戳
 * @return 自 epoch 起的毫秒数
 * @sample const long long now = vision_now_ms();
 */
static long long vision_now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
}

/**
 * @brief 判断 NCNN 推理是否处于冷却期
 * @return true 表示冷却中，应跳过 ex.input/extract；false 表示可推理
 * @sample if (vision_infer_in_cooldown()) return;
 */
static bool vision_infer_in_cooldown(void)
{
    return s_infer_cooldown_until_ms > 0 && vision_now_ms() < s_infer_cooldown_until_ms;
}

/**
 * @brief 启动 NCNN 推理冷却（连续 3 帧确认后调用）
 * @return 无
 * @sample vision_infer_start_cooldown();
 * @note  冷却期间仍执行 probe_red_hint 与 model_roi_cut，仅跳过 NCNN
 */
static void vision_infer_start_cooldown(void)
{
    s_infer_cooldown_until_ms = vision_now_ms() + VISION_INFER_COOLDOWN_MS;
    std::cout << "[智能视觉] 推理冷却 "
              << (VISION_INFER_COOLDOWN_MS / 1000) << "s" << std::endl;
}

/**
 * @brief 把 NCNN 6 类标签索引映射为中线偏移动作
 * @param idx 标签索引（0=Ambulance 1=Armored 2=Binoculars 3=Grenade 4=Guns 5=medical）
 * @return 对应 VisionBypassAction；未知索引按 VBA_STRAIGHT 处理
 * @sample g_vision_bypass_action = map_label_to_action(candidate_index);
 * @note  炸药/枪支 → 左绕行；望远镜/医疗箱 → 右绕行；救护车/装甲车 → 直行
 */
static VisionBypassAction map_label_to_action(int idx) {
    switch (idx) {
        case 3: case 4: return VBA_LEFT;   /* Grenade / Guns */
        case 2: case 5: return VBA_RIGHT;  /* Binoculars / medical */
        case 0: case 1: default: return VBA_STRAIGHT; /* Ambulance / Armored vehicle */
    }
}

// 实例化一个本文件内部使用的检测器
static RedRectDetector my_detector;

int block_w = 0; // 全局变量，记录当前检测到的红色块宽度
int block_h = 0; // 全局变量，记录当前检测到的红色

/* ====================== 视觉状态缓存（供 display_show_vision 读取） ====================== */
cv::Mat g_last_roi;          /* 最近一次 model_roi_cut 成功输出的 64×64 BGR */
cv::Mat g_last_saved_roi;    /* KEY_3 最近一次保存成功的 64×64 ROI 深拷贝 */
int     g_last_pred_index = -1;
float   g_last_pred_prob  = 0.0f;

/* model_roi_cut 拒因：-1=未运行, 0=OK, 1=find_red_block fail, 2=block_w/h 超阈值, 3=梯形越界 */
int g_roi_cut_reject_reason = -1;

VisionSnapshotResult g_vision_snapshot_result = VSNAP_NONE;
int                  g_vision_snapshot_ttl      = 0;

static constexpr int VISION_SNAPSHOT_TTL_FRAMES = 45;

static const std::vector<std::string> kVisionLabels = {
    "Ambulance",
    "Armored vehicle",
    "Binoculars",
    "Grenade",
    "Guns",
    "medical"
};

/**
 * @brief 取 6 类标签数组
 * @return 标签数组常量引用
 * @sample auto& labels = vision_labels(); printf("%s", labels[i].c_str());
 */
const std::vector<std::string>& vision_labels(void) { return kVisionLabels; }

static constexpr int VISION_SCAN_FALLBACK_X = 45;

/**
 * @brief 取指定行的红块扫描列范围
 * @param img   BGR ROI
 * @param y     行号
 * @param x_lo  [out] 含左端列
 * @param x_hi  [out] 不含右端列
 * @return 无
 * @note  边界无效时回退 45 .. cols-45
 */
static void vision_row_scan_xrange(const Mat &img, int y, int &x_lo, int &x_hi)
{
    if (track_row_bounds_enabled() && track_row_bounds_xrange(y, x_lo, x_hi))
        return;
    x_lo = VISION_SCAN_FALLBACK_X;
    x_hi = img.cols - VISION_SCAN_FALLBACK_X;
    if (x_hi <= x_lo)
    {
        x_lo = 0;
        x_hi = img.cols;
    }
}

/**
 * @brief 判断像素是否落在当行赛道边界内
 * @param img BGR ROI
 * @param x   列号
 * @param y   行号
 * @return true 在允许范围内或边界未启用时的回退范围内
 */
static bool vision_point_in_row_bounds(const Mat &img, int x, int y)
{
    int x_lo = 0;
    int x_hi = 0;
    vision_row_scan_xrange(img, y, x_lo, x_hi);
    return x >= x_lo && x < x_hi;
}

/**
 * @brief 检查四边形四角是否均在各行赛道边界内
 * @param img  BGR ROI
 * @param quad 4 角点
 * @return true 全部在界内或边界未启用
 */
static bool vision_quad_within_track_bounds(const Mat &img, const Point2f quad[4])
{
    if (!track_row_bounds_enabled())
        return true;
    for (int i = 0; i < 4; i++)
    {
        const int y = (int)(quad[i].y + 0.5f);
        const int x = (int)(quad[i].x + 0.5f);
        if (y < 0 || y >= img.rows)
            return false;
        int x_lo = 0;
        int x_hi = 0;
        if (!track_row_bounds_xrange(y, x_lo, x_hi))
            return false;
        if (x < x_lo || x >= x_hi)
            return false;
    }
    return true;
}

// ==========================================
// RedRectDetector 类的方法实现
// ==========================================

/**
 * @brief 把 minAreaRect 返回的 4 个角点按 TL,TR,BR,BL 顺时针顺序输出
 * @param raw  [in]  OpenCV minAreaRect.points() 给出的原始 4 点（顺序不定）
 * @param out  [out] 排好序的 4 点：out[0]=TL out[1]=TR out[2]=BR out[3]=BL
 * @return  无
 * @note 先按 y 升序分成上下两对，上面那对 x 小的是 TL、x 大的是 TR；
 *       下面那对 x 小的是 BL、x 大的是 BR。倾斜 ±45° 内都能正确排出顺序。
 */
void RedRectDetector::sort_quad_clockwise_from_topleft(const Point2f raw[4], Point2f out[4]) {
    Point2f pts[4] = { raw[0], raw[1], raw[2], raw[3] };
    std::sort(pts, pts + 4, [](const Point2f& a, const Point2f& b) {
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    });
    /* 上面两点：pts[0], pts[1] —— 再按 x 分 TL / TR */
    if (pts[0].x <= pts[1].x) { out[0] = pts[0]; out[1] = pts[1]; }
    else                       { out[0] = pts[1]; out[1] = pts[0]; }
    /* 下面两点：pts[2], pts[3] —— 再按 x 分 BL / BR */
    if (pts[2].x <= pts[3].x) { out[3] = pts[2]; out[2] = pts[3]; }
    else                       { out[3] = pts[3]; out[2] = pts[2]; }
}

/**
 * @brief 在原图中检测红色色块并输出其 4 个倾斜角点
 * @param img      [in]  BGR 裁剪图（通常 320×130）
 * @param corners  [out] 红块 4 角，顺序 TL_r, TR_r, BR_r, BL_r
 * @return true 找到合法红块；false 未找到或点云不足
 * @sample Point2f red[4]; if (find_red_block(frame, red)) { ... }
 * @note 从底行往上逐行检查"是否有 >= MIN_CONSECUTIVE_RED 个连续红像素"来确认
 *       红块底边 confirmed_bottom，再在该行取最长红段中点为种子；从种子做八邻域 BFS，
 *       只在 is_red_bgr 像素上扩散，并用 max_h_cap 限制离种子最大高度，
 *       避免与上方急救包红十字等红色物体粘连时被吃进去。
 *       BFS 完成后验证：高度 < 5 行或高度 > 宽度视为不合格，跳过并继续往上找。
 *       漫水得到的点云再喂 cv::minAreaRect 拟合旋转矩形，输出 4 角。
 *       320×130 分辨率下红块本体仅数百像素，单帧 < 1ms。
 */
bool RedRectDetector::find_red_block(Mat& img, Point2f corners[4]) {
    const int scan_y_start = 40;
    const int MIN_CONSECUTIVE_RED = 5;

    if (visited_.empty() || visited_.size() != img.size() || visited_.type() != CV_8U) {
        visited_.create(img.size(), CV_8U);
    }
    visited_.setTo(0);

    static const int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    int search_from = img.rows - 1; /* 每轮从此行开始往上找下一个 confirmed_bottom */

    while (search_from >= scan_y_start) {
        /* 1) 从 search_from 往上找 confirmed_bottom */
        int confirmed_bottom = -1;
        int red_row_count    = 0;
        for (int y = search_from; y >= scan_y_start; y--) {
            if (visited_.at<uchar>(y, img.cols / 2)) { red_row_count = 0; continue; }
            int scan_x_start = 0;
            int scan_x_end   = 0;
            vision_row_scan_xrange(img, y, scan_x_start, scan_x_end);
            Vec3b* ptr = img.ptr<Vec3b>(y);
            bool has_run = false;
            int run = 0;
            for (int x = scan_x_start; x < scan_x_end; x++) {
                if (is_red_bgr(ptr[x])) {
                    if (++run >= MIN_CONSECUTIVE_RED) { has_run = true; break; }
                } else { run = 0; }
            }
            if (has_run) {
                red_row_count++;
                if (red_row_count >= RED_CONFIRM_ROWS) {
                    confirmed_bottom = y + RED_CONFIRM_ROWS - 1;
                    break;
                }
            } else { red_row_count = 0; }
        }
        if (confirmed_bottom < 0) return false;

        /* 2) 在 confirmed_bottom 向上 SEED_SEARCH_ROWS 行内找全局最长红色连续段，
         *    取中点作 BFS 种子。倾斜红块时底行只有底角宽度，需要看几行才能拿到接近全宽。 */
        const int SEED_SEARCH_ROWS = 10;
        int best_lx = -1, best_rx = -1, best_w = 0;
        int best_seed_y = confirmed_bottom;
        const int seed_scan_top = std::max(scan_y_start, confirmed_bottom - SEED_SEARCH_ROWS + 1);
        for (int y = confirmed_bottom; y >= seed_scan_top; y--) {
            int scan_x_start = 0;
            int scan_x_end   = 0;
            vision_row_scan_xrange(img, y, scan_x_start, scan_x_end);
            Vec3b* row_ptr = img.ptr<Vec3b>(y);
            int cur_lx = -1;
            for (int x = scan_x_start; x < scan_x_end; x++) {
                if (is_red_bgr(row_ptr[x])) {
                    if (cur_lx < 0) cur_lx = x;
                } else if (cur_lx >= 0) {
                    int w = x - cur_lx;
                    if (w > best_w) { best_w = w; best_lx = cur_lx; best_rx = x - 1; best_seed_y = y; }
                    cur_lx = -1;
                }
            }
            if (cur_lx >= 0) {
                int w = scan_x_end - cur_lx;
                if (w > best_w) { best_w = w; best_lx = cur_lx; best_rx = scan_x_end - 1; best_seed_y = y; }
            }
        }
        if (best_w < 8) { search_from = confirmed_bottom - 1; continue; }

        const Point seed((best_lx + best_rx) / 2, best_seed_y);
        if (!vision_point_in_row_bounds(img, seed.x, seed.y))
        {
            search_from = confirmed_bottom - 1;
            continue;
        }
        const int base_w    = best_w;
        const int max_h_cap = std::max(int(base_w * 2.5f), 25); /* 下限 25 兜底，倾斜大块也能爬到顶 */

        /* 3) 八邻域 BFS */
        std::vector<Point2f> blob;
        blob.reserve(2048);
        std::vector<Point> stk;
        stk.reserve(2048);

        stk.push_back(seed);
        visited_.at<uchar>(seed.y, seed.x) = 1;

        int blob_min_y = seed.y, blob_max_y = seed.y;
        int blob_min_x = seed.x, blob_max_x = seed.x;

        while (!stk.empty()) {
            Point p = stk.back();
            stk.pop_back();
            blob.emplace_back((float)p.x, (float)p.y);
            if (p.y < blob_min_y) blob_min_y = p.y;
            if (p.y > blob_max_y) blob_max_y = p.y;
            if (p.x < blob_min_x) blob_min_x = p.x;
            if (p.x > blob_max_x) blob_max_x = p.x;
            for (int k = 0; k < 8; k++) {
                int nx = p.x + dx8[k];
                int ny = p.y + dy8[k];
                if (nx < 0 || nx >= img.cols || ny < 0 || ny >= img.rows) continue;
                if (seed.y - ny > max_h_cap) continue;
                if (!vision_point_in_row_bounds(img, nx, ny)) continue;
                uchar& v = visited_.at<uchar>(ny, nx);
                if (v) continue;
                if (!is_red_bgr(img.at<Vec3b>(ny, nx))) continue;
                v = 1;
                stk.push_back(Point(nx, ny));
            }
        }

        const int blob_h = blob_max_y - blob_min_y + 1;
        const int blob_w = blob_max_x - blob_min_x + 1;

        /* 4) 合法性验证：点数不足 / 高度 < 5 行 / 高度 > 宽度 → 不合格，继续往上找 */
        if (blob.size() < 40 || blob_h < 5 || blob_h > 1.5*blob_w) {
            search_from = blob_min_y - 1;
            continue;
        }

        /* 5) minAreaRect → 4 角点；再排成 TL,TR,BR,BL */
        RotatedRect rr = minAreaRect(blob);
        Point2f raw[4];
        rr.points(raw);
        sort_quad_clockwise_from_topleft(raw, corners);
        return true;
    }
    return false;
}

/**
 * @brief 轻量红块预检：自底向上行扫描，连续 RED_CONFIRM_ROWS 行有红色连续段则通过
 * @param img [in] BGR 裁剪图
 * @return true 可能存在红块；false 可跳过 model_roi_cut 与 NCNN
 * @note  无 visited_/BFS/透视，直道无红块时省算力；有效红块仍以 model_roi_cut 为准
 */
bool RedRectDetector::probe_red_hint(const Mat& img) const
{
    const int scan_y_start = 40;
    const int MIN_CONSECUTIVE_RED = 5;

    int red_row_count = 0;
    for (int y = img.rows - 1; y >= scan_y_start; y--) {
        int scan_x_start = 0;
        int scan_x_end   = 0;
        vision_row_scan_xrange(img, y, scan_x_start, scan_x_end);
        const Vec3b* ptr = img.ptr<Vec3b>(y);
        bool has_run = false;
        int run = 0;
        for (int x = scan_x_start; x < scan_x_end; x++) {
            if (is_red_bgr(ptr[x])) {
                if (++run >= MIN_CONSECUTIVE_RED) { has_run = true; break; }
            } else {
                run = 0;
            }
        }
        if (has_run) {
            red_row_count++;
            if (red_row_count >= RED_CONFIRM_ROWS)
                return true;
        } else {
            red_row_count = 0;
        }
    }
    return false;
}

/**
 * @brief 以红块 4 角为基准，沿碑体斜边构造图片梯形并透视裁剪为 64×64 ROI
 * @param img      [in,out] BGR 原图；is_draw=true 时会在图上绘制梯形
 * @param roi      [out]    透视摆正后的 64×64 BGR 图（喂给 NCNN）
 * @param is_draw  [in]     是否在 img 上绘制图片梯形（绿）与红块斜矩形（红）
 * @return true 裁剪成功；false 未找到红块或梯形越界或红块尺寸不合规
 * @sample Mat roi; if (det.model_roi_cut(frame, roi, true)) ncnn_infer(roi);
 * @note 走马观碑专用。图片宽 ≈ block_w，高 = PIC_H_RATIO × block_h（默认 3×），
 *       沿红块左右两条斜边方向向上延伸而不是垂直于顶边，从而跟随透视；
 *       getPerspectiveTransform + warpPerspective 一步完成「裁 + 摆正 + 缩放」，
 *       省一次 Mat 拷贝。针对 320×240 分辨率调参。
 */
bool RedRectDetector::model_roi_cut(Mat& img, Mat& roi, bool is_draw) {
    Point2f red[4]; /* TL_r, TR_r, BR_r, BL_r */
    if (!find_red_block(img, red)) { g_roi_cut_reject_reason = 1; return false; }

    /* 用斜边长度刷新老的 block_w/block_h 全局，含义更对 */
    block_w = (int)cv::norm(red[1] - red[0]); /* |TR_r - TL_r| 顶边长 */
    block_h = (int)cv::norm(red[3] - red[0]); /* |BL_r - TL_r| 左侧斜边长 */
    if (block_w < 15 || block_h < 5 || block_w > 55 || block_h > 30) {
        g_roi_cut_reject_reason = 2;
        return false;
    }

    /* 沿左右两条斜边向上延伸 PIC_H_RATIO 倍，构造图片梯形 */
    picture_quad[0] = red[0] + PIC_H_RATIO * (red[0] - red[3]); /* pic TL = TL_r + 3×(TL_r-BL_r) */
    picture_quad[1] = red[1] + PIC_H_RATIO * (red[1] - red[2]); /* pic TR = TR_r + 3×(TR_r-BR_r) */
    picture_quad[2] = red[1];                                    /* pic BR = TR_r */
    picture_quad[3] = red[0];                                    /* pic BL = TL_r */

    /* PIC_W_RATIO ≠ 1 时沿顶边方向左右扩展（保留以后调参余地） */
    if (std::fabs(PIC_W_RATIO - 1.0f) > 1e-3f) {
        const float k = (PIC_W_RATIO - 1.0f) * 0.5f;
        Point2f top_dir = picture_quad[1] - picture_quad[0]; /* pic TL → pic TR */
        Point2f bot_dir = picture_quad[2] - picture_quad[3]; /* pic BL → pic BR */
        picture_quad[0] -= k * top_dir;
        picture_quad[1] += k * top_dir;
        picture_quad[3] -= k * bot_dir;
        picture_quad[2] += k * bot_dir;
    }

    /* 任一角越界就放弃这一帧（车离碑太近 / 红块贴边 / 超出赛道行边界） */
    for (int i = 0; i < 4; i++) {
        if (picture_quad[i].x < 0 || picture_quad[i].x >= img.cols ||
            picture_quad[i].y < 0 || picture_quad[i].y >= img.rows) {
            g_roi_cut_reject_reason = 3;
            return false;
        }
    }
    if (!vision_quad_within_track_bounds(img, red) ||
        !vision_quad_within_track_bounds(img, picture_quad)) {
        g_roi_cut_reject_reason = 3;
        return false;
    }

    /* 一步到位：透视去畸变 + 缩放到 MODEL_INPUT_WIDTH × MODEL_INPUT_WIDTH */
    const float S = (float)MODEL_INPUT_WIDTH;
    Point2f dst[4] = { {0.0f, 0.0f}, {S - 1.0f, 0.0f}, {S - 1.0f, S - 1.0f}, {0.0f, S - 1.0f} };
    Mat M = getPerspectiveTransform(picture_quad, dst);
    warpPerspective(img, roi, M, Size(MODEL_INPUT_WIDTH, MODEL_INPUT_WIDTH),
                    INTER_LINEAR, BORDER_REPLICATE);

    /* 外接 bbox 留给 target_rect（兼容 HUD/外部读取） */
    std::vector<Point2f> quad_vec(picture_quad, picture_quad + 4);
    target_rect = boundingRect(quad_vec) & Rect(0, 0, img.cols, img.rows);

    if (is_draw) {
        /* 绿：图片梯形（替代原来的轴对齐绿框） */
        std::vector<Point> pic_i;
        pic_i.reserve(4);
        for (int i = 0; i < 4; i++) pic_i.emplace_back((int)picture_quad[i].x, (int)picture_quad[i].y);
        const Point* pic_pts = pic_i.data();
        int pic_npts = 4;
        polylines(img, &pic_pts, &pic_npts, 1, true, Scalar(0, 255, 0), 2);

        /* 红：红块斜矩形（替代原来的轴对齐红框） */
        std::vector<Point> red_i;
        red_i.reserve(4);
        for (int i = 0; i < 4; i++) red_i.emplace_back((int)red[i].x, (int)red[i].y);
        const Point* red_pts = red_i.data();
        int red_npts = 4;
        polylines(img, &red_pts, &red_npts, 1, true, Scalar(0, 0, 255), 2);
    }
    g_roi_cut_reject_reason = 0;
    return true;
}
/**
 * @brief 视觉模块初始化：加载 NCNN 分类模型并配置推理选项
 * @param 无
 * @return 无（加载失败 NCNN 内部 printf 报错，对象保持空状态）
 * @sample vision_init();
 * @note LS2K0300 无 FP16 加速，强制 FP32；LS2K0300 单核 LA264，单线程避免与主线程争抢
 */
 void vision_init(void)
 {
     printf("[vision] 正在加载 NCNN 模型...\n");
     my_net.opt.num_threads        = 1;
     my_net.opt.use_fp16_arithmetic = false;
     my_net.opt.use_fp16_storage    = false;
     my_net.load_param("tiny_classifier_fp32.ncnn.param");
     my_net.load_model("tiny_classifier_fp32.ncnn.bin");
     printf("[vision] NCNN 模型加载完毕！\n");
 }

#define VISION_PICTURE_DIR "/home/root/picture/"

/**
 * @brief 更新 KEY_3 拍照屏显状态
 * @param r  拍照结果枚举
 * @return 无
 * @note  TTL 在 display_show_vision 中递减
 */
static void vision_snapshot_notify(VisionSnapshotResult r)
{
    g_vision_snapshot_result = r;
    g_vision_snapshot_ttl    = VISION_SNAPSHOT_TTL_FRAMES;
}

/**
 * @brief 确保 picture 目录存在
 * @return  true 表示目录可用
 */
static bool vision_ensure_picture_dir(void)
{
    struct stat st;
    if (stat(VISION_PICTURE_DIR, &st) == 0)
    {
        if (S_ISDIR(st.st_mode))
            return true;
        printf("[vision] %s 存在但不是目录\n", VISION_PICTURE_DIR);
        return false;
    }
    if (mkdir(VISION_PICTURE_DIR, 0755) != 0)
    {
        printf("[vision] mkdir %s 失败: %s\n", VISION_PICTURE_DIR, strerror(errno));
        return false;
    }
    return true;
}

/**
 * @brief 将最近一次 vision 裁切的 64×64 ROI 保存为 JPG
 * @return  true 表示写入成功；false 表示 ROI 无效、目录创建失败或 imwrite 失败
 * @sample  KEY_3 按下边沿 → vision_save_last_roi_to_picture();
 * @note    文件名 pic_序号_时间戳.jpg，写入 /home/root/picture/
 */
bool vision_save_last_roi_to_picture(void)
{
    if (g_last_roi.empty()
        || g_last_roi.cols != MODEL_INPUT_WIDTH
        || g_last_roi.rows != MODEL_INPUT_WIDTH
        || g_last_roi.channels() != 3)
    {
        printf("[vision] 无有效 64x64 ROI，未保存\n");
        vision_snapshot_notify(VSNAP_FAIL_NO_ROI);
        return false;
    }
    if (!vision_ensure_picture_dir())
    {
        vision_snapshot_notify(VSNAP_FAIL_DIR);
        return false;
    }

    static unsigned s_pic_seq = 0;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    const long long ts_sec = (long long)tv.tv_sec;

    char path[256];
    snprintf(path, sizeof(path), VISION_PICTURE_DIR "pic_%04u_%lld.jpg",
             s_pic_seq++, (long long)ts_sec);

    if (!cv::imwrite(path, g_last_roi))
    {
        printf("[vision] imwrite 失败: %s\n", path);
        vision_snapshot_notify(VSNAP_FAIL_IO);
        return false;
    }
    g_last_roi.copyTo(g_last_saved_roi);
    printf("[vision] ROI 已保存: %s\n", path);
    vision_snapshot_notify(VSNAP_OK);
    return true;
}

/**
 * @brief 红块/ROI 丢失时递增计数，超 VISION_LOST_FRAMES 后清绕行动作与 3 帧确认状态
 * @return 无
 */
static void vision_handle_no_roi(void)
{
    if (s_no_roi_frames <= VISION_LOST_FRAMES)
        ++s_no_roi_frames;
    if (s_no_roi_frames > VISION_LOST_FRAMES) {
        s_infer_cooldown_until_ms = 0;
        if (g_vision_bypass_action != VBA_STRAIGHT) {
            std::cout << "[智能视觉] 红块离开视野，退出绕行" << std::endl;
            g_vision_bypass_action = VBA_STRAIGHT;
            g_is_bypassing_binoculars = false;
            s_confirmed_index = -2;
            s_candidate_index = -2;
            s_consecutive_cnt = 0;
        }
    }
}

// =============================================================================
// 流水线主函数实现
// =============================================================================
void process_car_vision(cv::Mat& frame) {
    g_dbg_stage_id = 31;
    // #region agent log
    static int s_dbg_vision_entry_cnt = 0;
    if (s_dbg_vision_entry_cnt < 8) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
        FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
        if (debug_log) {
            fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H5\",\"location\":\"vision.cpp:process_car_vision:entry\",\"message\":\"vision entry frame info\",\"data\":{\"rows\":%d,\"cols\":%d,\"channels\":%d,\"empty\":%d},\"timestamp\":%lld}\n", frame.rows, frame.cols, frame.channels(), frame.empty() ? 1 : 0, ts_ms);
            fclose(debug_log);
        }
        s_dbg_vision_entry_cnt++;
    }
    // #endregion

    if (!my_detector.probe_red_hint(frame)) {
        vision_handle_no_roi();
        g_dbg_stage_id = 42;
        return;
    }

    cv::Mat roi;
    
    const bool has_roi = my_detector.model_roi_cut(frame, roi, true);
    g_dbg_stage_id = 32;
    // #region agent log
    static int s_dbg_vision_roi_cnt = 0;
    if (s_dbg_vision_roi_cnt < 8) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
        FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
        if (debug_log) {
            fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H1\",\"location\":\"vision.cpp:process_car_vision:roi\",\"message\":\"roi cut result\",\"data\":{\"has_roi\":%d,\"roi_rows\":%d,\"roi_cols\":%d,\"block_w\":%d,\"block_h\":%d},\"timestamp\":%lld}\n", has_roi ? 1 : 0, roi.rows, roi.cols, block_w, block_h, ts_ms);
            fclose(debug_log);
        }
        s_dbg_vision_roi_cnt++;
    }
    // #endregion

    /* 红块视野维护：连续 VISION_LOST_FRAMES 帧未拿到 ROI 视为红块离开 → 清绕行动作 */
    if (has_roi) {
        s_no_roi_frames = 0;
    } else {
        vision_handle_no_roi();
    }

    if (has_roi) {
        g_dbg_stage_id = 33;
        /* 缓存最近一次 ROI，供 display_show_vision 显示（深拷贝避免下一帧覆盖） */
        roi.copyTo(g_last_roi);

        if (vision_infer_in_cooldown()) {
            g_dbg_stage_id = 38;
            return;
        }

        // ✨ [新增] 调试抓拍逻辑开始
        // static int snapshot_cnt = 0; // 静态计数器
        // if (snapshot_cnt < 5) {
        //     // 生成文件名，比如 debug_crop_0.jpg
        //     std::string filename = "debug_crop_" + std::to_string(snapshot_cnt) + ".jpg";
            
        //     // 将 64x64 的 roi 保存到本地
        //     cv::imwrite(filename, roi);
            
        //     // 把画了红框、绿框的原图也保存下来，方便对比！
        //     std::string full_filename = "debug_full_" + std::to_string(snapshot_cnt) + ".jpg";
        //     cv::imwrite(full_filename, frame);
            
        //     printf("[调试] 抓拍成功！已保存: %s 和 %s\n", filename.c_str(), full_filename.c_str());
        //     snapshot_cnt++;
        // }
        // // ✨ 调试抓拍逻辑结束
        
        ncnn::Mat in = ncnn::Mat::from_pixels(roi.data, ncnn::Mat::PIXEL_BGR2RGB, roi.cols, roi.rows);
        
        const float mean_vals[3] = {123.675f, 116.28f, 103.53f};
        const float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
        in.substract_mean_normalize(mean_vals, norm_vals);

        ncnn::Extractor ex = my_net.create_extractor();
        const int in_ret = ex.input("in0", in); 
        ncnn::Mat out;
        const int out_ret = ex.extract("out0", out);
        g_dbg_stage_id = 34;
        // #region agent log
        static int s_dbg_vision_ncnn_cnt = 0;
        if (s_dbg_vision_ncnn_cnt < 8) {
            struct timeval tv;
            gettimeofday(&tv, nullptr);
            const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
            FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
            if (debug_log) {
                fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H1\",\"location\":\"vision.cpp:process_car_vision:ncnn\",\"message\":\"ncnn io status\",\"data\":{\"in_ret\":%d,\"out_ret\":%d,\"out_w\":%d,\"out_h\":%d,\"out_c\":%d},\"timestamp\":%lld}\n", in_ret, out_ret, out.w, out.h, out.c, ts_ms);
                fclose(debug_log);
            }
            s_dbg_vision_ncnn_cnt++;
        }
        // #endregion

        // 前面的 NCNN 提取结果代码保持不变...
        float max_prob = -100.0f;
        int max_index = -1;
        for (int i = 0; i < out.w; i++) {
            if (out[i] > max_prob) {
                max_prob = out[i];
                max_index = i;
            }
        }
        g_dbg_stage_id = 35;
        // #region agent log
        static int s_dbg_vision_pred_cnt = 0;
        if (s_dbg_vision_pred_cnt < 8) {
            struct timeval tv;
            gettimeofday(&tv, nullptr);
            const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
            FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
            if (debug_log) {
                fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H4\",\"location\":\"vision.cpp:process_car_vision:pred\",\"message\":\"prediction result before debounce\",\"data\":{\"max_index\":%d,\"max_prob\":%.6f,\"out_w\":%d},\"timestamp\":%lld}\n", max_index, max_prob, out.w, ts_ms);
                fclose(debug_log);
            }
            s_dbg_vision_pred_cnt++;
        }
        // #endregion

        /* 3 帧确认机制（状态已提到文件级 static，便于丢失红块时统一复位） */
        int& confirmed_index = s_confirmed_index;
        int& candidate_index = s_candidate_index;
        int& consecutive_cnt = s_consecutive_cnt;

        // 1. 考察候选结果
        if (max_index == candidate_index) {
            // 如果这一帧的结果和上一帧一样，计数器加 1
            consecutive_cnt++;
        } else {
            // 如果出现了一个新的结果，打断施法，重新开始计数
            candidate_index = max_index;
            consecutive_cnt = 1;
        }

        // 2. 判断候选结果是否熬过“考察期”（连续 3 帧）
        if (consecutive_cnt >= 3) {
            
            // 3. 如果这个新确认的结果，和我们之前一直在执行的“官方结果”不一样，才触发动作和打印
            if (candidate_index != confirmed_index) {
                g_dbg_stage_id = 36;

                /* 更新视觉状态缓存（display_show_vision 会读这两个变量） */
                g_last_pred_index = candidate_index;
                g_last_pred_prob  = max_prob;

                const std::vector<std::string>& labels = kVisionLabels;
                // #region agent log
                static int s_dbg_vision_label_cnt = 0;
                if (s_dbg_vision_label_cnt < 8) {
                    struct timeval tv;
                    gettimeofday(&tv, nullptr);
                    const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
                    FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
                    if (debug_log) {
                        fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H4\",\"location\":\"vision.cpp:process_car_vision:label\",\"message\":\"before labels index access\",\"data\":{\"candidate_index\":%d,\"labels_size\":%zu,\"confirmed_index\":%d,\"consecutive_cnt\":%d},\"timestamp\":%lld}\n", candidate_index, labels.size(), confirmed_index, consecutive_cnt, ts_ms);
                        fclose(debug_log);
                    }
                    s_dbg_vision_label_cnt++;
                }
                // #endregion
                
                // ... 前面的代码不变 ...
                if (candidate_index >= 0) {
                    g_dbg_stage_id = 37;
                    // #region agent log
                    if ((candidate_index >= (int)labels.size()) || (candidate_index < 0)) {
                        fprintf(stderr, "[termdbg] vision invalid label index=%d labels_size=%zu\n", candidate_index, labels.size());
                    }
                    // #endregion
                    std::cout << "[智能视觉] 连续 3 帧确认目标: " << labels[candidate_index] << std::endl;

#if ENABLE_VISION_BYPASS
                    /* 类别 → 中线偏移动作映射（炸药/枪支 左绕；望远镜/医疗箱 右绕；救护车/装甲车 直行） */
                    const VisionBypassAction new_action = map_label_to_action(candidate_index);
                    g_vision_bypass_action = new_action;
                    /* 维持老 API 兼容：望远镜场景下保留旗标供 display HUD 使用 */
                    g_is_bypassing_binoculars = (new_action != VBA_STRAIGHT);
                    g_bypass_timer = 0;
                    switch (new_action) {
                        case VBA_LEFT:
                            std::cout << "[智能视觉] 动作 → 左绕行（左线 +"
                                      << VISION_BYPASS_SHIFT_PX << "px 作为中线）" << std::endl;
                            break;
                        case VBA_RIGHT:
                            std::cout << "[智能视觉] 动作 → 右绕行（右线 -"
                                      << VISION_BYPASS_SHIFT_PX << "px 作为中线）" << std::endl;
                            break;
                        case VBA_STRAIGHT:
                        default:
                            std::cout << "[智能视觉] 动作 → 直行（不偏移中线）" << std::endl;
                            break;
                    }
#else
                    std::cout << "[智能视觉] 已识别: " << labels[candidate_index]
                              << "（绕行关闭，不改中线）" << std::endl;
#endif
                }
                 else {
                    std::cout << "[智能视觉] 确认目标已丢失" << std::endl;
                }

                // 4. 将候选结果转正，更新为官方结果
                confirmed_index = candidate_index;
                g_dbg_stage_id = 41;
            }

            vision_infer_start_cooldown();
            s_consecutive_cnt = 0;
            s_candidate_index = -2;
        }
    }
    g_dbg_stage_id = 42;
}