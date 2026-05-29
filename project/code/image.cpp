/**
 * @file image.cpp
 * @brief 巡线主流水线实现（移植自 temp_repo/track/standard/standard.cpp）
 *
 * 完整流水线（image_process）：
 *   ImageProcess::processImage(rgb_img) → bin_img (OTSU + 闭运算)
 *   trackRecognition(gray, bin)         → floodFill + 最长白列 + 迷宫法 + 透视
 *                                          + blur + resample + 角度 + NMS + 找拐点
 *   Cross_Check / Cross_Run            → 十字状态机
 *   Ring_Check  / Ring_Run             → 环岛状态机
 *   fitting() → normalizeCenterEdge → CenterEdge
 *   Image_Error_Get() → 中线像素偏差 → img_err
 */

#include "app_config.h"
#include "image.hpp"
#include "motor.hpp"
#include "math.hpp"
#include "redbrick.hpp"
#include "vision.hpp"
#include "banmaxian.hpp"
#include "bird_lut.hpp"
#include "findline_core.hpp"
#include "perf_stats.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <time.h>
extern RedBlockAvoider g_brick_avoider;



/* ====================== 全局图像 ====================== */
cv::Mat rgb_img;
cv::Mat rgb_cut_img;
cv::Mat gray_img;
cv::Mat gray_cut_img;
cv::Mat gray_bird_img;
cv::Mat bin_img;
cv::Mat bin_bird_img;
cv::Mat imgShow;


/* ====================== 全局点集 ====================== */
std::vector<POINT> pointsEdgeLeft;
std::vector<POINT> pointsEdgeRight;
std::vector<POINT> t_pointsEdgeLeft;
std::vector<POINT> t_pointsEdgeRight;
std::vector<POINT> b_t_pointsEdgeLeft;
std::vector<POINT> b_t_pointsEdgeRight;
std::vector<POINT> s_b_t_pointsEdgeLeft;
std::vector<POINT> s_b_t_pointsEdgeRight;
std::vector<POINT> a_t_pointsEdgeLeft;
std::vector<POINT> a_t_pointsEdgeRight;
std::vector<POINT> n_a_t_pointsEdgeLeft;
std::vector<POINT> n_a_t_pointsEdgeRight;
std::vector<POINT> t_left_CenterEdge;
std::vector<POINT> t_right_CenterEdge;
std::vector<POINT> t_CenterEdge;
std::vector<POINT> CenterEdge;

int pointsEdgeLeft_size        = 0;
int pointsEdgeRight_size       = 0;
int t_pointsEdgeLeft_size      = 0;
int t_pointsEdgeRight_size     = 0;
int b_t_pointsEdgeLeft_size    = 0;
int b_t_pointsEdgeRight_size   = 0;
int s_b_t_pointsEdgeLeft_size  = 0;
int s_b_t_pointsEdgeRight_size = 0;
int a_t_pointsEdgeLeft_size    = 0;
int a_t_pointsEdgeRight_size   = 0;
int n_a_t_pointsEdgeLeft_size  = 0;
int n_a_t_pointsEdgeRight_size = 0;
int t_left_CenterEdge_size     = 0;
int t_right_CenterEdge_size    = 0;
int t_CenterEdge_size          = 0;
int CenterEdge_size            = 0;

bool      is_t_L_pointLeft_find  = false;
bool      is_t_L_pointRight_find = false;
int       t_L_pointLeft_id       = 0;
int       t_L_pointRight_id      = 0;
cv::Point t_L_pointLeft;
cv::Point t_L_pointRight;

bool is_left_straight  = false;
bool is_right_straight = false;
bool is_left_curve     = false;
bool is_right_curve    = false;

float aim_angle = 0.0f;
int   scene     = (int)Scene::NormalScene;


/* ====================== 静态工作变量（流水线参数） ====================== */
static General        s_general;
static ImageProcess   s_imgproc;
static Cross          s_cross;
static Ring           s_ring;

static const int      block_size       = 9;
static const int      clip_value       = 3;
static const int      thresOTSU        = 128;
static const double   pixel_per_meter  = 88.88;
static const double   SAMPLE_DIST      = 0.02;
static const double   ROAD_WIDTH       = 0.45;
static const int      approx_num       = 3;
static const double   dist_half_road   = pixel_per_meter * ROAD_WIDTH / 2.0;

static cv::Point      seedPoint        = cv::Point(COLSIMAGE / 2, seed_y_roi);
static float          aim_angle_last     = 0.0f;
static float          s_track_cx         = COLSIMAGE / 2.0f;
static float          s_track_cy         = (float)(ROI_H * 0.95);
static bool           s_center_effective = false;
static int            x0_seed = COLSIMAGE / 2;
static int            x1_seed = COLSIMAGE / 2;

static bool           s_track_bufs_inited   = false;
static uint32_t       s_track_frame_idx       = 0;
static bool           s_prev_L_left           = false;
static bool           s_prev_L_right          = false;
static bool           s_straight_hold_l         = false;
static bool           s_straight_hold_r       = false;
static bool           s_center_edge_roi_dirty = true;

#if ENABLE_PERF_IMAGE_STAGES
static uint64_t perf_now_us_local(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}
#endif

#define EDGE_CHECK_DIS  2    /* 跳变两侧连续同色像素数，抑制毛刺 */
#define EDGE_MARGIN     10   /* 与迷宫起点边界 x0v>10 / x1v<WIDTH-10 一致 */

static constexpr int TRACK_BOUND_MIN_WIDTH  = 8;
static constexpr int TRACK_BOUND_VALID_ROWS = 10;

static int16_t g_track_bound_left[ROI_H];
static int16_t g_track_bound_right[ROI_H];
static bool    g_track_row_bounds_valid = false;


/* ====================== 内部函数声明 ====================== */
static bool find_bottom_edge_seed_left(const cv::Mat &img, int y, int scan_from, int margin,
                                       int &out_x);
static bool find_bottom_edge_seed_right(const cv::Mat &img, int y, int scan_from, int margin,
                                        int &out_x);
static int  track_bypass_seed_scan_from(int side, int mid, int margin, int cols);
static void blur_points(int side, int kernel);
static void resample_points(std::vector<POINT> &in, int in_size,
                            std::vector<POINT> &out, int &out_size, float dist);
static void local_angle_points(const std::vector<POINT> &pointsEdgeIn, int size,
                               std::vector<POINT> &pointsEdgeOut, int dist);
static void nms_angle(std::vector<POINT> &in, int in_size,
                      std::vector<POINT> &out, int kernel);
static void centerCompute(const std::vector<POINT> &pointsEdge, int size, int side);
static void track_both_edge();
static void line_straight_detection();
static void find_corners();
static void trackRecognition(cv::Mat &imageBinary);
static void fitting();
static bool normalizeCenterEdge(float &cx, float &cy);
static void recover_bird_edge_if_empty(int side);
static bool track_need_full_postprocess(void);


/**
 * @brief 行数持平时决定补齐右侧（319）还是左侧（0）
 * @param nL 左边界有效行数
 * @param nR 右边界有效行数
 * @return true 表示缺行时补右侧为 COLSIMAGE-1；false 表示补左侧为 0
 * @note  对齐 select_track_state 的 dl/dr 与 aim_angle_last 防抖
 */
static bool track_pick_pad_right_side(int nL, int nR)
{
    const int dl = pointsEdgeLeft_size;
    const int dr = pointsEdgeRight_size;
    if (nL > nR)
        return true;
    if (nR > nL)
        return false;
    if ((dr - dl) > 1)
        return false;
    if ((dl - dr) > 1)
        return true;
    if (dl > dr)
        return true;
    if (dr > dl)
        return false;
    if (dl > 0 && dr > 0)
        return aim_angle_last > 0.f;
    if (dl > 0)
        return true;
    if (dr > 0)
        return false;
    return true;
}


/**
 * @brief 迷宫巡线后由 pointsEdgeLeft/Right 重建每行左右边界
 * @return 无
 * @note  按迷宫步进顺序记录每 y 首个 x；短边缺行补 0 / COLSIMAGE-1
 */
void track_build_row_bounds_from_maze(void)
{
    for (int y = 0; y < ROI_H; y++)
    {
        g_track_bound_left[y]  = -1;
        g_track_bound_right[y] = -1;
    }

    for (int i = 0; i < pointsEdgeLeft_size; i++)
    {
        const int y = pointsEdgeLeft[i].y;
        if (y < 0 || y >= ROI_H)
            continue;
        if (g_track_bound_left[y] < 0)
            g_track_bound_left[y] = (int16_t)pointsEdgeLeft[i].x;
    }
    for (int i = 0; i < pointsEdgeRight_size; i++)
    {
        const int y = pointsEdgeRight[i].y;
        if (y < 0 || y >= ROI_H)
            continue;
        if (g_track_bound_right[y] < 0)
            g_track_bound_right[y] = (int16_t)pointsEdgeRight[i].x;
    }

    int nL = 0;
    int nR = 0;
    for (int y = 0; y < ROI_H; y++)
    {
        if (g_track_bound_left[y] >= 0)
            nL++;
        if (g_track_bound_right[y] >= 0)
            nR++;
    }

    const bool pad_right = track_pick_pad_right_side(nL, nR);
    const int  x_max     = COLSIMAGE - 1;
    if (pad_right)
    {
        for (int y = 0; y < ROI_H; y++)
        {
            if (g_track_bound_right[y] < 0)
                g_track_bound_right[y] = (int16_t)x_max;
        }
    }
    else
    {
        for (int y = 0; y < ROI_H; y++)
        {
            if (g_track_bound_left[y] < 0)
                g_track_bound_left[y] = 0;
        }
    }

    int valid_rows = 0;
    for (int y = 0; y < ROI_H; y++)
    {
        int l = (int)g_track_bound_left[y];
        int r = (int)g_track_bound_right[y];
        if (l < 0 || r < 0)
            continue;
        if (l >= r)
        {
            r = std::min(x_max, l + TRACK_BOUND_MIN_WIDTH);
            g_track_bound_right[y] = (int16_t)r;
        }
        if (r - l < TRACK_BOUND_MIN_WIDTH)
        {
            g_track_bound_left[y]  = -1;
            g_track_bound_right[y] = -1;
            continue;
        }
        valid_rows++;
    }
    g_track_row_bounds_valid = (valid_rows >= TRACK_BOUND_VALID_ROWS);
}


/**
 * @brief 本帧行边界是否可用于约束视觉
 * @return true 表示有效行数达到阈值
 * @note  仅 NCNN 视觉模块使用，红砖避障勿调用
 */
bool track_row_bounds_enabled(void)
{
    return g_track_row_bounds_valid;
}


/**
 * @brief 查询指定 ROI 行的允许列范围（已含 TRACK_BOUND_MARGIN）
 * @param y     ROI 局部行号
 * @param x_lo  [out] 含左端列号
 * @param x_hi  [out] 不含右端列号
 * @return      该行边界有效为 true
 * @note  仅 NCNN 视觉模块使用，红砖避障勿调用
 */
bool track_row_bounds_xrange(int y, int &x_lo, int &x_hi)
{
    if (!g_track_row_bounds_valid || y < 0 || y >= ROI_H)
        return false;

    const int l = (int)g_track_bound_left[y];
    const int r = (int)g_track_bound_right[y];
    if (l < 0 || r < 0)
        return false;

    x_lo = l + TRACK_BOUND_MARGIN;
    x_hi = r - TRACK_BOUND_MARGIN;
    if (x_hi <= x_lo)
        return false;
    if (x_lo < 0)
        x_lo = 0;
    if (x_hi > COLSIMAGE - 1)
        x_hi = COLSIMAGE - 1;
    if (x_hi <= x_lo)
        return false;
    return true;
}


/**
 * @brief 预分配巡线点集 vector 容量
 * @return 无
 * @sample image_track_buffers_init();
 */
void image_track_buffers_init(void)
{
    if (s_track_bufs_inited)
        return;
    const size_t cap = (size_t)POINTS_MAX_LEN;
    pointsEdgeLeft.reserve(cap);          pointsEdgeRight.reserve(cap);
    t_pointsEdgeLeft.reserve(cap);        t_pointsEdgeRight.reserve(cap);
    b_t_pointsEdgeLeft.reserve(cap);      b_t_pointsEdgeRight.reserve(cap);
    s_b_t_pointsEdgeLeft.reserve(cap);    s_b_t_pointsEdgeRight.reserve(cap);
    a_t_pointsEdgeLeft.reserve(cap);      a_t_pointsEdgeRight.reserve(cap);
    n_a_t_pointsEdgeLeft.reserve(cap);    n_a_t_pointsEdgeRight.reserve(cap);
    t_left_CenterEdge.reserve(cap);       t_right_CenterEdge.reserve(cap);
    t_CenterEdge.reserve(cap);            CenterEdge.reserve(cap);
    bird_lut_init();
    s_track_bufs_inited = true;
}


/**
 * @brief 将俯视 t_CenterEdge 反透视写入 CenterEdge
 * @return 无
 * @sample image_sync_center_edge_roi();
 */
void image_sync_center_edge_roi(void)
{
    if (!s_center_edge_roi_dirty && CenterEdge_size > 0)
        return;

    CenterEdge.clear();
    CenterEdge_size = 0;
    for (int i = 0; i < t_CenterEdge_size; i++)
    {
        int a = 0, b = 0;
        if (bird_lut_reverse(a, b, t_CenterEdge[i].x, t_CenterEdge[i].y))
            CenterEdge.emplace_back(a, b);
    }
    CenterEdge_size = (int)CenterEdge.size();
    s_center_edge_roi_dirty = false;
}


/**
 * @brief 判定本帧是否走完整 trackRecognition 后处理链
 * @return true 需要 blur/角度/NMS/角点/直道检测
 */
static bool track_need_full_postprocess(void)
{
#if !TRACK_FAST_PATH
    return true;
#endif
    if (s_cross.flag_cross != Cross::Cross_None)
        return true;
    if (s_ring.flag_ring != Ring::Ring_None)
        return true;
    if (s_prev_L_left || s_prev_L_right)
        return true;
    if ((s_track_frame_idx % 3u) == 0u)
        return true;
    return false;
}


/**
 * @brief 校验透视边线 size 不超过 vector 实际长度
 * @param where 调用位置描述（用于断言日志）
 * @return      true 表示一致；false 表示已检测到不一致（ENABLE_TERMINAL_DEBUG 下 abort）
 */
static bool track_assert_edge_sizes(const char *where)
{
    bool ok = true;
    if (t_pointsEdgeLeft_size > (int)t_pointsEdgeLeft.size())
    {
        printf("[BUG] %s: tL size=%d vec=%zu\n",
               where, t_pointsEdgeLeft_size, t_pointsEdgeLeft.size());
        t_pointsEdgeLeft_size = (int)t_pointsEdgeLeft.size();
        ok = false;
    }
    if (t_pointsEdgeRight_size > (int)t_pointsEdgeRight.size())
    {
        printf("[BUG] %s: tR size=%d vec=%zu\n",
               where, t_pointsEdgeRight_size, t_pointsEdgeRight.size());
        t_pointsEdgeRight_size = (int)t_pointsEdgeRight.size();
        ok = false;
    }
#if defined(ENABLE_TERMINAL_DEBUG) && (ENABLE_TERMINAL_DEBUG == 1)
    if (!ok)
        abort();
#endif
    return ok;
}


/* ====================== 摄像头取帧 ====================== */

/**
 * @brief 摄像头取帧
 * @param camera     摄像头对象引用
 * @param raw        [out] BGR 320x240 原图
 * @param gray       [out] 灰度 320x240
 * @param cut_raw    [out] BGR ROI 320x130（y∈[ROI_TOP, ROI_BOTTOM)）
 * @param cut_gray   [out] 灰度 ROI 320x130（y∈[ROI_TOP, ROI_BOTTOM)）
 * @return           取帧成功为 true
 * @note             flip(-1) 与历史代码兼容（摄像头倒装时整图旋转 180°）
 */
bool image_get(lq_camera_ex &camera, cv::Mat &raw, cv::Mat &gray,
               cv::Mat &cut_raw, cv::Mat &cut_gray)
{
    bool ok = camera.get_frame_raw_gray(raw, gray);
    if (!ok || raw.empty() || gray.empty()) return false;

    const cv::Rect roi(0, ROI_TOP, COLSIMAGE, ROI_BOTTOM - ROI_TOP);

#if ENABLE_VISION_NCNN || ENABLE_VISION_BRICK
    cv::flip(raw,  raw,  -1);
    cv::flip(gray, gray, -1);
    cut_raw  = raw(roi);
    cut_gray = gray(roi);
#else
    /* 无彩色/NCNN 模块时仅 flip ROI，减少约 46% 像素搬运 */
    cut_raw  = raw(roi);
    cut_gray = gray(roi);
    cv::flip(cut_raw,  cut_raw,  -1);
    cv::flip(cut_gray, cut_gray, -1);
#endif
    return ok;
}


/* ====================== 主流水线 ====================== */

/**
 * @brief 巡线主流水线
 * @sample image_get(camera, rgb_img, gray_img, rgb_cut_img, gray_cut_img); image_process();
 */
void image_process(void)
{
    if (rgb_img.empty() || gray_img.empty()) return;

    image_track_buffers_init();
    s_track_frame_idx++;

#if ENABLE_PERF_IMAGE_STAGES
    const uint64_t t0 = perf_now_us_local();
#endif

    /* 1)  灰度 → OTSU → 闭运算（复用 bin_img 缓冲） */
    if (bin_img.rows != gray_cut_img.rows || bin_img.cols != gray_cut_img.cols
        || bin_img.type() != CV_8UC1)
    {
        bin_img.create(gray_cut_img.rows, gray_cut_img.cols, CV_8UC1);
    }
    if (!s_imgproc.processImageInPlace(bin_img, gray_cut_img) || bin_img.empty())
        return;

#if ENABLE_PERF_IMAGE_STAGES
    const uint64_t t1 = perf_now_us_local();
    perf_stats_image_stage_add(PERF_STAGE_BIN, t1 - t0);
#endif

    imgShow = rgb_img;

#if ENABLE_PERF_IMAGE_STAGES
    const uint64_t t_tr0 = perf_now_us_local();
#endif

    /* 3) 主巡线 */
    trackRecognition(bin_img);

#if ENABLE_PERF_IMAGE_STAGES
    const uint64_t t_tr1 = perf_now_us_local();
    perf_stats_image_stage_add(PERF_STAGE_TRACK, t_tr1 - t_tr0);
    const uint64_t t_cr0 = perf_now_us_local();
#endif

    /* 4) 元素状态机：先判定 Cross 与 Ring，再按 scene 进入相应逻辑 */
    const bool both_straight = is_left_straight && is_right_straight;
    s_cross.Cross_Check(is_t_L_pointLeft_find, is_t_L_pointRight_find, bin_img,
                        t_L_pointLeft, t_L_pointRight,
                        t_L_pointLeft_id, t_L_pointRight_id,
                        t_pointsEdgeLeft_size, t_pointsEdgeRight_size,
                        both_straight);
    s_cross.Cross_Run(t_pointsEdgeLeft, t_pointsEdgeRight, bin_img,
                      is_t_L_pointLeft_find, is_t_L_pointRight_find,
                      t_L_pointLeft_id, t_L_pointRight_id,
                      t_pointsEdgeLeft_size, t_pointsEdgeRight_size,
                      both_straight);

    if (s_cross.flag_cross != Cross::Cross_None || banmaxian_is_armed())
        banmaxian_check(bin_img, (int)s_cross.flag_cross);

    if (s_cross.flag_cross == Cross::Cross_None)
    {
        s_ring.Ring_Check(bin_img,
                          is_left_straight, is_right_straight,
                          t_pointsEdgeLeft_size, t_pointsEdgeRight_size,
                          is_t_L_pointLeft_find, is_t_L_pointRight_find,
                          t_L_pointLeft, t_L_pointRight,
                          t_pointsEdgeLeft, t_pointsEdgeRight);
        s_ring.Ring_Run(t_pointsEdgeLeft, t_pointsEdgeRight,
                        t_pointsEdgeLeft_size, t_pointsEdgeRight_size,
                        is_t_L_pointLeft_find, is_t_L_pointRight_find,
                        t_L_pointLeft_id, t_L_pointRight_id,
                        bin_img);
    }

    if (s_cross.flag_cross == Cross::Cross_Out
        || s_cross.flag_cross == Cross::Cross_Begin)
    {
        if (t_pointsEdgeLeft_size <= 0 && t_pointsEdgeRight_size <= 0)
        {
            recover_bird_edge_if_empty(0);
            recover_bird_edge_if_empty(1);
        }
    }
    else
    {
        recover_bird_edge_if_empty(0);
        recover_bird_edge_if_empty(1);
    }

    if (s_cross.flag_cross != Cross::Cross_None)      scene = (int)Scene::CrossScene;
    else if (s_ring.flag_ring != Ring::Ring_None)     scene = (int)Scene::RingScene;
    else                                              scene = (int)Scene::NormalScene;

    track_assert_edge_sizes("pre-fitting");

#if ENABLE_PERF_IMAGE_STAGES
    const uint64_t t_cr1 = perf_now_us_local();
    perf_stats_image_stage_add(PERF_STAGE_CROSS_RING, t_cr1 - t_cr0);
    const uint64_t t_fit0 = perf_now_us_local();
#endif

    fitting();

#if ENABLE_PERF_IMAGE_STAGES
    const uint64_t t_fit1 = perf_now_us_local();
    perf_stats_image_stage_add(PERF_STAGE_FITTING, t_fit1 - t_fit0);
#endif

    s_prev_L_left  = is_t_L_pointLeft_find;
    s_prev_L_right = is_t_L_pointRight_find;
    s_center_edge_roi_dirty = true;
}


/**
 * @brief 将十字、环岛状态机复位为 None，scene 置 NormalScene
 * @return 无
 * @sample track_elements_reset();
 * @note  供按键/调试调用；不修改 t_pointsEdge 边线，下一帧 trackRecognition 自然刷新
 */
void track_elements_reset(void)
{
    s_cross.reset();
    s_ring.reset();
    banmaxian_reset();
    scene = (int)Scene::NormalScene;
}


/**
 * @brief 中线横向像素偏差（后轮差速，无阿克曼几何）
 * @return aim_angle 横向偏差（像素），avg_x - COLSIMAGE/2
 * @sample image_process(); float err = Image_Error_Get();
 * @note   对俯视图 t_CenterEdge index 15~20 共 6 点求平均列坐标；
 *         归一化无效或点数≤20 时沿用 aim_angle_last；
 *         写入 img_err 供 motor.cpp yaw_loop / pd_yaw。
 *         中线偏右时 avg_x 大，偏差为正（与角速度环差速符号一致）。
 *         Cross_Out 阶段 img_err 限幅 ±5 px（控制量）；返回值 aim_angle 仍为原始偏差。
 */
float Image_Error_Get(void)
{
    if (!s_center_effective || t_CenterEdge_size <= 20)
    {
        aim_angle = aim_angle_last;
    }
    else
    {
        int sum_x = 0;
        for (int i = 25; i <= 30; i++)
            sum_x += t_CenterEdge[i].x;
        const float avg_x = (float)sum_x / 6.0f;
        aim_angle = avg_x - (float)(COLSIMAGE / 2);
        aim_angle_last = aim_angle;
    }

    img_err = aim_angle;
    // if (s_cross.flag_cross == Cross::Cross_Out)
    //     img_err = limit_float(img_err, -2.0f, 2.0f);
    return aim_angle;
}


/* ============================================================================
 *                           以下为内部静态函数实现
 * ========================================================================== */

/**
 * @brief 巡线主识别（迷宫法 + 透视 + 后处理 + 找拐点）
 * @param imageBinary OTSU + 闭运算后的 ROI 二值图（320x130，y 为局部行号 0..ROI_H-1）
 * @note  对应 temp_repo Standard::trackRecognition；输入为 gray_cut 同尺寸 bin_img
 */
static void trackRecognition(cv::Mat &imageBinary)
{
    const int bin_rows = imageBinary.rows;
    const int bin_cols = imageBinary.cols;

    /* 清空所有工作缓存 */
    pointsEdgeLeft.clear();        pointsEdgeLeft_size        = 0;
    pointsEdgeRight.clear();       pointsEdgeRight_size       = 0;
    t_pointsEdgeLeft.clear();      t_pointsEdgeLeft_size      = 0;
    t_pointsEdgeRight.clear();     t_pointsEdgeRight_size     = 0;
    b_t_pointsEdgeLeft.clear();    b_t_pointsEdgeLeft_size    = 0;
    b_t_pointsEdgeRight.clear();   b_t_pointsEdgeRight_size   = 0;
    s_b_t_pointsEdgeLeft.clear();  s_b_t_pointsEdgeLeft_size  = 0;
    s_b_t_pointsEdgeRight.clear(); s_b_t_pointsEdgeRight_size = 0;
    a_t_pointsEdgeLeft.clear();    a_t_pointsEdgeLeft_size    = 0;
    a_t_pointsEdgeRight.clear();   a_t_pointsEdgeRight_size   = 0;
    n_a_t_pointsEdgeLeft.clear();  n_a_t_pointsEdgeLeft_size  = 0;
    n_a_t_pointsEdgeRight.clear(); n_a_t_pointsEdgeRight_size = 0;
    g_track_row_bounds_valid = false;

    is_t_L_pointLeft_find  = false;
    is_t_L_pointRight_find = false;
    is_left_straight       = false;
    is_right_straight      = false;
    is_left_curve          = false;
    is_right_curve         = false;
    t_L_pointLeft_id       = 0;
    t_L_pointRight_id      = 0;

    /* ===== 1) floodFill：把不连通白噪声涂黑，保留赛道 ===== */
    // cv::Mat mask = cv::Mat::zeros(bin_rows + 2, bin_cols + 2, CV_8UC1);
    // bool flood_effect = false;
    // if (seedPoint.x < 0 || seedPoint.x >= bin_cols
    //  || seedPoint.y < 0 || seedPoint.y >= bin_rows)
    // {
    //     seedPoint = cv::Point(bin_cols / 2, bin_rows - 1);
    // }
    // if (imageBinary.at<uchar>(seedPoint.y, seedPoint.x) != 255)
    // {
    //     /* 在 ROI 底边附近重新找种子点 */
    //     int sy = bin_rows - 1;
    //     if (aim_angle_last > 0)
    //     {
    //         for (int x = bin_cols / 2 - 10; x > 80; x--)
    //         {
    //             if (imageBinary.at<uchar>(sy, x) == 255)
    //             {
    //                 seedPoint = cv::Point(x, sy);
    //                 flood_effect = true;
    //                 break;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         for (int x = bin_cols / 2 + 10; x < bin_cols - 80; x++)
    //         {
    //             if (imageBinary.at<uchar>(sy, x) == 255)
    //             {
    //                 seedPoint = cv::Point(x, sy);
    //                 flood_effect = true;
    //                 break;
    //             }
    //         }
    //     }
    // }
    // else
    // {
    //     flood_effect = true;
    // }
    // if (flood_effect)
    // {
    //     cv::floodFill(imageBinary, mask, seedPoint, 127, 0, 0, 4);
    //     for (int y = 0; y < bin_rows; y++)
    //     {
    //         for (int x = 0; x < bin_cols; x++)
    //         {
    //             if (imageBinary.at<uchar>(y, x) == 127)
    //                 imageBinary.at<uchar>(y, x) = 255;
    //             else
    //                 imageBinary.at<uchar>(y, x) = 0;
    //         }
    //     }
    // }
    // mask.release();

    /* ===== 2) 底行跳变：正常从中心扫；视觉绕行时从上一帧边线起点 ±5px 锚定 ===== */
    int y0 = rowCutBottom_roi;
    int y1 = rowCutBottom_roi;
    const int mid = bin_cols / 2;
    int x0v = mid;
    int x1v = mid;
    const int scan_l = track_bypass_seed_scan_from(0, mid, EDGE_MARGIN, bin_cols);
    const int scan_r = track_bypass_seed_scan_from(1, mid, EDGE_MARGIN, bin_cols);
    bool found_l = find_bottom_edge_seed_left(imageBinary, y0, scan_l, EDGE_MARGIN, x0v);
    bool found_r = find_bottom_edge_seed_right(imageBinary, y1, scan_r, EDGE_MARGIN, x1v);
    if (!found_l && scan_l != mid)
        found_l = find_bottom_edge_seed_left(imageBinary, y0, mid, EDGE_MARGIN, x0v);
    if (!found_r && scan_r != mid)
        found_r = find_bottom_edge_seed_right(imageBinary, y1, mid, EDGE_MARGIN, x1v);
    if (found_l)
        x0_seed = x0v;
    else
        x0v = x0_seed;
    if (found_r)
        x1_seed = x1v;
    else
        x1v = x1_seed;

#if 0
    /* [legacy] 最长白列 + 水平扫黑起点，已由底行跳变替代（保留备查） */
    /* ===== 2) 最长白列算法：求左右起点 x0 / x1 ===== */
    int left_start[COLSIMAGE]  = {0};
    int right_start[COLSIMAGE] = {0};
    x0_seed = COLSIMAGE / 2;
    x1_seed = COLSIMAGE / 2;

    for (int i = 0; i < bin_cols; i++)
    {
        for (int j = rowCutBottom_roi; j > rowCutUp_roi; j--)
        {
            if (imageBinary.at<uchar>(j, i) < thresOTSU) break;
            left_start[i]++;
            right_start[i]++;
        }
    }
    for (int i = 1; i < bin_cols; i++)
    {
        if (left_start[i]                  > left_start[x0_seed])  x0_seed = i;
        if (right_start[bin_cols - 1 - i] > right_start[x1_seed]) x1_seed = bin_cols - 1 - i;
    }

    /* ===== 3) 迷宫法左右手巡边线 ===== */
    int y0 = rowCutBottom_roi;
    int x0v = x0_seed;
    for (; x0v > 0; x0v--)
    {
        if (imageBinary.at<uchar>(y0, x0v - 1) < thresOTSU) break;
    }
    if (imageBinary.at<uchar>(y0, x0v) >= thresOTSU && x0v > 10)
        findline_lefthand_adaptive(imageBinary, block_size, clip_value, x0v, y0,
                                   pointsEdgeLeft, pointsEdgeLeft_size);
    else
        pointsEdgeLeft_size = 0;

    int y1 = rowCutBottom_roi;
    int x1v = x1_seed;
    for (; x1v < bin_cols - 1; x1v++)
    {
        if (imageBinary.at<uchar>(y1, x1v + 1) < thresOTSU) break;
    }
    if (imageBinary.at<uchar>(y1, x1v) >= thresOTSU && x1v < WIDTH - 10)
        findline_righthand_adaptive(imageBinary, block_size, clip_value, x1v, y1,
                                    pointsEdgeRight, pointsEdgeRight_size);
    else
        pointsEdgeRight_size = 0;
#endif

    /* ===== 3) 迷宫法左右手巡边线 ===== */
    if (imageBinary.at<uchar>(y0, x0v) >= thresOTSU && x0v > EDGE_MARGIN)
        findline_core::lefthand_fixed(imageBinary, x0v, y0,
                                      pointsEdgeLeft, pointsEdgeLeft_size);
    else
        pointsEdgeLeft_size = 0;

    if (imageBinary.at<uchar>(y1, x1v) >= thresOTSU && x1v < WIDTH - EDGE_MARGIN)
        findline_core::righthand_fixed(imageBinary, x1v, y1,
                                       pointsEdgeRight, pointsEdgeRight_size);
    else
        pointsEdgeRight_size = 0;

    track_build_row_bounds_from_maze();

    /* ===== 4) 透视变换（LUT） ===== */
    for (int i = 0; i < pointsEdgeLeft_size; i++)
    {
        int a = 0, b = 0;
        if (bird_lut_transf(a, b, pointsEdgeLeft[i].x, pointsEdgeLeft[i].y))
            t_pointsEdgeLeft.emplace_back(a, b);
    }
    t_pointsEdgeLeft_size = (int)t_pointsEdgeLeft.size();
    for (int i = 0; i < pointsEdgeRight_size; i++)
    {
        int a = 0, b = 0;
        if (bird_lut_transf(a, b, pointsEdgeRight[i].x, pointsEdgeRight[i].y))
            t_pointsEdgeRight.emplace_back(a, b);
    }
    t_pointsEdgeRight_size = (int)t_pointsEdgeRight.size();

    const bool full_post = track_need_full_postprocess();
    const int blur_k = full_post ? 11 : 5;

    /* ===== 5) 滤波 ===== */
    blur_points(0, blur_k);
    blur_points(1, blur_k);

    /* ===== 6) 等距采样 ===== */
    resample_points(b_t_pointsEdgeLeft,  b_t_pointsEdgeLeft_size,
                    s_b_t_pointsEdgeLeft, s_b_t_pointsEdgeLeft_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));
    resample_points(b_t_pointsEdgeRight, b_t_pointsEdgeRight_size,
                    s_b_t_pointsEdgeRight, s_b_t_pointsEdgeRight_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));

    if (full_post)
    {
        /* ===== 7) 角度 ===== */
        local_angle_points(s_b_t_pointsEdgeLeft,  s_b_t_pointsEdgeLeft_size,
                           a_t_pointsEdgeLeft, 7);
        a_t_pointsEdgeLeft_size = (int)a_t_pointsEdgeLeft.size();
        local_angle_points(s_b_t_pointsEdgeRight, s_b_t_pointsEdgeRight_size,
                           a_t_pointsEdgeRight, 7);
        a_t_pointsEdgeRight_size = (int)a_t_pointsEdgeRight.size();

        /* ===== 8) NMS ===== */
        nms_angle(a_t_pointsEdgeLeft,  a_t_pointsEdgeLeft_size,
                  n_a_t_pointsEdgeLeft, 14);
        n_a_t_pointsEdgeLeft_size = (int)n_a_t_pointsEdgeLeft.size();
        nms_angle(a_t_pointsEdgeRight, a_t_pointsEdgeRight_size,
                  n_a_t_pointsEdgeRight, 14);
        n_a_t_pointsEdgeRight_size = (int)n_a_t_pointsEdgeRight.size();

        /* ===== 9) 写回 t_pointsEdge ===== */
        t_pointsEdgeLeft.swap(s_b_t_pointsEdgeLeft);
        t_pointsEdgeLeft_size = (int)t_pointsEdgeLeft.size();
        t_pointsEdgeRight.swap(s_b_t_pointsEdgeRight);
        t_pointsEdgeRight_size = (int)t_pointsEdgeRight.size();
        s_b_t_pointsEdgeLeft.clear();
        s_b_t_pointsEdgeRight.clear();
        s_b_t_pointsEdgeLeft_size  = 0;
        s_b_t_pointsEdgeRight_size = 0;

        line_straight_detection();
        find_corners();
        s_straight_hold_l = is_left_straight;
        s_straight_hold_r = is_right_straight;
    }
    else
    {
        t_pointsEdgeLeft.swap(s_b_t_pointsEdgeLeft);
        t_pointsEdgeLeft_size = (int)t_pointsEdgeLeft.size();
        t_pointsEdgeRight.swap(s_b_t_pointsEdgeRight);
        t_pointsEdgeRight_size = (int)t_pointsEdgeRight.size();
        s_b_t_pointsEdgeLeft.clear();
        s_b_t_pointsEdgeRight.clear();
        s_b_t_pointsEdgeLeft_size  = 0;
        s_b_t_pointsEdgeRight_size = 0;

        is_left_straight  = s_straight_hold_l;
        is_right_straight = s_straight_hold_r;
        is_left_curve     = false;
        is_right_curve    = false;
    }
}


/** @brief 巡线侧选择（对齐 standard.cpp TrackState，不含双边贝塞尔） */
enum class TrackState
{
    TRACK_NONE  = 0,
    TRACK_LEFT  = 1,
    TRACK_RIGHT = 2,
};

static TrackState s_track_state = TrackState::TRACK_NONE;
static constexpr int RB_CENTER_SHIFT_PX = 10;


/**
 * @brief 按边线点数选定巡线侧（点多选边，差≤1 用 aim_angle_last 防抖）
 * @note  逻辑对齐 temp_repo Standard::run 中 trackState 判定，不使用 TRACK_BOTH
 */
static void select_track_state(void)
{
    switch (s_ring.flag_ring)
    {
    case Ring::Left_Ring_Begin:
    case Ring::Left_Ring_Out:
    case Ring::Right_Ring_In:
        s_track_state = TrackState::TRACK_RIGHT;
        return;
    case Ring::Left_Ring_In:
    case Ring::Right_Ring_Begin:
    case Ring::Right_Ring_Out:
        s_track_state = TrackState::TRACK_LEFT;
        return;
    default:
        break;
    }

    const int dl = t_pointsEdgeLeft_size;
    const int dr = t_pointsEdgeRight_size;

    if ((dr - dl) > 1)
        s_track_state = TrackState::TRACK_RIGHT;
    else if ((dl - dr) > 1)
        s_track_state = TrackState::TRACK_LEFT;
    else if (dl > dr)
        s_track_state = TrackState::TRACK_LEFT;
    else if (dr > dl)
        s_track_state = TrackState::TRACK_RIGHT;
    else if (dl > 0 && dr > 0)
    {
        /* aim_angle>0：中线偏右，跟左侧边线 */
        if (aim_angle_last > 0.f)
            s_track_state = TrackState::TRACK_LEFT;
        else
            s_track_state = TrackState::TRACK_RIGHT;
    }
    else if (dl > 0)
        s_track_state = TrackState::TRACK_LEFT;
    else if (dr > 0)
        s_track_state = TrackState::TRACK_RIGHT;
    else
        s_track_state = TrackState::TRACK_NONE;
}


/**
 * @brief 左单边中线写入 t_CenterEdge
 * @note  对应 standard.cpp track_left 分支
 */
static void track_left(void)
{
    t_CenterEdge.clear();
    for (int i = 0; i < t_left_CenterEdge_size; i++)
        t_CenterEdge.emplace_back(t_left_CenterEdge[i].x, t_left_CenterEdge[i].y);
    t_CenterEdge_size = (int)t_CenterEdge.size();
}


/**
 * @brief 右单边中线写入 t_CenterEdge
 * @note  对应 standard.cpp track_right 分支
 */
static void track_right(void)
{
    t_CenterEdge.clear();
    for (int i = 0; i < t_right_CenterEdge_size; i++)
        t_CenterEdge.emplace_back(t_right_CenterEdge[i].x, t_right_CenterEdge[i].y);
    t_CenterEdge_size = (int)t_CenterEdge.size();
}

/**
 * @brief 直接使用单侧边线，按固定像素平移生成中线
 * @param edge      [in]  俯视图边线点列
 * @param edge_size [in]  逻辑点数
 * @param shift_x   [in]  列方向平移量（像素）：左移为负、右移为正
 * @return 生成成功且点数>=3 返回 true
 * @sample track_shifted_from_edge(t_pointsEdgeRight, t_pointsEdgeRight_size, -10);
 * @note y 保持原值，x 夹紧到 [0, COLSIMAGE-1]，确保后续 normalizeCenterEdge 可用。
 */
static bool track_shifted_from_edge(const std::vector<POINT> &edge, int edge_size, int shift_x)
{
    t_CenterEdge.clear();
    t_CenterEdge_size = 0;

    if (edge_size <= 0) return false;
    const int vec_size = (int)edge.size();
    if (vec_size <= 0) return false;

    const int use_n = std::min(edge_size, vec_size);
    for (int i = 0; i < use_n; i++)
    {
        const int sx = std::clamp(edge[i].x + shift_x, 0, COLSIMAGE - 1);
        const int sy = std::clamp(edge[i].y, 0, ROI_H - 1);
        t_CenterEdge.emplace_back(sx, sy);
    }
    t_CenterEdge_size = (int)t_CenterEdge.size();
    return t_CenterEdge_size >= 3;
}


/**
 * @brief 拟合中线（单边法向偏移；点多选边，不用双边 Bezier）
 * @note  红砖 ACTIVE 时优先使用边线±10px 直接生成中线；否则走常规单边法向偏移选边。
 */
static void fitting()
{
    t_left_CenterEdge.clear();   t_left_CenterEdge_size  = 0;
    t_right_CenterEdge.clear();  t_right_CenterEdge_size = 0;
    t_CenterEdge.clear();        t_CenterEdge_size       = 0;
    CenterEdge.clear();          CenterEdge_size         = 0;

    centerCompute(t_pointsEdgeLeft,  t_pointsEdgeLeft_size,  0);
    t_left_CenterEdge_size = (int)t_left_CenterEdge.size();
    centerCompute(t_pointsEdgeRight, t_pointsEdgeRight_size, 1);
    t_right_CenterEdge_size = (int)t_right_CenterEdge.size();

    bool rb_override_ok = false;
    if (g_brick_avoider.get_state() == RB_STATE_ACTIVE)
    {
        if (g_brick_avoider.get_force_track_type() == FORCE_RIGHT_LINE)
            rb_override_ok = track_shifted_from_edge(
                t_pointsEdgeRight, t_pointsEdgeRight_size, -RB_CENTER_SHIFT_PX);
        else if (g_brick_avoider.get_force_track_type() == FORCE_LEFT_LINE)
            rb_override_ok = track_shifted_from_edge(
                t_pointsEdgeLeft, t_pointsEdgeLeft_size, RB_CENTER_SHIFT_PX);
    }

#if ENABLE_VISION_BYPASS
    /* 视觉绕行优先级低于红砖避障：仅在红砖 NORMAL 且未由 brick override 写入中线时生效。
     * 左绕行 → 左线列方向 +VISION_BYPASS_SHIFT_PX；右绕行 → 右线列方向 -VISION_BYPASS_SHIFT_PX。 */
    if (!rb_override_ok && g_brick_avoider.get_state() == RB_STATE_NORMAL)
    {
        if (g_vision_bypass_action == VBA_LEFT)
            rb_override_ok = track_shifted_from_edge(
                t_pointsEdgeLeft,  t_pointsEdgeLeft_size,  +VISION_BYPASS_SHIFT_PX);
        else if (g_vision_bypass_action == VBA_RIGHT)
            rb_override_ok = track_shifted_from_edge(
                t_pointsEdgeRight, t_pointsEdgeRight_size, -VISION_BYPASS_SHIFT_PX);
    }
#endif

    if (!rb_override_ok)
    {
        select_track_state();
        if (s_track_state == TrackState::TRACK_LEFT)
            track_left();
        else if (s_track_state == TrackState::TRACK_RIGHT)
            track_right();

        if (t_CenterEdge_size <= 0 && t_left_CenterEdge_size >= 3)
            track_left();
        else if (t_CenterEdge_size <= 0 && t_right_CenterEdge_size >= 3)
            track_right();
    }

    /* 起始点中线归一化（俯视图）：截断车前段 + 等距重采样 */
    float cx = COLSIMAGE / 2.0f;
    float cy = (float)(ROI_H * 0.95);
    s_center_effective = normalizeCenterEdge(cx, cy);
    s_track_cx = cx;
    s_track_cy = cy;

    /* 反透视：按需由 image_sync_center_edge_roi() 填充 CenterEdge */
    s_center_edge_roi_dirty = true;
}


/**
 * @brief 按锚点与中线第一点纵向关系分两支归一化并等距重采样
 * @param[in,out] cx  锚点列（俯视图），输入为车体参考列，输出保持不变
 * @param[in,out] cy  锚点行（俯视图），输入为车体参考行，输出保持不变
 * @return            true 表示重采样成功且点数足够，可继续预瞄
 * @sample            fitting() 内: normalizeCenterEdge(cx, cy);  // cx=COLSIMAGE/2, cy=ROI_H*0.95
 * @note              t_CenterEdge 须在俯视图 [0,COLSIMAGE)×[0,ROI_H)；y 向下增大。
 *                    t_CenterEdge[0].y > cy：第一点已在锚点下方，保留原中线 [0,end) 后重采样；
 *                    t_CenterEdge[0].y <= cy：第一点在上，锚点作首点再接原中线后重采样。
 */
static bool normalizeCenterEdge(float &cx, float &cy)
{
    if (t_CenterEdge_size <= 0)
        return false;

    const float anchor_x = cx;
    const float anchor_y = cy;

    std::vector<POINT> temp_center;
    temp_center.reserve((size_t)t_CenterEdge_size + 1);

    if ((float)t_CenterEdge[0].y > anchor_y)
    {
        /* 情形一：第一点已在锚点下方 → 原中线从 index 0 到末尾 */
        for (int i = 0; i < t_CenterEdge_size; i++)
            temp_center.emplace_back(t_CenterEdge[i].x, t_CenterEdge[i].y);
    }
    else
    {
        /* 情形二：第一点在上 → 锚点作首点，再接原中线 */
        temp_center.emplace_back((int)(anchor_x + 0.5f), (int)(anchor_y + 0.5f));
        for (int i = 0; i < t_CenterEdge_size; i++)
            temp_center.emplace_back(t_CenterEdge[i].x, t_CenterEdge[i].y);
    }

    if ((int)temp_center.size() < 3)
        return false;

    int temp_center_size = (int)temp_center.size();
    t_CenterEdge.clear();
    t_CenterEdge_size = 0;
    resample_points(temp_center, temp_center_size, t_CenterEdge, t_CenterEdge_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));
    return t_CenterEdge_size >= 2;
}


/* ============================ 底行跳变起点 ============================ */

/**
 * @brief 按视觉绕行状态计算底行跳变扫描起点
 * @param side      0=左线，1=右线
 * @param mid       图像中心列（正常模式）
 * @param margin    边界留白 EDGE_MARGIN
 * @param cols      图像宽度
 * @return          本帧 scan_from（已 clamp 到合法范围）
 * @note            VBA_RIGHT 时右线从 x1_seed-5 起扫；VBA_LEFT 时左线从 x0_seed+5 起扫；
 *                  VBA_STRAIGHT 或非 ENABLE_VISION_BYPASS 时返回 mid
 */
static int track_bypass_seed_scan_from(int side, int mid, int margin, int cols)
{
    const int x_min = margin + EDGE_CHECK_DIS;
    const int x_max = cols - margin - EDGE_CHECK_DIS - 1;
    int scan = mid;

#if ENABLE_VISION_BYPASS
    if (side == 1 && g_vision_bypass_action == VBA_RIGHT)
        scan = x1_seed - VISION_BYPASS_SHIFT_PX;
    else if (side == 0 && g_vision_bypass_action == VBA_LEFT)
        scan = x0_seed + VISION_BYPASS_SHIFT_PX;
#endif

    if (scan < x_min)
        scan = x_min;
    else if (scan > x_max)
        scan = x_max;
    return scan;
}


/**
 * @brief 在固定行上从 scan_from 向左找白→黑跳变作为左迷宫起点
 * @param img       二值图（ROI 局部坐标）
 * @param y         扫描行号（局部行，常用 rowCutBottom_roi）
 * @param scan_from 扫描起始列号（正常模式为 mid，绕行时为上一帧左线起点 +5）
 * @param margin    左右边界留白（列号下限为 margin + EDGE_CHECK_DIS）
 * @param out_x     [out] 跳变后白侧列号
 * @return          找到合法跳变为 true
 * @note            取离 scan_from 最近且通过 EDGE_CHECK_DIS 连续段检验的跳变
 */
static bool find_bottom_edge_seed_left(const cv::Mat &img, int y, int scan_from, int margin,
                                       int &out_x)
{
    const int cols = img.cols;
    const int left_stop = margin + EDGE_CHECK_DIS;
    if (y < 0 || y >= img.rows || scan_from < left_stop)
        return false;

    for (int x = scan_from; x > left_stop; --x)
    {
        if (img.at<uchar>(y, x) < thresOTSU || img.at<uchar>(y, x - 1) >= thresOTSU)
            continue;

        bool ok_white = true;
        for (int k = 0; k <= EDGE_CHECK_DIS && ok_white; ++k)
        {
            int xi = x + k;
            if (xi >= cols || img.at<uchar>(y, xi) < thresOTSU)
                ok_white = false;
        }
        bool ok_black = true;
        for (int k = 1; k <= EDGE_CHECK_DIS && ok_black; ++k)
        {
            int xi = x - k;
            if (xi < 0 || img.at<uchar>(y, xi) >= thresOTSU)
                ok_black = false;
        }
        if (ok_white && ok_black)
        {
            out_x = x;
            return true;
        }
    }
    return false;
}


/**
 * @brief 在固定行上从 scan_from 向右找白→黑跳变作为右迷宫起点
 * @param img       二值图（ROI 局部坐标）
 * @param y         扫描行号（局部行，常用 rowCutBottom_roi）
 * @param scan_from 扫描起始列号（正常模式为 mid，绕行时为上一帧右线起点 -5）
 * @param margin    左右边界留白（列号上限为 cols - margin - EDGE_CHECK_DIS - 1）
 * @param out_x     [out] 跳变前白侧列号
 * @return          找到合法跳变为 true
 * @note            取离 scan_from 最近且通过 EDGE_CHECK_DIS 连续段检验的跳变
 */
static bool find_bottom_edge_seed_right(const cv::Mat &img, int y, int scan_from, int margin,
                                        int &out_x)
{
    const int cols = img.cols;
    const int right_stop = cols - margin - EDGE_CHECK_DIS - 1;
    if (y < 0 || y >= img.rows || scan_from > right_stop)
        return false;

    for (int x = scan_from; x < right_stop; ++x)
    {
        if (img.at<uchar>(y, x) < thresOTSU || img.at<uchar>(y, x + 1) >= thresOTSU)
            continue;

        bool ok_white = true;
        for (int k = 0; k <= EDGE_CHECK_DIS && ok_white; ++k)
        {
            int xi = x - k;
            if (xi < 0 || img.at<uchar>(y, xi) < thresOTSU)
                ok_white = false;
        }
        bool ok_black = true;
        for (int k = 1; k <= EDGE_CHECK_DIS && ok_black; ++k)
        {
            int xi = x + k;
            if (xi >= cols || img.at<uchar>(y, xi) >= thresOTSU)
                ok_black = false;
        }
        if (ok_white && ok_black)
        {
            out_x = x;
            return true;
        }
    }
    return false;
}


/* ============================ 后处理 ============================ */

/**
 * @brief 三角窗口平滑边线点
 * @param side   0=左边线，1=右边线
 * @param kernel 滤波核宽度（建议 11）
 */
static void blur_points(int side, int kernel)
{
    int half = kernel / 2;
    if (side == 0)
    {
        b_t_pointsEdgeLeft.clear();
        for (int i = 0; i < t_pointsEdgeLeft_size; i++)
        {
            b_t_pointsEdgeLeft.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                int idx = s_general.clip(i + j, 0, t_pointsEdgeLeft_size - 1);
                b_t_pointsEdgeLeft[i].x +=
                    t_pointsEdgeLeft[idx].x * (half + 1 - std::abs(j));
                b_t_pointsEdgeLeft[i].y +=
                    t_pointsEdgeLeft[idx].y * (half + 1 - std::abs(j));
            }
            b_t_pointsEdgeLeft[i].x /= (2 * half + 2) * (half + 1) / 2;
            b_t_pointsEdgeLeft[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    else
    {
        b_t_pointsEdgeRight.clear();
        for (int i = 0; i < t_pointsEdgeRight_size; i++)
        {
            b_t_pointsEdgeRight.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                int idx = s_general.clip(i + j, 0, t_pointsEdgeRight_size - 1);
                b_t_pointsEdgeRight[i].x +=
                    t_pointsEdgeRight[idx].x * (half + 1 - std::abs(j));
                b_t_pointsEdgeRight[i].y +=
                    t_pointsEdgeRight[idx].y * (half + 1 - std::abs(j));
            }
            b_t_pointsEdgeRight[i].x /= (2 * half + 2) * (half + 1) / 2;
            b_t_pointsEdgeRight[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    b_t_pointsEdgeLeft_size  = (int)b_t_pointsEdgeLeft.size();
    b_t_pointsEdgeRight_size = (int)b_t_pointsEdgeRight.size();
}


/**
 * @brief 边线等距采样
 * @param in      输入点序列
 * @param in_size 输入点数
 * @param out     [out] 输出点序列
 * @param out_size [out] 输出点数
 * @param dist    采样间隔（像素）
 */
static void resample_points(std::vector<POINT> &in, int in_size,
                            std::vector<POINT> &out, int &out_size, float dist)
{
    const int vec_n = (int)in.size();
    if (in_size > vec_n)
        in_size = vec_n;
    if (in_size < 2)
    {
        out_size = 0;
        return;
    }

    int remain = 0;
    int len    = 0;
    for (int i = 0; i < in_size - 1; i++)
    {
        float x0v = in[i].x;
        float y0v = in[i].y;
        float dx  = in[i + 1].x - x0v;
        float dy  = in[i + 1].y - y0v;
        float dn  = std::sqrt(dx * dx + dy * dy);
        if (dn < 1e-6f) continue;
        dx /= dn;
        dy /= dn;
        while (remain < dn)
        {
            x0v += dx * remain;
            y0v += dy * remain;
            out.emplace_back((int)x0v, (int)y0v);
            len++;
            dn    -= remain;
            remain = (int)dist;
        }
        remain -= (int)dn;
    }
    out_size = len;
}


/**
 * @brief 局部角度计算
 * @param pointsEdgeIn  输入点序列
 * @param size          输入点数
 * @param pointsEdgeOut [out] 输出点序列（写入 angle 字段）
 * @param dist          前后步长（单位：点）
 */
static void local_angle_points(const std::vector<POINT> &pointsEdgeIn, int size,
                               std::vector<POINT> &pointsEdgeOut, int dist)
{
    for (int i = 0; i < size; i++)
    {
        pointsEdgeOut.emplace_back(pointsEdgeIn[i].x, pointsEdgeIn[i].y);
        if (i <= 0 || i >= size - 1)
        {
            pointsEdgeOut[i].angle = 0;
            continue;
        }
        float dx1 = pointsEdgeIn[i].x
                  - pointsEdgeIn[s_general.clip(i - dist, 0, size - 1)].x;
        float dy1 = pointsEdgeIn[i].y
                  - pointsEdgeIn[s_general.clip(i - dist, 0, size - 1)].y;
        float dn1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        float dx2 = pointsEdgeIn[s_general.clip(i + dist, 0, size - 1)].x
                  - pointsEdgeIn[i].x;
        float dy2 = pointsEdgeIn[s_general.clip(i + dist, 0, size - 1)].y
                  - pointsEdgeIn[i].y;
        float dn2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        if (dn1 < 1e-6f || dn2 < 1e-6f) { pointsEdgeOut[i].angle = 0; continue; }
        float c1 = dx1 / dn1, s1 = dy1 / dn1;
        float c2 = dx2 / dn2, s2 = dy2 / dn2;
        pointsEdgeOut[i].angle = std::atan2(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}


/**
 * @brief 角度非极大值抑制
 * @param in       输入点序列（angle 已计算）
 * @param in_size  点数
 * @param out      [out] 输出点序列
 * @param kernel   滑动窗口宽度
 */
static void nms_angle(std::vector<POINT> &in, int in_size,
                      std::vector<POINT> &out, int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < in_size; i++)
    {
        out.emplace_back(in[i].x, in[i].y);
        out[i].angle = in[i].angle;
        for (int j = -half; j <= half; j++)
        {
            if (std::fabs(in[s_general.clip(i + j, 0, in_size - 1)].angle)
                > std::fabs(out[i].angle))
            {
                out[i].angle = 0;
                break;
            }
        }
    }
}


/**
 * @brief 单边巡线生成中线（基于法向位移 dist_half_road）
 * @param pointsEdge 边线点序列
 * @param size       点数
 * @param side       0=左边线（中线在边线右），1=右边线
 */
static void centerCompute(const std::vector<POINT> &pointsEdge, int size, int side)
{
    const int vec_n = (int)pointsEdge.size();
    if (size > vec_n)
        size = vec_n;
    if (size < 6)
        return;

    if (side == 0)
    {
        for (int i = 0; i < size - 3; i++)
        {
            float dx = pointsEdge[s_general.clip(i + approx_num, 0, size - 1)].x
                     - pointsEdge[s_general.clip(i - approx_num, 0, size - 1)].x;
            float dy = pointsEdge[s_general.clip(i + approx_num, 0, size - 1)].y
                     - pointsEdge[s_general.clip(i - approx_num, 0, size - 1)].y;
            float dn = std::sqrt(dx * dx + dy * dy);
            if (dn < 1e-6f) continue;
            dx /= dn;
            dy /= dn;
            t_left_CenterEdge.emplace_back((int)(pointsEdge[i].x - dy * dist_half_road),
                                           (int)(pointsEdge[i].y + dx * dist_half_road));
        }
    }
    else
    {
        for (int i = 0; i < size - 3; i++)
        {
            float dx = pointsEdge[s_general.clip(i + approx_num, 0, size - 1)].x
                     - pointsEdge[s_general.clip(i - approx_num, 0, size - 1)].x;
            float dy = pointsEdge[s_general.clip(i + approx_num, 0, size - 1)].y
                     - pointsEdge[s_general.clip(i - approx_num, 0, size - 1)].y;
            float dn = std::sqrt(dx * dx + dy * dy);
            if (dn < 1e-6f) continue;
            dx /= dn;
            dy /= dn;
            t_right_CenterEdge.emplace_back((int)(pointsEdge[i].x + dy * dist_half_road),
                                            (int)(pointsEdge[i].y - dx * dist_half_road));
        }
    }
}


/**
 * @brief 俯视图边线被元素机裁空时，从 ROI 迷宫边线重新透视
 * @param side 0=左，1=右
 * @return 无
 * @note  仅当 t_pointsEdge 为空且 pointsEdge 非空时执行；跳过越界点
 */
static void recover_bird_edge_if_empty(int side)
{
    if (side == 0)
    {
        if (t_pointsEdgeLeft_size > 0 || pointsEdgeLeft_size <= 0)
            return;
        t_pointsEdgeLeft.clear();
        for (int i = 0; i < pointsEdgeLeft_size; i++)
        {
            int a = 0, b = 0;
            if (bird_lut_transf(a, b, pointsEdgeLeft[i].x, pointsEdgeLeft[i].y))
                t_pointsEdgeLeft.emplace_back(a, b);
        }
        t_pointsEdgeLeft_size = (int)t_pointsEdgeLeft.size();
    }
    else
    {
        if (t_pointsEdgeRight_size > 0 || pointsEdgeRight_size <= 0)
            return;
        t_pointsEdgeRight.clear();
        for (int i = 0; i < pointsEdgeRight_size; i++)
        {
            int a = 0, b = 0;
            if (bird_lut_transf(a, b, pointsEdgeRight[i].x, pointsEdgeRight[i].y))
                t_pointsEdgeRight.emplace_back(a, b);
        }
        t_pointsEdgeRight_size = (int)t_pointsEdgeRight.size();
    }
}


/**
 * @brief 双边巡线生成中线（4 控制点贝塞尔）
 * @note  控制点取自 t_pointsEdgeLeft / t_pointsEdgeRight 的 0、1/3、2/3、末点
 */
// static void track_both_edge()
// {
//     if (t_pointsEdgeLeft_size <= 0 || t_pointsEdgeRight_size <= 0) return;
//     std::vector<POINT> v_center(4);
//     v_center[0] = POINT((t_pointsEdgeLeft[0].x + t_pointsEdgeRight[0].x) / 2,
//                         (t_pointsEdgeLeft[0].y + t_pointsEdgeRight[0].y) / 2);
//     v_center[1] = POINT((t_pointsEdgeLeft[t_pointsEdgeLeft_size / 3].x
//                         + t_pointsEdgeRight[t_pointsEdgeRight_size / 3].x) / 2,
//                         (t_pointsEdgeLeft[t_pointsEdgeLeft_size / 3].y
//                         + t_pointsEdgeRight[t_pointsEdgeRight_size / 3].y) / 2);
//     v_center[2] = POINT((t_pointsEdgeLeft[t_pointsEdgeLeft_size * 2 / 3].x
//                         + t_pointsEdgeRight[t_pointsEdgeRight_size * 2 / 3].x) / 2,
//                         (t_pointsEdgeLeft[t_pointsEdgeLeft_size * 2 / 3].y
//                         + t_pointsEdgeRight[t_pointsEdgeRight_size * 2 / 3].y) / 2);
//     v_center[3] = POINT((t_pointsEdgeLeft[t_pointsEdgeLeft_size - 1].x
//                         + t_pointsEdgeRight[t_pointsEdgeRight_size - 1].x) / 2,
//                         (t_pointsEdgeLeft[t_pointsEdgeLeft_size - 1].y
//                         + t_pointsEdgeRight[t_pointsEdgeRight_size - 1].y) / 2);
//     t_both_CenterEdge = s_general.Bezier(0.01, v_center);
//     t_both_CenterEdge_size = (int)t_both_CenterEdge.size();
// }


/**
 * @brief 直道判断（基于 fitLine 残差）
 * @note  m_ea < 50 视为直道，m_ea > 200 视为明显弯道
 */
static void line_straight_detection()
{
    if (t_pointsEdgeLeft_size > 30)
    {
        std::vector<cv::Point> points;
        int trans[2];
        int y_counter = 0;
        for (int i = 0; i < t_pointsEdgeLeft_size && i < 0.6 / SAMPLE_DIST; i++, y_counter++)
        {
            s_general.Reverse_transf(trans[0], trans[1],
                                     t_pointsEdgeLeft[i].x, t_pointsEdgeLeft[i].y);
            points.push_back(cv::Point(trans[0], trans[1]));
        }
        if (!points.empty())
        {
            cv::Vec4f line_para;
            cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
            float k_ = line_para[1] / (line_para[0] + 1e-6f);
            float b_ = line_para[3] - k_ * line_para[2];
            float m_ea = 0.0f;
            for (int i = 0; i < y_counter; i++)
                m_ea += std::fabs(k_ * points[i].x + b_ - points[i].y);
            if (m_ea < 65.f && t_pointsEdgeLeft_size > 30) is_left_straight = true;
            else if (m_ea > 200.f) is_left_curve = true;
        }
    }
    if (t_pointsEdgeRight_size > 30)
    {
        std::vector<cv::Point> points;
        int trans[2];
        int y_counter = 0;
        for (int i = 0; i < t_pointsEdgeRight_size && i < 0.6 / SAMPLE_DIST; i++, y_counter++)
        {
            s_general.Reverse_transf(trans[0], trans[1],
                                     t_pointsEdgeRight[i].x, t_pointsEdgeRight[i].y);
            points.push_back(cv::Point(trans[0], trans[1]));
        }
        if (!points.empty())
        {
            cv::Vec4f line_para;
            cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
            float k_ = line_para[1] / (line_para[0] + 1e-6f);
            float b_ = line_para[3] - k_ * line_para[2];
            float m_ea = 0.0f;
            for (int i = 0; i < y_counter; i++)
                m_ea += std::fabs(k_ * points[i].x + b_ - points[i].y);
            if (m_ea < 65.0f && t_pointsEdgeRight_size > 30) is_right_straight = true;
            else if (m_ea > 200.f) is_right_curve = true;
        }
    }
}


/**
 * @brief 找 L 角点（基于 NMS 后角度）
 */
static void find_corners()
{
    if (!is_left_straight && t_pointsEdgeLeft_size > 20)
    {
        for (int i = 0; i < n_a_t_pointsEdgeLeft_size; i++)
        {
            int im1 = s_general.clip(i - 2, 0, n_a_t_pointsEdgeLeft_size - 1);
            int ip1 = s_general.clip(i + 2, 0, n_a_t_pointsEdgeLeft_size - 1);
            float conf = std::fabs(n_a_t_pointsEdgeLeft[i].angle)
                       - (std::fabs(n_a_t_pointsEdgeLeft[im1].angle)
                        + std::fabs(n_a_t_pointsEdgeLeft[ip1].angle)) / 2.f;
            conf = std::fabs(conf * 180.0f / PI);
            if (!is_t_L_pointLeft_find && LCONF_MIN < conf && conf < LCONF_MAX)
            {
                for (int j = 0; j < t_pointsEdgeLeft_size; j++)
                {
                    if (t_pointsEdgeLeft[j].x == n_a_t_pointsEdgeLeft[i].x
                     && t_pointsEdgeLeft[j].y == n_a_t_pointsEdgeLeft[i].y)
                    {
                        /* 候选下标小于 MIN_CORNER_ID 视为鸟瞰图底部噪声，
                         * 跳出内层匹配但不锁定，让外层 i 继续往后扫描下一个候选 */
                        if (j < MIN_CORNER_ID) break;
                        t_L_pointLeft_id      = j;
                        is_t_L_pointLeft_find = true;
                        t_L_pointLeft         = cv::Point(t_pointsEdgeLeft[j].x,
                                                          t_pointsEdgeLeft[j].y);
                        break;
                    }
                }
            }
            else if (is_t_L_pointLeft_find) break;
        }
    }
    if (!is_right_straight && t_pointsEdgeRight_size > 20)
    {
        for (int i = 0; i < n_a_t_pointsEdgeRight_size; i++)
        {
            int im1 = s_general.clip(i - 2, 0, n_a_t_pointsEdgeRight_size - 1);
            int ip1 = s_general.clip(i + 2, 0, n_a_t_pointsEdgeRight_size - 1);
            float conf = std::fabs(n_a_t_pointsEdgeRight[i].angle)
                       - (std::fabs(n_a_t_pointsEdgeRight[im1].angle)
                        + std::fabs(n_a_t_pointsEdgeRight[ip1].angle)) / 2.f;
            conf = std::fabs(conf * 180.0f / PI);
            if (!is_t_L_pointRight_find && LCONF_MIN < conf && conf < LCONF_MAX)
            {
                for (int j = 0; j < t_pointsEdgeRight_size; j++)
                {
                    if (t_pointsEdgeRight[j].x == n_a_t_pointsEdgeRight[i].x
                     && t_pointsEdgeRight[j].y == n_a_t_pointsEdgeRight[i].y)
                    {
                        /* 候选下标小于 MIN_CORNER_ID 视为鸟瞰图底部噪声，
                         * 跳出内层匹配但不锁定，让外层 i 继续往后扫描下一个候选 */
                        if (j < MIN_CORNER_ID) break;
                        t_L_pointRight_id      = j;
                        is_t_L_pointRight_find = true;
                        t_L_pointRight         = cv::Point(t_pointsEdgeRight[j].x,
                                                           t_pointsEdgeRight[j].y);
                        break;
                    }
                }
            }
            else if (is_t_L_pointRight_find) break;
        }
    }
}


/* ====================== 阶段 C 调试 ====================== */

/**
 * @brief 从全局巡线变量填充阶段 C 快照
 * @param out [out] 输出结构体
 * @return  无
 */
void track_debug_fill_phase_c(TrackDebugSnapshotC &out)
{
    out.pl   = pointsEdgeLeft_size;
    out.pr   = pointsEdgeRight_size;
    out.tl   = t_pointsEdgeLeft_size;
    out.tr   = t_pointsEdgeRight_size;
    out.ll   = is_t_L_pointLeft_find  ? 1 : 0;
    out.lr   = is_t_L_pointRight_find ? 1 : 0;
    out.lxid = is_t_L_pointLeft_find  ? t_L_pointLeft_id  : -1;
    out.lyid = is_t_L_pointLeft_find  ? t_L_pointLeft.y   : -1;
    out.rxid = is_t_L_pointRight_find ? t_L_pointRight_id : -1;
    out.ryid = is_t_L_pointRight_find ? t_L_pointRight.y  : -1;
    out.lx   = is_t_L_pointLeft_find  ? t_L_pointLeft.x   : -1;
    out.ly   = is_t_L_pointLeft_find  ? t_L_pointLeft.y   : -1;
    out.rx   = is_t_L_pointRight_find ? t_L_pointRight.x  : -1;
    out.ry   = is_t_L_pointRight_find ? t_L_pointRight.y  : -1;
}

/**
 * @brief 十字状态枚举转可读字符串
 * @param flag Cross::flag_Cross_e 整型值
 * @return     状态名称（小写）
 */
static const char *cross_flag_name(int flag)
{
    switch (flag)
    {
    case Cross::Cross_None:  return "cross none";
    case Cross::Cross_Begin: return "cross begin";
    case Cross::Cross_Out:   return "cross out";
    default:                 return "cross ?";
    }
}

/**
 * @brief 环岛状态枚举转可读字符串
 * @param flag Ring::flag_Ring_e 整型值
 * @return     状态名称（小写）
 */
static const char *ring_flag_name(int flag)
{
    switch (flag)
    {
    case Ring::Ring_None:        return "ring none";
    case Ring::Left_Ring_Begin:  return "left ring begin";
    case Ring::Left_Ring_In:     return "left ring in";
    case Ring::Left_Ring_Out:    return "left ring out";
    case Ring::Right_Ring_Begin: return "right ring begin";
    case Ring::Right_Ring_In:    return "right ring in";
    case Ring::Right_Ring_Out:   return "right ring out";
    default:                     return "ring ?";
    }
}

/**
 * @brief 阶段 C 低频串口输出
 * @return 无
 * @note  调试打印已关闭；十字/环岛状态见 track_debug_print_phase_d_throttled
 */
void track_debug_print_phase_c_throttled(void)
{
    static int frame_cnt = 0;
    static int last_ll   = 0;
    static int last_lr   = 0;
    TrackDebugSnapshotC s{};
    track_debug_fill_phase_c(s);

    // if (s.ll && !last_ll)
    //     printf("[C] L_corner id=%d (%d,%d)\n", s.lxid, s.lx, s.ly);
    // if (s.lr && !last_lr)
    //     printf("[C] R_corner id=%d (%d,%d)\n", s.rxid, s.rx, s.ry);
    last_ll = s.ll;
    last_lr = s.lr;

    if (++frame_cnt % 30 != 0)
        return;
    // printf("[C] L=%d R=%d tL=%d tR=%d LL=%d LR=%d\n",
    //        s.pl, s.pr, s.tl, s.tr, s.ll, s.lr);
}


/* ====================== 阶段 D~G 调试 ====================== */

/**
 * @brief 从全局巡线变量填充阶段 D~G 快照
 * @param out [out] 输出结构体
 * @return  无
 */
void track_debug_fill_phase_d(TrackDebugSnapshotD &out)
{
    out.aim_angle  = aim_angle;
    out.center_ok  = s_center_effective ? 1 : 0;
    out.track_cx   = (int)(s_track_cx + 0.5f);
    out.track_cy   = (int)(s_track_cy + 0.5f);
    out.track_state = (int)s_track_state;
    out.scene      = scene;
    out.cross_flag = (int)s_cross.flag_cross;
    out.ring_flag  = (int)s_ring.flag_ring;
    out.straight_l = is_left_straight  ? 1 : 0;
    out.straight_r = is_right_straight ? 1 : 0;
    out.curve_l    = is_left_curve     ? 1 : 0;
    out.curve_r    = is_right_curve    ? 1 : 0;
}

/**
 * @brief 阶段 D~G 低频串口输出
 * @return 无
 * @note  十字/环岛状态变化时打印可读状态名；其余调试输出已关闭
 */
void track_debug_print_phase_d_throttled(void)
{
    static int frame_cnt    = 0;
    static int last_cross   = 0;
    static int last_ring    = 0;
    TrackDebugSnapshotD s{};
    track_debug_fill_phase_d(s);

    if (s.cross_flag != last_cross)
        printf("[scene] %s\n", cross_flag_name(s.cross_flag));
    if (s.ring_flag != last_ring)
        printf("[scene] %s\n", ring_flag_name(s.ring_flag));
    last_cross = s.cross_flag;
    last_ring  = s.ring_flag;

    if (++frame_cnt % 30 != 0)
        return;
    // printf("[D] err=%.2f cen=%d cx=%d cy=%d | [F] trk=%d | [G] scn=%d X=%d R=%d\n",
    //        s.aim_angle, s.center_ok, s.track_cx, s.track_cy,
    //        s.track_state, s.scene, s.cross_flag, s.ring_flag);
}
