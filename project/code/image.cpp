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

#include "image.hpp"
#include "motor.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>


/* ====================== 全局图像 ====================== */
cv::Mat rgb_img;
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


/* ====================== 内部函数声明 ====================== */
static void findline_lefthand_adaptive(cv::Mat &img, int /*bs*/, int /*cv*/,
                                       int x, int y,
                                       std::vector<POINT> &out, int &out_size);
static void findline_righthand_adaptive(cv::Mat &img, int /*bs*/, int /*cv*/,
                                        int x, int y,
                                        std::vector<POINT> &out, int &out_size);
static void blur_points(int side, int kernel);
static void resample_points(std::vector<POINT> &in, int in_size,
                            std::vector<POINT> &out, int &out_size, float dist);
static void local_angle_points(std::vector<POINT> pointsEdgeIn, int size,
                               std::vector<POINT> &pointsEdgeOut, int dist);
static void nms_angle(std::vector<POINT> &in, int in_size,
                      std::vector<POINT> &out, int kernel);
static void centerCompute(std::vector<POINT> pointsEdge, int size, int side);
static void track_both_edge();
static void line_straight_detection();
static void find_corners();
static void trackRecognition(cv::Mat &imageBinary);
static void fitting();
static bool normalizeCenterEdge(float &cx, float &cy);


/* ====================== 摄像头取帧 ====================== */

/**
 * @brief 摄像头取帧
 * @param camera     摄像头对象引用
 * @param raw        [out] BGR 320x240 原图
 * @param gray       [out] 灰度 320x240
 * @return           取帧成功为 true
 * @note             flip(-1) 与历史代码兼容（摄像头倒装时整图旋转 180°）
 */
bool image_get(lq_camera_ex &camera, cv::Mat &raw, cv::Mat &gray, cv::Mat &cut_gray)
{
    bool ok = camera.get_frame_raw_gray(raw, gray);
    if (!ok || raw.empty() || gray.empty()) return false;
    cv::flip(raw,  raw,  -1);
    cv::flip(gray, gray, -1);
    cv::Rect roi(0, ROI_TOP, COLSIMAGE, ROI_BOTTOM - ROI_TOP);
    cut_gray = gray(roi);
    return ok;
}


/* ====================== 主流水线 ====================== */

/**
 * @brief 巡线主流水线
 * @sample image_get(camera, rgb_img, gray_img, cut_gray); image_process();
 */
void image_process(void)
{
    if (rgb_img.empty() || gray_img.empty()) return;

    /* 1)  灰度 → OTSU → 闭运算 */
    bin_img = s_imgproc.processImage(gray_cut_img);
    if (bin_img.empty()) return;
    
    bin_bird_img = s_imgproc.image_correction(bin_img);

    /* 2) 调试可视化图（en_show=false 时仅占位） */
    /* TODO: 可视化总开关，暂时 imgShow = rgb_img 的浅拷贝即可 */
    imgShow = rgb_img;

    /* 3) 主巡线 */
    trackRecognition(bin_img);

    /* 4) 元素状态机：先判定 Cross 与 Ring，再按 scene 进入相应逻辑 */
    s_cross.Cross_Check(is_t_L_pointLeft_find, is_t_L_pointRight_find, bin_img,
                        t_L_pointLeft, t_L_pointRight,
                        t_L_pointLeft_id, t_L_pointRight_id,
                        t_pointsEdgeLeft_size, t_pointsEdgeRight_size);
    s_cross.Cross_Run(t_pointsEdgeLeft, t_pointsEdgeRight, bin_img,
                      is_t_L_pointLeft_find, is_t_L_pointRight_find,
                      t_L_pointLeft_id, t_L_pointRight_id,
                      t_pointsEdgeLeft_size, t_pointsEdgeRight_size);

    s_ring.Ring_Check(bin_img,
                      is_left_straight, is_right_straight,
                      is_t_L_pointLeft_find, is_t_L_pointRight_find,
                      t_pointsEdgeLeft_size, t_pointsEdgeRight_size,
                      is_t_L_pointLeft_find, is_t_L_pointRight_find,
                      t_L_pointLeft_id, t_L_pointRight_id,
                      t_pointsEdgeLeft, t_pointsEdgeRight);
    s_ring.Ring_Run(t_pointsEdgeLeft, t_pointsEdgeRight,
                    t_pointsEdgeLeft_size, t_pointsEdgeRight_size,
                    is_t_L_pointLeft_find, is_t_L_pointRight_find,
                    t_L_pointLeft_id, t_L_pointRight_id,
                    bin_img);

    /* 5) scene 联动（十字 / 环岛场景标记） */
    if (s_cross.flag_cross != Cross::Cross_None)      scene = (int)Scene::CrossScene;
    else if (s_ring.flag_ring != Ring::Ring_None)     scene = (int)Scene::RingScene;
    else                                              scene = (int)Scene::NormalScene;

    /* 6) 拟合中线 */
    fitting();
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
    scene = (int)Scene::NormalScene;
}


/**
 * @brief 中线横向像素偏差（后轮差速，无阿克曼几何）
 * @return aim_angle 横向偏差（像素），COLSIMAGE/2 - avg_x
 * @sample image_process(); float err = Image_Error_Get();
 * @note   对俯视图 t_CenterEdge index 15~20 共 6 点求平均列坐标；
 *         归一化无效或点数≤20 时沿用 aim_angle_last；
 *         写入 img_err 供 motor.cpp yaw_loop / pd_yaw。
 *         中线偏右时 avg_x 大，偏差为负。
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
        for (int i = 15; i <= 20; i++)
            sum_x += t_CenterEdge[i].x;
        const float avg_x = (float)sum_x / 6.0f;
        aim_angle = (float)(COLSIMAGE / 2) - avg_x;
        aim_angle_last = aim_angle;
    }

    img_err = aim_angle;
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
    is_t_L_pointLeft_find  = false;
    is_t_L_pointRight_find = false;
    is_left_straight       = false;
    is_right_straight      = false;
    is_left_curve          = false;
    is_right_curve         = false;
    t_L_pointLeft_id       = 0;
    t_L_pointRight_id      = 0;

    /* ===== 1) floodFill：把不连通白噪声涂黑，保留赛道 ===== */
    cv::Mat mask = cv::Mat::zeros(bin_rows + 2, bin_cols + 2, CV_8UC1);
    bool flood_effect = false;
    if (seedPoint.x < 0 || seedPoint.x >= bin_cols
     || seedPoint.y < 0 || seedPoint.y >= bin_rows)
    {
        seedPoint = cv::Point(bin_cols / 2, bin_rows - 1);
    }
    if (imageBinary.at<uchar>(seedPoint.y, seedPoint.x) != 255)
    {
        /* 在 ROI 底边附近重新找种子点 */
        int sy = bin_rows - 1;
        if (aim_angle_last > 0)
        {
            for (int x = bin_cols / 2 - 10; x > 80; x--)
            {
                if (imageBinary.at<uchar>(sy, x) == 255)
                {
                    seedPoint = cv::Point(x, sy);
                    flood_effect = true;
                    break;
                }
            }
        }
        else
        {
            for (int x = bin_cols / 2 + 10; x < bin_cols - 80; x++)
            {
                if (imageBinary.at<uchar>(sy, x) == 255)
                {
                    seedPoint = cv::Point(x, sy);
                    flood_effect = true;
                    break;
                }
            }
        }
    }
    else
    {
        flood_effect = true;
    }
    if (flood_effect)
    {
        cv::floodFill(imageBinary, mask, seedPoint, 127, 0, 0, 4);
        for (int y = 0; y < bin_rows; y++)
        {
            for (int x = 0; x < bin_cols; x++)
            {
                if (imageBinary.at<uchar>(y, x) == 127)
                    imageBinary.at<uchar>(y, x) = 255;
                else
                    imageBinary.at<uchar>(y, x) = 0;
            }
        }
    }
    mask.release();

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

    /* 圆形伪线过滤（直行邻接 4 点完全相同视为环形）!!!!!疑似有问题 */
    if (pointsEdgeLeft_size > 8)
    {
        bool circular = true;
        for (int i = 0; i < 4; i++)
            if (pointsEdgeLeft[i].x != pointsEdgeLeft[i + 4].x
             || pointsEdgeLeft[i].y != pointsEdgeLeft[i + 4].y) { circular = false; break; }
        if (circular) pointsEdgeLeft_size = 0;
    }
    if (pointsEdgeRight_size > 8)
    {
        bool circular = true;
        for (int i = 0; i < 4; i++)
            if (pointsEdgeRight[i].x != pointsEdgeRight[i + 4].x
             || pointsEdgeRight[i].y != pointsEdgeRight[i + 4].y) { circular = false; break; }
        if (circular) pointsEdgeRight_size = 0;
    }

    /* ===== 4) 透视变换 ===== */
    for (int i = 0; i < pointsEdgeLeft_size; i++)
    {
        int a, b;
        if (s_general.transf(a, b, pointsEdgeLeft[i].x, pointsEdgeLeft[i].y))
            t_pointsEdgeLeft.emplace_back(a, b);
        else
            break;
    }
    t_pointsEdgeLeft_size = (int)t_pointsEdgeLeft.size();
    for (int i = 0; i < pointsEdgeRight_size; i++)
    {
        int a, b;
        if (s_general.transf(a, b, pointsEdgeRight[i].x, pointsEdgeRight[i].y))
            t_pointsEdgeRight.emplace_back(a, b);
        else
            break;
    }
    t_pointsEdgeRight_size = (int)t_pointsEdgeRight.size();

    /* ===== 5) 滤波 ===== */
    blur_points(0, 11);
    blur_points(1, 11);
    b_t_pointsEdgeLeft_size  = t_pointsEdgeLeft_size;
    b_t_pointsEdgeRight_size = t_pointsEdgeRight_size;

    /* ===== 6) 等距采样 ===== */
    resample_points(b_t_pointsEdgeLeft,  b_t_pointsEdgeLeft_size,
                    s_b_t_pointsEdgeLeft, s_b_t_pointsEdgeLeft_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));
    resample_points(b_t_pointsEdgeRight, b_t_pointsEdgeRight_size,
                    s_b_t_pointsEdgeRight, s_b_t_pointsEdgeRight_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));

    /* ===== 7) 角度 ===== */
    local_angle_points(s_b_t_pointsEdgeLeft,  s_b_t_pointsEdgeLeft_size,
                       a_t_pointsEdgeLeft, 11);
    a_t_pointsEdgeLeft_size = (int)a_t_pointsEdgeLeft.size();
    local_angle_points(s_b_t_pointsEdgeRight, s_b_t_pointsEdgeRight_size,
                       a_t_pointsEdgeRight, 11);
    a_t_pointsEdgeRight_size = (int)a_t_pointsEdgeRight.size();

    /* ===== 8) NMS ===== */
    nms_angle(a_t_pointsEdgeLeft,  a_t_pointsEdgeLeft_size,
              n_a_t_pointsEdgeLeft, 22);
    n_a_t_pointsEdgeLeft_size = (int)n_a_t_pointsEdgeLeft.size();
    nms_angle(a_t_pointsEdgeRight, a_t_pointsEdgeRight_size,
              n_a_t_pointsEdgeRight, 22);
    n_a_t_pointsEdgeRight_size = (int)n_a_t_pointsEdgeRight.size();

    /* ===== 9) 把等距采样后点写回 t_pointsEdge ===== */
    t_pointsEdgeLeft.clear();   t_pointsEdgeLeft_size  = 0;
    t_pointsEdgeRight.clear();  t_pointsEdgeRight_size = 0;
    for (int i = 0; i < s_b_t_pointsEdgeLeft_size; i++)
    {
        t_pointsEdgeLeft.emplace_back(s_b_t_pointsEdgeLeft[i].x,
                                      s_b_t_pointsEdgeLeft[i].y);
        t_pointsEdgeLeft_size++;
    }
    for (int i = 0; i < s_b_t_pointsEdgeRight_size; i++)
    {
        t_pointsEdgeRight.emplace_back(s_b_t_pointsEdgeRight[i].x,
                                       s_b_t_pointsEdgeRight[i].y);
        t_pointsEdgeRight_size++;
    }

    /* ===== 10) 直道判断 + 找 L 角点 ===== */
    line_straight_detection();
    find_corners();
}


/** @brief 巡线侧选择（对齐 standard.cpp TrackState，不含双边贝塞尔） */
enum class TrackState
{
    TRACK_NONE  = 0,
    TRACK_LEFT  = 1,
    TRACK_RIGHT = 2,
};

static TrackState s_track_state = TrackState::TRACK_NONE;


/**
 * @brief 按边线点数选定巡线侧（点多选边，差≤1 用 aim_angle_last 防抖）
 * @note  逻辑对齐 temp_repo Standard::run 中 trackState 判定，不使用 TRACK_BOTH
 */
static void select_track_state(void)
{
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
        if (aim_angle_last < 0.f)
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
 * @brief 拟合中线（单边法向偏移；点多选边，不用双边 Bezier）
 * @note  centerCompute 生成左右单边中线，select_track_state + track_left/right 选定 t_CenterEdge
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

    select_track_state();
    if (s_track_state == TrackState::TRACK_LEFT)
        track_left();
    else if (s_track_state == TrackState::TRACK_RIGHT)
        track_right();

    /* 起始点中线归一化（俯视图）：截断车前段 + 等距重采样 */
    float cx = COLSIMAGE / 2.0f;
    float cy = (float)(ROI_H * 0.95);
    s_center_effective = normalizeCenterEdge(cx, cy);
    s_track_cx = cx;
    s_track_cy = cy;

    /* 反透视：基于归一化后的 t_CenterEdge，送逐飞助手 XY_BOUNDARY（ROI 局部坐标） */
    for (int i = 0; i < t_CenterEdge_size; i++)
    {
        int a, b;
        if (s_general.Reverse_transf(a, b, t_CenterEdge[i].x, t_CenterEdge[i].y))
            CenterEdge.emplace_back(a, b);
        else
            break;
    }
    CenterEdge_size = (int)CenterEdge.size();
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


/* ============================ 迷宫法（适配版） ============================ */

/**
 * @brief 左手迷宫法巡线
 * @param img          二值图
 * @param x,y          起点坐标
 * @param out          输出点序列
 * @param out_size     输出点数
 * @note  无自适应阈值版本，固定阈值 128 与 temp_repo 对齐；
 *        循环上限 POINTS_MAX_LEN（180）。
 */
static void findline_lefthand_adaptive(cv::Mat &img, int /*bs*/, int /*cv*/,
                                       int x, int y,
                                       std::vector<POINT> &out, int &out_size)
{
    // int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while ((step < POINTS_MAX_LEN)
        && 0 < x && x < (img.cols  - 1)
        && 0 < y && y < (img.rows  - 1)
        && turn < 4)
    {
        int local_thres     = 128;
        int front_value     = img.at<uchar>(y + s_cross.dir_front[dir][1],
                                            x + s_cross.dir_front[dir][0]);
        int frontleft_value = img.at<uchar>(y + s_cross.dir_frontleft[dir][1],
                                            x + s_cross.dir_frontleft[dir][0]);
        if (front_value < local_thres) { dir = (dir + 1) % 4; turn++; }
        else if (frontleft_value < local_thres)
        {
            x += s_cross.dir_front[dir][0];
            y += s_cross.dir_front[dir][1];
            out.emplace_back(x, y);
            step++; turn = 0;
        }
        else
        {
            x += s_cross.dir_frontleft[dir][0];
            y += s_cross.dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            out.emplace_back(x, y);
            step++; turn = 0;
        }
    }
    out_size = step;
}


/**
 * @brief 右手迷宫法巡线
 */
static void findline_righthand_adaptive(cv::Mat &img, int /*bs*/, int /*cv*/,
                                        int x, int y,
                                        std::vector<POINT> &out, int &out_size)
{
    int step = 0, dir = 0, turn = 0;
    while ((step < POINTS_MAX_LEN)
        && 0 < x && x < (img.cols - 1)
        && 0 < y && y < (img.rows - 1)
        && turn < 4)
    {
        int local_thres      = 128;
        int front_value      = img.at<uchar>(y + s_cross.dir_front[dir][1],
                                             x + s_cross.dir_front[dir][0]);
        int frontright_value = img.at<uchar>(y + s_cross.dir_frontright[dir][1],
                                             x + s_cross.dir_frontright[dir][0]);
        if (front_value < local_thres) { dir = (dir + 3) % 4; turn++; }
        else if (frontright_value < local_thres)
        {
            x += s_cross.dir_front[dir][0];
            y += s_cross.dir_front[dir][1];
            out.emplace_back(x, y);
            step++; turn = 0;
        }
        else
        {
            x += s_cross.dir_frontright[dir][0];
            y += s_cross.dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            out.emplace_back(x, y);
            step++; turn = 0;
        }
    }
    out_size = step;
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
static void local_angle_points(std::vector<POINT> pointsEdgeIn, int size,
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
static void centerCompute(std::vector<POINT> pointsEdge, int size, int side)
{
    if (side == 0)
    {
        for (int i = 0; i < size - 10; i++)
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
        for (int i = 0; i < size - 10; i++)
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
        for (int i = 0; i < t_pointsEdgeLeft_size && i < 0.7 / SAMPLE_DIST; i++, y_counter++)
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
            if (m_ea < 50.f && t_pointsEdgeLeft_size > 30) is_left_straight = true;
            else if (m_ea > 200.f) is_left_curve = true;
        }
    }
    if (t_pointsEdgeRight_size > 30)
    {
        std::vector<cv::Point> points;
        int trans[2];
        int y_counter = 0;
        for (int i = 0; i < t_pointsEdgeRight_size && i < 0.7 / SAMPLE_DIST; i++, y_counter++)
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
            if (m_ea < 50.f && t_pointsEdgeRight_size > 30) is_right_straight = true;
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
 * @brief 阶段 C 低频串口输出
 * @return 无
 * @note  约每 30 帧打印一次；L 角点 0→1 时额外打印坐标
 */
void track_debug_print_phase_c_throttled(void)
{
    static int frame_cnt = 0;
    static int last_ll   = 0;
    static int last_lr   = 0;
    TrackDebugSnapshotC s{};
    track_debug_fill_phase_c(s);

    if (s.ll && !last_ll)
        printf("[C] L_corner id=%d (%d,%d)\n", s.lxid, s.lx, s.ly);
    if (s.lr && !last_lr)
        printf("[C] R_corner id=%d (%d,%d)\n", s.rxid, s.rx, s.ry);
    last_ll = s.ll;
    last_lr = s.lr;

    if (++frame_cnt % 30 != 0)
        return;
    printf("[C] L=%d R=%d tL=%d tR=%d LL=%d LR=%d\n",
           s.pl, s.pr, s.tl, s.tr, s.ll, s.lr);
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
 * @note  约每 30 帧打印一次；十字/环岛状态变化时额外打印
 */
void track_debug_print_phase_d_throttled(void)
{
    static int frame_cnt    = 0;
    static int last_cross   = 0;
    static int last_ring    = 0;
    static int last_track   = 0;
    TrackDebugSnapshotD s{};
    track_debug_fill_phase_d(s);

    if (s.cross_flag != last_cross)
        printf("[G] cross_flag %d -> %d\n", last_cross, s.cross_flag);
    if (s.ring_flag != last_ring)
        printf("[G] ring_flag %d -> %d\n", last_ring, s.ring_flag);
    if (s.track_state != last_track)
        printf("[F] track_state %d -> %d (tL=%d tR=%d)\n",
               last_track, s.track_state, t_pointsEdgeLeft_size, t_pointsEdgeRight_size);
    last_cross = s.cross_flag;
    last_ring  = s.ring_flag;
    last_track = s.track_state;

    if (++frame_cnt % 30 != 0)
        return;
    printf("[D] err=%.2f cen=%d cx=%d cy=%d | [F] trk=%d | [G] scn=%d X=%d R=%d\n",
           s.aim_angle, s.center_ok, s.track_cx, s.track_cy,
           s.track_state, s.scene, s.cross_flag, s.ring_flag);
}
