#include "display.hpp"
#include "image.hpp"
#include "cross.hpp"
#include "ring.hpp"
#include "redbrick.hpp"
#include "vision.hpp"
#include "zf_common_headfile.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

static zf_device_ips200 *s_ips = nullptr;

static constexpr int DISP_W      = 240;
static constexpr int DISP_H      = ROI_H;
static constexpr int BIN_THRESH  = 128;
static constexpr int IPS_SCR_H   = 320;

static inline int iroundf_local(float v)
{
    return (int)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

static constexpr int ELEMENT_STATUS_W = 8;

/**
 * @brief 将十字状态枚举转为定宽 ASCII 文本
 * @param flag  Cross::flag_Cross_e 整型值
 * @param out   输出缓冲，至少 ELEMENT_STATUS_W+1 字节
 * @return      out 指针
 * @note        右侧补空格，避免 IPS 上残留旧字符
 */
static const char *cross_status_to_text(int flag, char *out)
{
    const char *text = "?";
    switch (flag)
    {
    case Cross::Cross_None:  text = "None";  break;
    case Cross::Cross_Begin: text = "Begin"; break;
    case Cross::Cross_Out:   text = "Out";   break;
    default: break;
    }
    snprintf(out, ELEMENT_STATUS_W + 1, "%-*s", ELEMENT_STATUS_W, text);
    return out;
}

/**
 * @brief 将环岛状态枚举转为定宽 ASCII 文本
 * @param flag  Ring::flag_Ring_e 整型值
 * @param out   输出缓冲，至少 ELEMENT_STATUS_W+1 字节
 * @return      out 指针
 * @note        右侧补空格，避免 IPS 上残留旧字符
 */
static const char *ring_status_to_text(int flag, char *out)
{
    const char *text = "?";
    switch (flag)
    {
    case Ring::Ring_None:           text = "None";  break;
    case Ring::Left_Ring_Begin:     text = "L-BEG"; break;
    case Ring::Left_Ring_In:        text = "L-IN";  break;
    case Ring::Left_Ring_Out:       text = "L-OUT"; break;
    case Ring::Right_Ring_Begin:    text = "R-BEG"; break;
    case Ring::Right_Ring_In:       text = "R-IN";  break;
    case Ring::Right_Ring_Out:      text = "R-OUT"; break;
    default: break;
    }
    snprintf(out, ELEMENT_STATUS_W + 1, "%-*s", ELEMENT_STATUS_W, text);
    return out;
}

/**
 * @brief 将 ROI 边线点绘制到 IPS 巡线区域（X 缩放到 DISP_W）
 * @param pts   点序列
 * @param n     点数
 * @param color RGB565 颜色
 */
static void draw_edge_on_track(const std::vector<POINT> &pts, int n, uint16 color)
{
    if (!s_ips || n <= 0) return;
    const int vec_n = (int)pts.size();
    if (n > vec_n)
        n = vec_n;
    for (int i = 0; i < n; i++)
    {
        int sx = pts[i].x * DISP_W / COLSIMAGE;
        int sy = pts[i].y;
        if (sx > 0 && sx < DISP_W - 1 && sy > 0 && sy < DISP_H - 1)
            s_ips->draw_point((uint16)sx, (uint16)sy, color);
    }
}

/**
 * @brief 在巡线区域绘制缩放后的十字标记
 * @param bx    俯视图/ROI 列坐标
 * @param by    俯视图/ROI 行坐标
 * @param color RGB565 颜色
 */
static void draw_cross_on_track(int bx, int by, uint16 color)
{
    int sx = bx * DISP_W / COLSIMAGE;
    int sy = by;
    draw_cross(sx, sy, color);
}

void display_init(zf_device_ips200 *ips)
{
    s_ips = ips;
}

/* ====================== 显示模式状态 & helpers ====================== */
volatile int g_display_mode = DISPLAY_MODE_TRACK;

/**
 * @brief BGR 三通道 → RGB565（5-6-5，低字节在前，与 ips200.show_rgb565_image color_mode=0 对应）
 * @param b  蓝通道 0..255
 * @param g  绿通道 0..255
 * @param r  红通道 0..255
 * @return 16 位 RGB565 像素值
 */
static inline uint16_t bgr_to_rgb565(uint8_t b, uint8_t g, uint8_t r)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/**
 * @brief 把 BGR cv::Mat 缩放后转 RGB565，再贴到 IPS200 指定位置
 * @param dst_x  屏幕起点 x
 * @param dst_y  屏幕起点 y
 * @param bgr    输入 BGR cv::Mat（CV_8UC3）；为空 / 通道不对会直接 return
 * @param dst_w  目标宽（像素）
 * @param dst_h  目标高（像素）
 * @param interp cv::resize 插值方式（INTER_LINEAR / INTER_NEAREST 等）
 * @return 无
 * @sample show_bgr_as_rgb565(0, 0, rgb_cut_img, 240, 97, cv::INTER_LINEAR);
 * @note 中转 buffer 在栈外用 std::vector，单次最大 240×97 + 128×128 ≈ 78 KB；
 *       LS2K0300 用户态堆够用，针对 320×240 / ROI 64×64 调参。
 */
static void show_bgr_as_rgb565(int dst_x, int dst_y, const cv::Mat &bgr,
                               int dst_w, int dst_h, int interp)
{
    if (!s_ips) return;
    if (bgr.empty() || bgr.channels() != 3 || dst_w <= 0 || dst_h <= 0) return;

    cv::Mat resized;
    if (bgr.cols == dst_w && bgr.rows == dst_h)
        resized = bgr;
    else
        cv::resize(bgr, resized, cv::Size(dst_w, dst_h), 0, 0, interp);

    std::vector<uint16_t> rgb565((size_t)dst_w * (size_t)dst_h);
    for (int y = 0; y < dst_h; y++)
    {
        const cv::Vec3b *row = resized.ptr<cv::Vec3b>(y);
        uint16_t *dst_row = rgb565.data() + (size_t)y * (size_t)dst_w;
        for (int x = 0; x < dst_w; x++)
        {
            dst_row[x] = bgr_to_rgb565(row[x][0], row[x][1], row[x][2]);
        }
    }
    s_ips->show_rgb565_image((uint16)dst_x, (uint16)dst_y,
                             rgb565.data(),
                             (uint16)dst_w, (uint16)dst_h,
                             (uint16)dst_w, (uint16)dst_h, 0);
}

/**
 * @brief 设置显示模式（与当前不同才清屏并切换）
 * @param mode  DisplayMode 枚举值（0=TRACK，1=TRACK+RING 参数，2=VISION）
 * @return 无
 * @sample display_set_mode(DISPLAY_MODE_VISION);
 * @note 清屏一次防止上一个模式的边线点/HUD/彩图残留像素干扰下一帧
 */
void display_set_mode(int mode)
{
    if (mode < DISPLAY_MODE_TRACK || mode > DISPLAY_MODE_VISION) return;
    if (mode == g_display_mode) return;
    if (s_ips) s_ips->clear();
    g_display_mode = mode;
}

/**
 * @brief 在 TRACK / TRACK+RING 参数 / VISION 三个模式间循环切换
 * @param 无 无
 * @return 无
 * @sample KEY_2 按下边沿 → display_toggle_mode();
 */
void display_toggle_mode(void)
{
    display_set_mode((g_display_mode + 1) % 3);
}

/**
 * @brief 视觉模式主绘制
 * @return 无
 * @sample if (g_display_mode == DISPLAY_MODE_VISION) display_show_vision();
 * @note 布局（240×320 竖屏）：
 *       - y=0   彩图 240×97  （rgb_cut_img 320×130 → 缩放）
 *       - y=92  文字 NOW (x=20)  SAVE (x=124)
 *       - y=108 NOW 96×96 (x=20)  +  SAVE 96×96 (x=124)
 *       - y=215 文字 bW/bH/Rej
 *       - y=242 文字 Label: <name>
 *       - y=265 文字 Prob : <float>
 *       - y=280 文字 KEY3 Snap: OK / NO ROI / …（按键 3 拍照后约 2s）
 *       - y=295 文字 Mode :VISION
 *       任一图为空就在该区域写 "no data"，绝不解引用空 cv::Mat。
 */
void display_show_vision(void)
{
    if (!s_ips) return;

    /* 1) 上半屏：彩图 rgb_cut_img */
    if (!rgb_cut_img.empty() && rgb_cut_img.channels() == 3)
    {
        show_bgr_as_rgb565(0, 0, rgb_cut_img, 240, 97, cv::INTER_LINEAR);
    }
    else
    {
        s_ips->show_string(0, 40, "RGB CUT: no data");
    }

    /* 2) 中部：左 NOW = 当前 ROI；右 SAVE = KEY_3 最近保存 ROI（各 96×96） */
    constexpr int ROI_DISP_W = 96;
    constexpr int ROI_DISP_Y = 108;
    constexpr int LEFT_X     = 20;   /* 左 96 块 [20, 116) */
    constexpr int RIGHT_X    = 124;  /* 右 96 块 [124, 220)，中间留 8px 缝 */

    s_ips->show_string(LEFT_X,  ROI_DISP_Y - 16, "NOW ");
    s_ips->show_string(RIGHT_X, ROI_DISP_Y - 16, "SAVE");

    if (!g_last_roi.empty() && g_last_roi.channels() == 3)
    {
        show_bgr_as_rgb565(LEFT_X, ROI_DISP_Y, g_last_roi, ROI_DISP_W, ROI_DISP_W, cv::INTER_NEAREST);
    }
    else
    {
        s_ips->show_string(LEFT_X, ROI_DISP_Y + 40, "no data ");
    }

    if (!g_last_saved_roi.empty() && g_last_saved_roi.channels() == 3)
    {
        show_bgr_as_rgb565(RIGHT_X, ROI_DISP_Y, g_last_saved_roi, ROI_DISP_W, ROI_DISP_W, cv::INTER_NEAREST);
    }
    else
    {
        s_ips->show_string(RIGHT_X, ROI_DISP_Y + 40, "no data ");
    }

    /* 3) 诊断行（y=215）：block_w / block_h / model_roi_cut 拒因 */
    const char *rej_text = "init   ";
    switch (g_roi_cut_reject_reason)
    {
        case 0:  rej_text = "OK     "; break;
        case 1:  rej_text = "noRed  "; break;
        case 2:  rej_text = "wh     "; break;
        case 3:  rej_text = "quadOOB"; break;
        default: rej_text = "init   "; break;
    }
    s_ips->show_string(0,   215, "bW:");
    s_ips->show_int   (24,  215, block_w, 3);
    s_ips->show_string(60,  215, "bH:");
    s_ips->show_int   (84,  215, block_h, 3);
    s_ips->show_string(120, 215, "Rej:");
    s_ips->show_string(150, 215, rej_text);

    /* 4) 底部文字：标签 + 置信度 + 模式 */
    const auto &labels = vision_labels();
    s_ips->show_string(0, 242, "Label:");
    if (g_last_pred_index >= 0 && g_last_pred_index < (int)labels.size())
        s_ips->show_string(56, 242, labels[g_last_pred_index].c_str());
    else
        s_ips->show_string(56, 242, "----            ");

    s_ips->show_string(0, 265, "Prob :");
    s_ips->show_float(56, 265, g_last_pred_prob, 4, 3);

    if (g_vision_snapshot_ttl > 0)
    {
        const char *snap_text = "KEY3 Snap: ----";
        switch (g_vision_snapshot_result)
        {
            case VSNAP_OK:            snap_text = "KEY3 Snap: OK"; break;
            case VSNAP_FAIL_NO_ROI:   snap_text = "KEY3 Snap: NO ROI"; break;
            case VSNAP_FAIL_DIR:      snap_text = "KEY3 Snap: DIR ERR"; break;
            case VSNAP_FAIL_IO:       snap_text = "KEY3 Snap: SAVE ERR"; break;
            default:                  break;
        }
        s_ips->show_string(0, 280, snap_text);
        g_vision_snapshot_ttl--;
    }

    s_ips->show_string(0, 295, "Mode :VISION");
}

/* -------------------- 小工具：画一组点 -------------------- */
void draw_points(float pts[][2], int num, int y_off, uint16 color)
{
    if (!s_ips || !pts || num <= 0) return;
    for (int i = 0; i < num; i++)
    {
        int x = iroundf_local(pts[i][0]);
        int y = iroundf_local(pts[i][1]) + y_off;
        if (x <= 1 || x >= COLSIMAGE - 2 || y <= 1 || y >= IPS_SCR_H - 2) continue;
        s_ips->draw_point((uint16)x, (uint16)y, color);
    }
}

/* -------------------- 小工具：画起始十字 -------------------- */
void draw_cross(int x, int y, uint16 color)
{
    if (!s_ips) return;
    if (x > 2 && x < COLSIMAGE - 3 && y > 2 && y < IPS_SCR_H - 3)
    {
        s_ips->draw_line((uint16)(x - 2), (uint16)y, (uint16)(x + 2), (uint16)y, color);
        s_ips->draw_line((uint16)x, (uint16)(y - 2), (uint16)x, (uint16)(y + 2), color);
    }
}

/* -------------------- 画文本和起点 -------------------- */
void display_show_overlay(int left_org_num, int right_org_num,
                          int sx_l, int sy_l, int sx_r, int sy_r,
                          float err_img)
{
    if (!s_ips) return;

    s_ips->show_string(0, 250, "O-L:");
    s_ips->show_int(35, 250, left_org_num, 3);
    s_ips->show_string(75, 250, "O-R:");
    s_ips->show_int(110, 250, right_org_num, 3);

    s_ips->show_string(170, 250, "ERR:");
    s_ips->show_float(205, 250, err_img, 4, 1);

    draw_cross(sx_l, sy_l, RGB565_GREEN);
    draw_cross(sx_r, sy_r, RGB565_66CCFF);
}

/**
 * @brief 在 IPS200 上显示未透视二值 ROI 与最终中线
 * @param 无 无
 * @return 无
 * @sample display_show_track();
 * @note   bin_img 320×130 横向缩放到 240 宽；绿/蓝为 ROI 边线，黄为 t_pointsEdge、红为 t_CenterEdge 俯视红点（不连线）
 */
void display_show_track(void)
{
    if (!s_ips) return;
    if (g_display_mode != DISPLAY_MODE_TRACK &&
        g_display_mode != DISPLAY_MODE_TRACK_RING_PARAM) return;
    if (bin_img.empty() || bin_img.rows != ROI_H || bin_img.cols != COLSIMAGE) return;

    const bool ring_param = (g_display_mode == DISPLAY_MODE_TRACK_RING_PARAM);
    const int src_cols = bin_img.cols;

    for (int dy = 0; dy < DISP_H; dy++)
    {
        const uint8_t *row_ptr = bin_img.ptr<uint8_t>(dy);
        for (int dx = 0; dx < DISP_W; dx++)
        {
            int src_x = dx * src_cols / DISP_W;
            if (src_x >= src_cols) src_x = src_cols - 1;
            uint16 color = (row_ptr[src_x] < BIN_THRESH) ? RGB565_BLACK : RGB565_WHITE;
            s_ips->draw_point((uint16)dx, (uint16)dy, color);
        }
    }
    /* 迷宫边线：绿/蓝 = ROI 原图坐标（TRACK+RING 参数档不绘制，减少叠加） */
    if (!ring_param)
    {
        draw_edge_on_track(pointsEdgeLeft,  pointsEdgeLeft_size,  RGB565_GREEN);
        draw_edge_on_track(pointsEdgeRight, pointsEdgeRight_size, RGB565_BLUE);
    }

    /* 阶段 C：俯视图边线（黄，直绘 t_pointsEdge，不做 Reverse_transf） */
    draw_edge_on_track(t_pointsEdgeLeft,  t_pointsEdgeLeft_size,  RGB565_YELLOW);
    draw_edge_on_track(t_pointsEdgeRight, t_pointsEdgeRight_size, RGB565_YELLOW);

    /* 阶段 C：L 角点（紫十字，俯视 → 原图） */
    if (is_t_L_pointLeft_find)
    {
        int rx = 0, ry = 0;
        static General s_geo_l;
        if (s_geo_l.Reverse_transf(rx, ry, t_L_pointLeft.x, t_L_pointLeft.y))
            draw_cross_on_track(rx, ry, RGB565_PURPLE);
    }
    if (is_t_L_pointRight_find)
    {
        int rx = 0, ry = 0;
        static General s_geo_r;
        if (s_geo_r.Reverse_transf(rx, ry, t_L_pointRight.x, t_L_pointRight.y))
            draw_cross_on_track(rx, ry, RGB565_PURPLE);
    }

    /* 俯视中线（红，直绘 t_CenterEdge，仅画点不连线；TRACK+RING 参数档不绘制） */
    if (!ring_param && t_CenterEdge_size >= 1)
    {
        for (int i = 0; i < t_CenterEdge_size; i++)
        {
            int sx = t_CenterEdge[i].x * DISP_W / COLSIMAGE;
            int sy = t_CenterEdge[i].y;
            if (sx <= 0 || sx >= DISP_W - 1 || sy <= 0 || sy >= DISP_H - 1) continue;

            s_ips->draw_point((uint16)sx, (uint16)sy, RGB565_RED);
        }
    }
}

/**
 * @brief 阶段 C HUD：原图/俯视图边线点数与 L 角点标志
 * @return 无
 * @sample display_show_debug_hud_phase_c();
 * @note  显示在 y=200、265，不覆盖巡线 ROI 主区域
 */
void display_show_debug_hud_phase_c(void)
{
    if (!s_ips) return;

    TrackDebugSnapshotC snap{};
    track_debug_fill_phase_c(snap);

    s_ips->show_string(0, 200, "L:");
    s_ips->show_int(18, 200, snap.pl, 3);
    s_ips->show_string(48, 200, "R:");
    s_ips->show_int(66, 200, snap.pr, 3);
    s_ips->show_string(96, 200, "tL:");
    s_ips->show_int(120, 200, snap.tl, 3);
    s_ips->show_string(150, 200, "tR:");
    s_ips->show_int(174, 200, snap.tr, 3);

    s_ips->show_string(0, 265, "LL:");
    s_ips->show_int(24, 265, snap.ll, 1);
    s_ips->show_string(40, 265, "LR:");
    s_ips->show_int(64, 265, snap.lr, 1);
    if (snap.ll)
    {
        s_ips->show_string(80, 265, "id");
        s_ips->show_int(100, 265, snap.lxid, 3);
    }
    if (snap.lr)
    {
        s_ips->show_string(140, 265, "id");
        s_ips->show_int(160, 265, snap.rxid, 3);
    }
}

/**
 * @brief 阶段 D~G HUD：偏差角、归一化、巡线侧、场景与十字/环岛标志
 * @return 无
 * @sample display_show_debug_hud_phase_d();
 * @note  y=250 可读十字/环岛状态；y=280/295 数值 HUD，与阶段 C 错行
 */
void display_show_debug_hud_phase_d(void)
{
    if (!s_ips) return;

    TrackDebugSnapshotD snap{};
    track_debug_fill_phase_d(snap);

    char cross_buf[ELEMENT_STATUS_W + 1];
    char ring_buf[ELEMENT_STATUS_W + 1];
    cross_status_to_text(snap.cross_flag, cross_buf);
    ring_status_to_text(snap.ring_flag, ring_buf);

    s_ips->show_string(0, 250, "Cross:");
    s_ips->show_string(48, 250, cross_buf);
    s_ips->show_string(120, 250, "Ring:");
    s_ips->show_string(160, 250, ring_buf);

    s_ips->show_string(0, 280, "ERR:");
    s_ips->show_float(35, 280, snap.aim_angle, 4, 1);
    s_ips->show_string(85, 280, "OK:");
    s_ips->show_int(108, 280, snap.center_ok, 1);
    s_ips->show_string(125, 280, "cx:");
    s_ips->show_int(145, 280, snap.track_cx, 3);
    s_ips->show_string(175, 280, "cy:");
    s_ips->show_int(195, 280, snap.track_cy, 3);

    s_ips->show_string(0, 295, "Trk:");
    s_ips->show_int(30, 295, snap.track_state, 1);
    s_ips->show_string(48, 295, "Scn:");
    s_ips->show_int(78, 295, snap.scene, 1);
    s_ips->show_string(96, 295, "X:");
    s_ips->show_int(110, 295, snap.cross_flag, 1);
    s_ips->show_string(125, 295, "R:");
    s_ips->show_int(140, 295, snap.ring_flag, 1);
    s_ips->show_string(155, 295, "St:");
    s_ips->show_int(175, 295, snap.straight_l, 1);
    s_ips->show_int(185, 295, snap.straight_r, 1);
    s_ips->show_string(200, 295, "Cv:");
    s_ips->show_int(220, 295, snap.curve_l, 1);
    s_ips->show_int(230, 295, snap.curve_r, 1);
}

/**
 * @brief 显示环岛进环条件调试 HUD（对应 ring.cpp 88~102 的判据项）
 * @param 无 无
 * @return 无
 * @sample if (g_display_mode == DISPLAY_MODE_TRACK_RING_PARAM) display_show_debug_hud_ring_entry();
 * @note  仅在参数档叠加；从 y=150 开始显示原始检测参数（带简短文本标签）：
 *        LLF/LRF/tL/tR/iLS/iRS/Ly/Ry/LC/RC。
 *        不显示 L_size_ok、L_straight_ok、L_y_ok、L_outer_ok 等 *_ok 子项。
 */
void display_show_debug_hud_ring_entry(void)
{
    if (!s_ips) return;
    if (g_display_mode != DISPLAY_MODE_TRACK_RING_PARAM) return;

    RingEntryDebugSnapshot snap{};
    ring_debug_fill_entry(snap);

    const int clear_y0 = 140;
    const int clear_y1 = 199;
    for (int y = clear_y0; y <= clear_y1; ++y)
    {
        for (int x = 0; x < DISP_W; ++x)
        {
            s_ips->draw_point((uint16)x, (uint16)y, RGB565_BLACK);
        }
    }

    const int y0 = 150;
    const int y1 = 166;
    const int y2 = 182;

    s_ips->show_string(0,   y0, "LLF:");
    s_ips->show_int   (28,  y0, snap.is_L_left_found, 1);
    s_ips->show_string(46,  y0, "LRF:");
    s_ips->show_int   (74,  y0, snap.is_L_right_found, 1);
    s_ips->show_string(92,  y0, "tL:");
    s_ips->show_int   (112, y0, snap.t_pointsEdgeLeft_size, 3);
    s_ips->show_string(150, y0, "tR:");
    s_ips->show_int   (170, y0, snap.t_pointsEdgeRight_size, 3);

    s_ips->show_string(0,   y1, "iLS:");
    s_ips->show_int   (28,  y1, snap.is_left_straight, 1);
    s_ips->show_string(46,  y1, "iRS:");
    s_ips->show_int   (74,  y1, snap.is_right_straight, 1);
    s_ips->show_string(92,  y1, "Ly:");
    s_ips->show_int   (112, y1, snap.t_L_pointLeft_y, 4);
    s_ips->show_string(150, y1, "Ry:");
    s_ips->show_int   (170, y1, snap.t_L_pointRight_y, 4);

    s_ips->show_string(0,   y2, "LC:");
    s_ips->show_int   (24,  y2, snap.left_entry_cond, 1);
    s_ips->show_string(50,  y2, "RC:");
    s_ips->show_int   (74,  y2, snap.right_entry_cond, 1);
    s_ips->show_string(100, y2, "Lb:");
    s_ips->show_int   (124, y2, snap.L_break_ok, 1);
    s_ips->show_string(150, y2, "Rb:");
    s_ips->show_int   (174, y2, snap.R_break_ok, 1);
}

/**
 * @brief 红砖避障 HUD：检测量、状态机与阈值（y=140/155/170）
 * @return 无
 * @sample display_show_debug_hud_redbrick(g_brick_avoider);
 * @note  显示在 y=140~170，不覆盖巡线 ROI 与 phase C/D HUD
 */
void display_show_debug_hud_redbrick(const RedBlockAvoider &av)
{
    if (!s_ips) return;

    RedbrickDebugSnapshot snap{};
    redbrick_debug_fill(av, snap);

    s_ips->show_string(0, 140, "St:");
    s_ips->show_int(20, 140, snap.state, 1);
    s_ips->show_string(35, 140, "Det:");
    s_ips->show_int(60, 140, snap.detected, 1);
    s_ips->show_string(75, 140, "Ar:");
    s_ips->show_int(95, 140, snap.area, 5);
    s_ips->show_string(145, 140, "Asp:");
    s_ips->show_float(175, 140, snap.aspect, 3, 1);

    s_ips->show_string(0, 155, "Of:");
    s_ips->show_float(20, 155, snap.offset, 4, 1);
    s_ips->show_string(70, 155, "Frc:");
    s_ips->show_int(95, 155, snap.force, 1);
    s_ips->show_string(110, 155, "botY:");
    s_ips->show_int(145, 155, snap.bot_y, 3);
    s_ips->show_string(175, 155, "cx:");
    s_ips->show_int(195, 155, snap.cx, 3);

    s_ips->show_string(0, 170, "trY:");
    s_ips->show_int(30, 170, snap.trg_y, 3);
    s_ips->show_string(65, 170, "minA:");
    s_ips->show_int(95, 170, snap.min_area, 4);
    s_ips->show_string(135, 170, "Asp:");
    s_ips->show_float(160, 170, snap.asp_min, 3, 1);
    s_ips->show_string(195, 170, "~");
    s_ips->show_float(205, 170, snap.asp_max, 3, 1);
}
