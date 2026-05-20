#include "display.hpp"
#include "image.hpp"
#include "cross.hpp"
#include "ring.hpp"
#include "zf_common_headfile.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>

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
    case Ring::Left_Ring_Find:      text = "L-FND"; break;
    case Ring::Left_Ring_Begin:     text = "L-BEG"; break;
    case Ring::Left_Ring_In:        text = "L-IN";  break;
    case Ring::Left_Ring_Out:       text = "L-OUT"; break;
    case Ring::Right_Ring_Find:     text = "R-FND"; break;
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
 * @return 无
 * @sample display_show_track();
 * @note   bin_img 320×130 横向缩放到 240 宽；绿/蓝为 ROI 边线，黄为 t_pointsEdge、红为 t_CenterEdge 俯视直绘
 */
void display_show_track(void)
{
    if (!s_ips) return;
    if (bin_img.empty() || bin_img.rows != ROI_H || bin_img.cols != COLSIMAGE) return;

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
    /* 迷宫边线：绿/蓝 = ROI 原图坐标 */
    draw_edge_on_track(pointsEdgeLeft,  pointsEdgeLeft_size,  RGB565_GREEN);
    draw_edge_on_track(pointsEdgeRight, pointsEdgeRight_size, RGB565_BLUE);

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

    /* 俯视中线（红，直绘 t_CenterEdge） */
    if (t_CenterEdge_size >= 2)
    {
        int prev_sx = -1;
        int prev_sy = -1;
        for (int i = 0; i < t_CenterEdge_size; i++)
        {
            int sx = t_CenterEdge[i].x * DISP_W / COLSIMAGE;
            int sy = t_CenterEdge[i].y;
            if (sx <= 0 || sx >= DISP_W - 1 || sy <= 0 || sy >= DISP_H - 1) continue;

            s_ips->draw_point((uint16)sx, (uint16)sy, RGB565_RED);
            if (prev_sx >= 0)
            {
                s_ips->draw_line((uint16)prev_sx, (uint16)prev_sy,
                                 (uint16)sx, (uint16)sy, RGB565_RED);
            }
            prev_sx = sx;
            prev_sy = sy;
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
