#include "display.hpp"
#include "image.hpp"
#include "zf_common_headfile.hpp"
#include <cmath>
#include <cstdint>

static zf_device_ips200 *s_ips = nullptr;

static constexpr int DISP_W      = 240;
static constexpr int DISP_H      = ROI_H;
static constexpr int BIN_THRESH  = 128;
static constexpr int IPS_SCR_H   = 320;

static inline int iroundf_local(float v)
{
    return (int)(v + (v >= 0.0f ? 0.5f : -0.5f));
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
 * @note   bin_img 320×130 横向缩放到 240 宽；CenterEdge 红色折线叠加
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

    if (CenterEdge_size <= 0) return;

    int prev_sx = -1;
    int prev_sy = -1;
    for (int i = 0; i < CenterEdge_size; i++)
    {
        int sx = CenterEdge[i].x * DISP_W / COLSIMAGE;
        int sy = CenterEdge[i].y;
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
