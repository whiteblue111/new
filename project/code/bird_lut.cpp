#include "bird_lut.hpp"

namespace {

constexpr int kLutSize = ROI_H * COLSIMAGE;
constexpr int16_t kInvalid = -1;

int16_t s_fwd_x[kLutSize];
int16_t s_fwd_y[kLutSize];
int16_t s_rev_x[kLutSize];
int16_t s_rev_y[kLutSize];
bool    s_ready = false;

static inline int lut_idx(int col, int row)
{
    return row * COLSIMAGE + col;
}

} /* namespace */

/**
 * @brief 构建 forward / reverse 透视 LUT（幂等，首次调用有效）
 * @return 无
 * @sample bird_lut_init();
 */
void bird_lut_init(void)
{
    if (s_ready)
        return;

    const General geo;
    for (int j = 0; j < ROI_H; ++j)
    {
        for (int i = 0; i < COLSIMAGE; ++i)
        {
            const int idx = lut_idx(i, j);
            int bx = 0, by = 0;
            int rx = 0, ry = 0;
            if (geo.transf(bx, by, i, j))
            {
                s_fwd_x[idx] = (int16_t)bx;
                s_fwd_y[idx] = (int16_t)by;
            }
            else
            {
                s_fwd_x[idx] = kInvalid;
                s_fwd_y[idx] = kInvalid;
            }
            if (geo.Reverse_transf(rx, ry, i, j))
            {
                s_rev_x[idx] = (int16_t)rx;
                s_rev_y[idx] = (int16_t)ry;
            }
            else
            {
                s_rev_x[idx] = kInvalid;
                s_rev_y[idx] = kInvalid;
            }
        }
    }
    s_ready = true;
}

/**
 * @brief 原图 ROI 坐标 → 俯视图坐标（查表）
 * @param[out] ri  俯视图列
 * @param[out] rj  俯视图行
 * @param      i   原图列（ROI 局部）
 * @param      j   原图行（ROI 局部 0..ROI_H-1）
 * @return         true 有效映射
 */
bool bird_lut_transf(int &ri, int &rj, int i, int j)
{
    if (!s_ready || i < 0 || i >= COLSIMAGE || j < 0 || j >= ROI_H)
        return false;
    const int idx = lut_idx(i, j);
    if (s_fwd_x[idx] == kInvalid)
        return false;
    ri = s_fwd_x[idx];
    rj = s_fwd_y[idx];
    return true;
}

/**
 * @brief 俯视图坐标 → 原图 ROI 坐标（查表）
 * @param[out] ri  原图列
 * @param[out] rj  原图行
 * @param      i   俯视图列
 * @param      j   俯视图行
 * @return         true 有效映射
 */
bool bird_lut_reverse(int &ri, int &rj, int i, int j)
{
    if (!s_ready || i < 0 || i >= COLSIMAGE || j < 0 || j >= ROI_H)
        return false;
    const int idx = lut_idx(i, j);
    if (s_rev_x[idx] == kInvalid)
        return false;
    ri = s_rev_x[idx];
    rj = s_rev_y[idx];
    return true;
}
