#ifndef FINDLINE_CORE_HPP
#define FINDLINE_CORE_HPP

/**
 * @file findline_core.hpp
 * @brief 迷宫法巡线核心（固定阈值 128，行指针访问）
 */

#include "general.hpp"
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <vector>

namespace findline_core {

static constexpr int kThres = 128;

static inline uint8_t pix_at(const cv::Mat &img, int x, int y)
{
    return img.ptr<uint8_t>(y)[x];
}

/**
 * @brief 左手迷宫法巡线（固定阈值）
 * @param img       二值图
 * @param x,y       起点
 * @param out       输出点列
 * @param out_size  输出点数
 * @param max_len   步数上限
 * @return 无
 */
static inline void lefthand_fixed(cv::Mat &img, int x, int y,
                                  std::vector<POINT> &out, int &out_size,
                                  int max_len = POINTS_MAX_LEN)
{
    static const int df[4][2]      = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    static const int dfl[4][2]     = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    int step = 0, dir = 0, turn = 0;
    const int cols_m1 = img.cols - 1;
    const int rows_m1 = img.rows - 1;

    while (step < max_len && 0 < x && x < cols_m1 && 0 < y && y < rows_m1 && turn < 4)
    {
        const int fx = x + df[dir][0];
        const int fy = y + df[dir][1];
        const int flx = x + dfl[dir][0];
        const int fly = y + dfl[dir][1];

        const uint8_t front_val = pix_at(img, fx, fy);
        const uint8_t fl_val    = pix_at(img, flx, fly);

        if (front_val < kThres)
        {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (fl_val < kThres)
        {
            x = fx;
            y = fy;
            out.emplace_back(x, y);
            step++;
            turn = 0;
        }
        else
        {
            x = flx;
            y = fly;
            dir = (dir + 3) % 4;
            out.emplace_back(x, y);
            step++;
            turn = 0;
        }
    }
    out_size = step;
}

/**
 * @brief 右手迷宫法巡线（固定阈值）
 * @param img       二值图
 * @param x,y       起点
 * @param out       输出点列
 * @param out_size  输出点数
 * @param max_len   步数上限
 * @return 无
 */
static inline void righthand_fixed(cv::Mat &img, int x, int y,
                                   std::vector<POINT> &out, int &out_size,
                                   int max_len = POINTS_MAX_LEN)
{
    static const int df[4][2]  = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    static const int dfr[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    int step = 0, dir = 0, turn = 0;
    const int cols_m1 = img.cols - 1;
    const int rows_m1 = img.rows - 1;

    while (step < max_len && 0 < x && x < cols_m1 && 0 < y && y < rows_m1 && turn < 4)
    {
        const int fx = x + df[dir][0];
        const int fy = y + df[dir][1];
        const int frx = x + dfr[dir][0];
        const int fry = y + dfr[dir][1];

        const uint8_t front_val = pix_at(img, fx, fy);
        const uint8_t fr_val    = pix_at(img, frx, fry);

        if (front_val < kThres)
        {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (fr_val < kThres)
        {
            x = fx;
            y = fy;
            out.emplace_back(x, y);
            step++;
            turn = 0;
        }
        else
        {
            x = frx;
            y = fry;
            dir = (dir + 1) % 4;
            out.emplace_back(x, y);
            step++;
            turn = 0;
        }
    }
    out_size = step;
}

} /* namespace findline_core */

#endif /* FINDLINE_CORE_HPP */
