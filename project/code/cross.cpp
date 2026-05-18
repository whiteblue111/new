/**
 * @file cross.cpp
 * @brief 十字路口处理类实现（移植自 temp_repo/track/basic/cross.cpp）
 *
 * 三段式状态机：Cross_None → Cross_Begin → Cross_Out → Cross_None
 * 主要适配：阈值调整到 320x120 ROI（y ∈ [40,160)），
 *           far_left_y0 / far_right_y0 初值改为 ROI_TOP + 30。
 */

#include "cross.hpp"
#include <cmath>
#include <cstdio>


Cross::Cross()
{
    both_L_find_counter = 0;
}


/**
 * @brief 十字识别（仅状态转移，不修改边线）
 * @sample 见 cross.hpp
 * @note   进入条件需连续两帧满足下列三种之一：
 *           1) 左右双 L 角点均找到，且角点未到边线末端 5 点以内
 *           2) 仅左 L 找到，且右边线 < 0.3/SAMPLE_DIST 点（右侧丢线）
 *           3) 仅右 L 找到，且左边线 < 0.3/SAMPLE_DIST 点（左侧丢线）
 */
void Cross::Cross_Check(bool is_L_left_found, bool is_L_right_found,
                        cv::Mat /*imgBinary*/,
                        cv::Point /*t_L_pointLeft*/, cv::Point /*t_L_pointRight*/,
                        int t_L_pointLeft_id, int t_L_pointRight_id,
                        int t_pointsEdgeLeft_size, int t_pointsEdgeRight_size)
{
    bool cond =
        (is_L_left_found && is_L_right_found
            && t_L_pointLeft_id  < t_pointsEdgeLeft_size  - 5
            && t_L_pointRight_id < t_pointsEdgeRight_size - 5)
     || (is_L_left_found  && t_L_pointLeft_id  < 0.70 / SAMPLE_DIST
            && t_pointsEdgeRight_size < 0.30 / SAMPLE_DIST)
     || (is_L_right_found && t_L_pointRight_id < 0.70 / SAMPLE_DIST
            && t_pointsEdgeLeft_size  < 0.30 / SAMPLE_DIST);

    if (flag_cross == Cross_None && cond)
    {
        Cross_counter++;
    }
    if (flag_cross == Cross_None && Cross_counter > 1 && cond)
    {
        flag_cross    = Cross_Begin;
        Cross_counter = 0;
    }
}


/**
 * @brief 十字执行（Cross_Begin 截断边线、Cross_Out 巡远线）
 * @note  - Cross_Begin: 把 t_pointsEdge 截断到 L 角点下标
 *        - Cross_Out:   调用 cross_find_farline 巡新边线，把远端边线接到 t_pointsEdge
 *        - 退出: Cross_Out 下连续多帧空线 + 短暂恢复双线时清状态
 */
void Cross::Cross_Run(std::vector<POINT> &t_pointsEdgeLeft,
                      std::vector<POINT> &t_pointsEdgeRight,
                      cv::Mat img,
                      bool is_L_left_found, bool is_L_right_found,
                      int t_L_pointLeft_id, int t_L_pointRight_id,
                      int &t_pointsEdgeLeft_size, int &t_pointsEdgeRight_size)
{
    Cross_counter++;

    /* ====== 状态机：Out → None ====== */
    if (flag_cross == Cross_Out
        && t_pointsEdgeLeft_size == 0 && t_pointsEdgeRight_size == 0)
    {
        no_line_counter++;
    }
    else if (flag_cross == Cross_Out
        && t_pointsEdgeLeft_size  > 5
        && t_pointsEdgeRight_size > 5
        && no_line_counter > 3)
    {
        no_line_counter = 0;
        flag_cross      = Cross_None;
        return;
    }
    /* ====== 状态机：Begin → Out ====== */
    else if (flag_cross == Cross_Begin && Cross_counter > 5 &&
             ((!is_L_left_found && !is_L_right_found)
              || (is_L_left_found  && t_L_pointLeft_id  < 0.26 / SAMPLE_DIST)
              || (is_L_right_found && t_L_pointRight_id < 0.26 / SAMPLE_DIST)
              || (t_pointsEdgeLeft_size  < 0.10 / SAMPLE_DIST
               && t_pointsEdgeRight_size < 0.10 / SAMPLE_DIST)
              || (Cross_counter > 30)))
    {
        flag_cross      = Cross_Out;
        L_left_found    = false;
        L_right_found   = false;
        no_line_counter = 0;
    }

    /* ====== 状态机：Begin 期间——截断边线到 L 角点 ====== */
    if (flag_cross == Cross_Begin)
    {
        if (is_L_left_found)
        {
            t_pointsEdgeLeft_size  = t_L_pointLeft_id;
            t_pointsEdgeRight_size = t_L_pointLeft_id;
        }
        if (is_L_right_found)
        {
            t_pointsEdgeLeft_size  = t_L_pointRight_id;
            t_pointsEdgeRight_size = t_L_pointRight_id;
        }
    }

    /* ====== 状态机：Out 期间——巡远线，把远端边线接回 t_pointsEdge ====== */
    if (flag_cross == Cross_Out)
    {
        cross_find_farline(img, is_L_left_found, is_L_right_found,
                           t_pointsEdgeLeft, t_pointsEdgeRight,
                           t_L_pointLeft_id, t_L_pointRight_id);
        t_pointsEdgeLeft.clear();
        t_pointsEdgeRight.clear();

        for (int i = far_t_L_pointLeft_id; i < far_t_pointsEdgeLeft_size; i++)
        {
            t_pointsEdgeLeft.emplace_back(far_t_pointsEdgeLeft[i].x,
                                          far_t_pointsEdgeLeft[i].y);
        }
        t_pointsEdgeLeft_size = (int)t_pointsEdgeLeft.size();

        for (int i = far_t_L_pointRight_id; i < far_t_pointsEdgeRight_size; i++)
        {
            t_pointsEdgeRight.emplace_back(far_t_pointsEdgeRight[i].x,
                                           far_t_pointsEdgeRight[i].y);
        }
        t_pointsEdgeRight_size = (int)t_pointsEdgeRight.size();
    }
}


/**
 * @brief 远线巡线主流程
 * @note 通过 L 角点反变换到原图坐标，外推一个起点（x±18, y-5），
 *       自该点沿列向上找白→黑跳变 → 跑迷宫法巡新边线 →
 *       perspective + blur + resample + angle + NMS → find_corners
 */
void Cross::cross_find_farline(cv::Mat &img,
                               bool is_L_left_found, bool is_L_right_found,
                               std::vector<POINT> t_pointsEdgeLeft,
                               std::vector<POINT> t_pointsEdgeRight,
                               int t_L_pointLeft_id, int t_L_pointRight_id)
{
    far_pointsEdgeLeft.clear();      far_pointsEdgeLeft_size      = 0;
    far_pointsEdgeRight.clear();     far_pointsEdgeRight_size     = 0;
    far_t_pointsEdgeLeft.clear();    far_t_pointsEdgeLeft_size    = 0;
    far_t_pointsEdgeRight.clear();   far_t_pointsEdgeRight_size   = 0;
    far_b_t_pointsEdgeLeft.clear();  far_b_t_pointsEdgeLeft_size  = 0;
    far_b_t_pointsEdgeRight.clear(); far_b_t_pointsEdgeRight_size = 0;
    far_s_b_t_pointsEdgeLeft.clear();  far_s_b_t_pointsEdgeLeft_size  = 0;
    far_s_b_t_pointsEdgeRight.clear(); far_s_b_t_pointsEdgeRight_size = 0;
    far_a_t_pointsEdgeLeft.clear();    far_a_t_pointsEdgeLeft_size    = 0;
    far_a_t_pointsEdgeRight.clear();   far_a_t_pointsEdgeRight_size   = 0;
    far_n_a_t_pointsEdgeLeft.clear();  far_n_a_t_pointsEdgeLeft_size  = 0;
    far_n_a_t_pointsEdgeRight.clear(); far_n_a_t_pointsEdgeRight_size = 0;
    is_far_t_L_pointLeft_find  = false;
    is_far_t_L_pointRight_find = false;
    far_t_L_pointLeft_id  = 0;
    far_t_L_pointRight_id = 0;

    if (is_L_left_found
        && t_L_pointLeft_id >= 0
        && t_L_pointLeft_id < (int)t_pointsEdgeLeft.size())
    {
        general.Reverse_transf(far_left_x0, far_left_y0,
                               t_pointsEdgeLeft[t_L_pointLeft_id].x,
                               t_pointsEdgeLeft[t_L_pointLeft_id].y);
        far_left_y0 -= 5;
        far_left_x0 -= 18;
        L_left_found = true;
    }
    if (is_L_right_found
        && t_L_pointRight_id >= 0
        && t_L_pointRight_id < (int)t_pointsEdgeRight.size())
    {
        general.Reverse_transf(far_right_x0, far_right_y0,
                               t_pointsEdgeRight[t_L_pointRight_id].x,
                               t_pointsEdgeRight[t_L_pointRight_id].y);
        far_right_y0 -= 5;
        far_right_x0 += 18;
        L_right_found = true;
    }
    if (far_left_x0  < 0 || far_left_x0  >= COLSIMAGE
     || far_left_y0  < 0 || far_left_y0  >= ROI_H
     || far_right_x0 < 0 || far_right_x0 >= COLSIMAGE
     || far_right_y0 < 0 || far_right_y0 >= ROI_H)
    {
        return;
    }

    /* 从外推起点沿列向上找白→黑跳变作为巡线起点 */
    int y0 = far_left_y0;
    int y1 = far_right_y0;
    for (; y0 > 0; y0--)
    {
        if (img.at<unsigned char>(y0 - 1, far_left_x0) < 128
            && img.at<unsigned char>(y0,     far_left_x0) > 128)
            break;
    }
    for (; y1 > 0; y1--)
    {
        if (img.at<unsigned char>(y1 - 1, far_right_x0) < 128
            && img.at<unsigned char>(y1,     far_right_x0) > 128)
            break;
    }
    /* 巡线起点应落在 ROI 有效行内（局部 y > 0，对应原全图 y > ROI_TOP） */
    if (y0 > 0 && L_left_found)
    {
        findline_lefthand_adaptive(img, block_size, clip_value, far_left_x0, y0,
                                   far_pointsEdgeLeft, far_pointsEdgeLeft_size);
    }
    else
    {
        far_pointsEdgeLeft_size = 0;
    }
    if (y1 > 0 && L_right_found)
    {
        findline_righthand_adaptive(img, block_size, clip_value, far_right_x0, y1,
                                    far_pointsEdgeRight, far_pointsEdgeRight_size);
    }
    else
    {
        far_pointsEdgeRight_size = 0;
    }
    /* 左右两条远线穿插了 → 视为右线无效 */
    if (far_pointsEdgeLeft_size > 0 && far_pointsEdgeRight_size > 0)
    {
        if (far_pointsEdgeRight[far_pointsEdgeRight_size - 1].x
            < far_pointsEdgeLeft[0].x)
        {
            far_pointsEdgeRight_size = 0;
        }
    }

    /* ---- 透视变换 ---- */
    for (int i = 0; i < far_pointsEdgeLeft_size; i++)
    {
        int a, b;
        if (general.transf(a, b, far_pointsEdgeLeft[i].x, far_pointsEdgeLeft[i].y))
            far_t_pointsEdgeLeft.emplace_back(a, b);
        else
            break;
    }
    far_t_pointsEdgeLeft_size = (int)far_t_pointsEdgeLeft.size();
    for (int i = 0; i < far_pointsEdgeRight_size; i++)
    {
        int a, b;
        if (general.transf(a, b, far_pointsEdgeRight[i].x, far_pointsEdgeRight[i].y))
            far_t_pointsEdgeRight.emplace_back(a, b);
        else
            break;
    }
    far_t_pointsEdgeRight_size = (int)far_t_pointsEdgeRight.size();

    /* ---- 滤波 ---- */
    blur_points(0, 11);
    blur_points(1, 11);
    far_b_t_pointsEdgeLeft_size  = far_t_pointsEdgeLeft_size;
    far_b_t_pointsEdgeRight_size = far_t_pointsEdgeRight_size;

    /* ---- 等距采样 ---- */
    resample_points(far_b_t_pointsEdgeLeft,  far_b_t_pointsEdgeLeft_size,
                    far_s_b_t_pointsEdgeLeft, far_s_b_t_pointsEdgeLeft_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));
    resample_points(far_b_t_pointsEdgeRight, far_b_t_pointsEdgeRight_size,
                    far_s_b_t_pointsEdgeRight, far_s_b_t_pointsEdgeRight_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));

    /* ---- 角度 ---- */
    local_angle_points(far_s_b_t_pointsEdgeLeft, far_s_b_t_pointsEdgeLeft_size,
                       far_a_t_pointsEdgeLeft, 11);
    far_a_t_pointsEdgeLeft_size = (int)far_a_t_pointsEdgeLeft.size();
    local_angle_points(far_s_b_t_pointsEdgeRight, far_s_b_t_pointsEdgeRight_size,
                       far_a_t_pointsEdgeRight, 11);
    far_a_t_pointsEdgeRight_size = (int)far_a_t_pointsEdgeRight.size();

    /* ---- NMS ---- */
    nms_angle(far_a_t_pointsEdgeLeft, far_a_t_pointsEdgeLeft_size,
              far_n_a_t_pointsEdgeLeft, 22);
    far_n_a_t_pointsEdgeLeft_size = (int)far_n_a_t_pointsEdgeLeft.size();
    nms_angle(far_a_t_pointsEdgeRight, far_a_t_pointsEdgeRight_size,
              far_n_a_t_pointsEdgeRight, 22);
    far_n_a_t_pointsEdgeRight_size = (int)far_n_a_t_pointsEdgeRight.size();

    /* 把等距采样后的点序列同步写回 far_t_pointsEdge */
    far_t_pointsEdgeLeft.clear();   far_t_pointsEdgeLeft_size  = 0;
    far_t_pointsEdgeRight.clear();  far_t_pointsEdgeRight_size = 0;
    for (int i = 0; i < far_s_b_t_pointsEdgeLeft_size; i++)
    {
        far_t_pointsEdgeLeft.emplace_back(far_s_b_t_pointsEdgeLeft[i].x,
                                          far_s_b_t_pointsEdgeLeft[i].y);
        far_t_pointsEdgeLeft_size++;
    }
    for (int i = 0; i < far_s_b_t_pointsEdgeRight_size; i++)
    {
        far_t_pointsEdgeRight.emplace_back(far_s_b_t_pointsEdgeRight[i].x,
                                           far_s_b_t_pointsEdgeRight[i].y);
        far_t_pointsEdgeRight_size++;
    }
    far_t_L_pointLeft_id  = far_t_pointsEdgeLeft_size;
    far_t_L_pointRight_id = far_a_t_pointsEdgeRight_size;
    find_corners();
}


/**
 * @brief 计算两点斜率（用于补线及拐点确认）
 * @param begin      起点（含）
 * @param end        终点（不含）
 * @param pointsEdge 点序列
 * @return           最小二乘斜率（除数为零时返回上一次结果）
 */
float Cross::Slope_Calculate(int begin, int end, std::vector<POINT> &pointsEdge)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    float result = 0;
    static float resultlast = 0;

    for (int i = begin; i < end; i++)
    {
        xsum  += pointsEdge[i].x;
        ysum  += pointsEdge[i].y;
        xysum += pointsEdge[i].x * pointsEdge[i].y;
        x2sum += pointsEdge[i].x * pointsEdge[i].x;
    }
    if ((end - begin) * x2sum - xsum * xsum)
    {
        result = ((end - begin) * xysum - xsum * ysum)
               / ((end - begin) * x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}


/**
 * @brief 区间斜率 + 截距
 * @param start         起点（含）
 * @param end           终点（不含）
 * @param pointsEdge    点序列
 * @param[out] slope_rate 斜率
 * @param[out] intercept  截距
 */
void Cross::calculate_s_i(int start, int end, std::vector<POINT> &pointsEdge,
                          float &slope_rate, float &intercept)
{
    int   num   = 0;
    int   xsum  = 0;
    int   ysum  = 0;
    float x_avg = 0;
    float y_avg = 0;

    for (int i = start; i < end; i++)
    {
        xsum += pointsEdge[i].x;
        ysum += pointsEdge[i].y;
        num++;
    }
    if (num)
    {
        x_avg = (float)xsum / num;
        y_avg = (float)ysum / num;
    }
    slope_rate = Slope_Calculate(start, end, pointsEdge);
    intercept  = y_avg - slope_rate * x_avg;
}


/* ============= 私有：迷宫法 / 滤波 / 采样 / 角度 / NMS / 找拐点 ============= */

void Cross::findline_lefthand_adaptive(cv::Mat &img, int block_size, int clip_value,
                                       int x, int y,
                                       std::vector<POINT> &pointsEdgeLeft,
                                       int &pointsEdgeLeft_size)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < POINTS_MAX_LEN) && half < x && x < (img.cols - half - 1) &&
           half < y && y < (img.rows - half - 1) && turn < 4)
    {
        int local_thres = 0;
        for (int dy = -half; dy < half; dy++)
            for (int dx = -half; dx <= half; dx++)
                local_thres += img.at<unsigned char>(y + dy, x + dx);
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value     = img.at<unsigned char>(y + dir_front[dir][1],
                                                    x + dir_front[dir][0]);
        int frontleft_value = img.at<unsigned char>(y + dir_frontleft[dir][1],
                                                    x + dir_frontleft[dir][0]);

        if (front_value < local_thres)
        {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (frontleft_value < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pointsEdgeLeft.emplace_back(x, y);
            step++;
            turn = 0;
        }
        else
        {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pointsEdgeLeft.emplace_back(x, y);
            step++;
            turn = 0;
        }
    }
    pointsEdgeLeft_size = step;
}


void Cross::findline_righthand_adaptive(cv::Mat &img, int block_size, int clip_value,
                                        int x, int y,
                                        std::vector<POINT> &pointsEdgeRight,
                                        int &pointsEdgeRight_size)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < POINTS_MAX_LEN) && 0 < x && x < (img.cols - 1) &&
           0 < y && y < (img.rows - 1) && turn < 4)
    {
        int local_thres = 0;
        for (int dy = -half; dy < half; dy++)
            for (int dx = -half; dx <= half; dx++)
                local_thres += img.at<unsigned char>(y + dy, x + dx);
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value      = img.at<unsigned char>(y + dir_front[dir][1],
                                                     x + dir_front[dir][0]);
        int frontright_value = img.at<unsigned char>(y + dir_frontright[dir][1],
                                                     x + dir_frontright[dir][0]);

        if (front_value < local_thres)
        {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (frontright_value < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pointsEdgeRight.emplace_back(x, y);
            step++;
            turn = 0;
        }
        else
        {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pointsEdgeRight.emplace_back(x, y);
            step++;
            turn = 0;
        }
    }
    pointsEdgeRight_size = step;
}


void Cross::blur_points(int side, int kernel)
{
    int half = kernel / 2;
    if (side == 0)
    {
        for (int i = 0; i < far_t_pointsEdgeLeft_size; i++)
        {
            far_b_t_pointsEdgeLeft.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                int idx = general.clip(i + j, 0, far_t_pointsEdgeLeft_size - 1);
                far_b_t_pointsEdgeLeft[i].x +=
                    far_t_pointsEdgeLeft[idx].x * (half + 1 - std::abs(j));
                far_b_t_pointsEdgeLeft[i].y +=
                    far_t_pointsEdgeLeft[idx].y * (half + 1 - std::abs(j));
            }
            far_b_t_pointsEdgeLeft[i].x /= (2 * half + 2) * (half + 1) / 2;
            far_b_t_pointsEdgeLeft[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    else
    {
        for (int i = 0; i < far_t_pointsEdgeRight_size; i++)
        {
            far_b_t_pointsEdgeRight.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                int idx = general.clip(i + j, 0, far_t_pointsEdgeRight_size - 1);
                far_b_t_pointsEdgeRight[i].x +=
                    far_t_pointsEdgeRight[idx].x * (half + 1 - std::abs(j));
                far_b_t_pointsEdgeRight[i].y +=
                    far_t_pointsEdgeRight[idx].y * (half + 1 - std::abs(j));
            }
            far_b_t_pointsEdgeRight[i].x /= (2 * half + 2) * (half + 1) / 2;
            far_b_t_pointsEdgeRight[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    far_b_t_pointsEdgeLeft_size  = (int)far_b_t_pointsEdgeLeft.size();
    far_b_t_pointsEdgeRight_size = (int)far_b_t_pointsEdgeRight.size();
}


void Cross::resample_points(std::vector<POINT> &in, int in_size,
                            std::vector<POINT> &out, int &out_size, float dist)
{
    int remain = 0;
    int len    = 0;
    for (int i = 0; i < in_size - 1; i++)
    {
        float x0 = in[i].x;
        float y0 = in[i].y;
        float dx = in[i + 1].x - x0;
        float dy = in[i + 1].y - y0;
        float dn = std::sqrt(dx * dx + dy * dy);
        if (dn < 1e-6f) continue;
        dx /= dn;
        dy /= dn;
        while (remain < dn)
        {
            x0 += dx * remain;
            y0 += dy * remain;
            out.emplace_back((int)x0, (int)y0);
            len++;
            dn    -= remain;
            remain = (int)dist;
        }
        remain -= (int)dn;
    }
    out_size = len;
}


void Cross::find_corners()
{
    if (far_t_pointsEdgeLeft_size > 20)
    {
        for (int i = 0; i < far_n_a_t_pointsEdgeLeft_size; i++)
        {
            int im1 = general.clip(i - 2, 0, far_n_a_t_pointsEdgeLeft_size - 1);
            int ip1 = general.clip(i + 2, 0, far_n_a_t_pointsEdgeLeft_size - 1);
            float conf = std::fabs(far_n_a_t_pointsEdgeLeft[i].angle)
                       - (std::fabs(far_n_a_t_pointsEdgeLeft[im1].angle
                                   + std::fabs(far_n_a_t_pointsEdgeLeft[ip1].angle))) / 2;
            conf = conf * 180.0f / PI;
            conf = std::fabs(conf);

            if (!is_far_t_L_pointLeft_find && Lconf_Min < conf && conf < Lconf_Max)
            {
                for (int j = 0; j < far_t_pointsEdgeLeft_size; j++)
                {
                    if (far_t_pointsEdgeLeft[j].x == far_n_a_t_pointsEdgeLeft[i].x
                     && far_t_pointsEdgeLeft[j].y == far_n_a_t_pointsEdgeLeft[i].y)
                    {
                        far_t_L_pointLeft_id      = j;
                        is_far_t_L_pointLeft_find = true;
                        break;
                    }
                }
            }
            else if (is_far_t_L_pointLeft_find) break;
        }
    }
    if (far_t_pointsEdgeRight_size > 20)
    {
        for (int i = 0; i < far_n_a_t_pointsEdgeRight_size; i++)
        {
            int im1 = general.clip(i - 2, 0, far_n_a_t_pointsEdgeRight_size - 1);
            int ip1 = general.clip(i + 2, 0, far_n_a_t_pointsEdgeRight_size - 1);
            float conf = std::fabs(far_n_a_t_pointsEdgeRight[i].angle)
                       - (std::fabs(far_n_a_t_pointsEdgeRight[im1].angle
                                   + std::fabs(far_n_a_t_pointsEdgeRight[ip1].angle))) / 2;
            conf = conf * 180.0f / PI;
            conf = std::fabs(conf);

            if (!is_far_t_L_pointRight_find && Lconf_Min < conf && conf < Lconf_Max)
            {
                for (int j = 0; j < far_t_pointsEdgeRight_size; j++)
                {
                    if (far_t_pointsEdgeRight[j].x == far_n_a_t_pointsEdgeRight[i].x
                     && far_t_pointsEdgeRight[j].y == far_n_a_t_pointsEdgeRight[i].y)
                    {
                        far_t_L_pointRight_id      = j;
                        is_far_t_L_pointRight_find = true;
                        break;
                    }
                }
            }
            else if (is_far_t_L_pointRight_find) break;
        }
    }
}


void Cross::local_angle_points(std::vector<POINT> pointsEdgeIn, int size,
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
        float dx1 = pointsEdgeIn[i].x - pointsEdgeIn[general.clip(i - dist, 0, size - 1)].x;
        float dy1 = pointsEdgeIn[i].y - pointsEdgeIn[general.clip(i - dist, 0, size - 1)].y;
        float dn1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        float dx2 = pointsEdgeIn[general.clip(i + dist, 0, size - 1)].x - pointsEdgeIn[i].x;
        float dy2 = pointsEdgeIn[general.clip(i + dist, 0, size - 1)].y - pointsEdgeIn[i].y;
        float dn2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        if (dn1 < 1e-6f || dn2 < 1e-6f) { pointsEdgeOut[i].angle = 0; continue; }
        float c1 = dx1 / dn1, s1 = dy1 / dn1;
        float c2 = dx2 / dn2, s2 = dy2 / dn2;
        pointsEdgeOut[i].angle = std::atan2(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}


void Cross::nms_angle(std::vector<POINT> &in, int in_size,
                      std::vector<POINT> &out, int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < in_size; i++)
    {
        out.emplace_back(in[i].x, in[i].y);
        out[i].angle = in[i].angle;
        for (int j = -half; j <= half; j++)
        {
            if (std::fabs(in[general.clip(i + j, 0, in_size - 1)].angle)
                > std::fabs(out[i].angle))
            {
                out[i].angle = 0;
                break;
            }
        }
    }
}
