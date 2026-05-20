/**
 * @file ring.cpp
 * @brief 环岛处理类实现
 *
 * 左环：Find(巡右) → Begin(巡左) → In(巡左) → Out(巡右) → None
 * 右环：镜像；丢复线边沿见 update_edge_regain；出环后 ring_cooldown 帧冷却。
 */

#include "ring.hpp"
#include <cmath>
#include <cstdio>


Ring::Ring() {}


void Ring::reset()
{
    flag_ring = Ring_None;

    state_locking      = false;
    right_regain_count = 0;
    left_regain_count  = 0;
    right_edge_phase   = EDGE_OK;
    left_edge_phase    = EDGE_OK;

    L_left_down_found  = false;
    L_left_mid_found   = false;
    L_left_up_found    = false;
    L_right_down_found = false;
    L_right_mid_found  = false;
    L_right_up_found   = false;
    L_id_left_down  = 666;
    L_id_left_mid   = 666;
    L_id_left_up    = 666;
    L_id_right_down = 666;
    L_id_right_mid  = 666;
    L_id_right_up   = 666;
    pointsLeft.clear();
    pointsRight.clear();
    pointsMid.clear();
    pointsLeft_size  = 0;
    pointsRight_size = 0;
    pointsMid_size   = 0;
}


void Ring::update_edge_regain(int size, int &regain_count, int &phase)
{
    if (size < LOST_LINE)
    {
        if (phase == EDGE_OK)
            phase = EDGE_LOST;
    }
    else if (size >= REGAIN_LINE)
    {
        if (phase == EDGE_LOST)
        {
            regain_count++;
            phase = EDGE_OK;
        }
    }
}


void Ring::Ring_Check(cv::Mat &imgBinary,
                      bool is_left_straight, bool is_right_straight,
                      bool /*L_left_found_out*/, bool /*L_right_found_out*/,
                      int t_pointsEdgeLeft_size, int t_pointsEdgeRight_size,
                      bool is_L_left_found, bool is_L_right_found,
                      int /*t_L_pointLeft_id*/, int /*t_L_pointRight_id*/,
                      std::vector<POINT> & /*t_pointsEdgeLeft*/,
                      std::vector<POINT> & /*t_pointsEdgeRight*/)
{
    L_left_down_found  = false;
    L_left_mid_found   = false;
    L_left_up_found    = false;
    L_right_down_found = false;
    L_right_mid_found  = false;
    L_right_up_found   = false;
    L_id_left_down  = 666;
    L_id_left_mid   = 666;
    L_id_left_up    = 666;
    L_id_right_down = 666;
    L_id_right_mid  = 666;
    L_id_right_up   = 666;
    pointsLeft.clear();
    pointsRight.clear();
    pointsMid.clear();
    pointsLeft_size  = 0;
    pointsRight_size = 0;
    pointsMid_size   = 0;

    if (t_pointsEdgeLeft_size == 0 && t_pointsEdgeRight_size == 0)
        return;

    ring_find_line(imgBinary, rowstart);

    if (flag_ring == Ring_None)
    {
        if (ring_cooldown > 0)
        {
            ring_cooldown--;
            return;
        }

        find_left_down();
        if (L_left_down_found
            && t_pointsEdgeLeft_size < L_SMALL
            && t_pointsEdgeRight_size > R_LARGE
            && is_right_straight && !is_left_straight)
        {
            flag_ring = Left_Ring_Find;
            right_regain_count = 0;
            right_edge_phase   = EDGE_OK;
            state_locking      = false;
        }
        else
        {
            find_right_down();
            if (L_right_down_found
                && t_pointsEdgeRight_size < L_SMALL
                && t_pointsEdgeLeft_size > R_LARGE
                && is_left_straight && !is_right_straight)
            {
                flag_ring = Right_Ring_Find;
                left_regain_count = 0;
                left_edge_phase   = EDGE_OK;
                state_locking     = false;
            }
        }
        return;
    }

    /* ===== 左圆环 ===== */
    if (flag_ring == Left_Ring_Find)
    {
        cv::Point mid = find_left_mid(115);
        if (L_left_mid_found && mid.y > Y_FIND_TO_BEGIN && !state_locking)
        {
            flag_ring       = Left_Ring_Begin;
            state_locking   = true;
            right_regain_count = 0;
            right_edge_phase   = EDGE_OK;
        }
    }
    else if (flag_ring == Left_Ring_Begin)
    {
        if (is_L_left_found)
            state_locking = false;

        update_edge_regain(t_pointsEdgeRight_size, right_regain_count, right_edge_phase);

        if (right_regain_count >= 1 && !state_locking)
        {
            flag_ring        = Left_Ring_In;
            state_locking    = true;
        }
    }
    else if (flag_ring == Left_Ring_In)
    {
        if (t_pointsEdgeRight_size < LOST_LINE)
            state_locking = false;

        update_edge_regain(t_pointsEdgeRight_size, right_regain_count, right_edge_phase);

        if (right_regain_count >= 2 && !state_locking)
        {
            flag_ring     = Left_Ring_Out;
            state_locking = true;
            left_regain_count = 0;
            left_edge_phase   = EDGE_OK;
        }
    }
    else if (flag_ring == Left_Ring_Out)
    {
        update_edge_regain(t_pointsEdgeLeft_size, left_regain_count, left_edge_phase);

        if (left_regain_count >= 1)
        {
            reset();
            ring_cooldown = RING_COOLDOWN_MAX;
        }
    }

    /* ===== 右圆环（镜像） ===== */
    if (flag_ring == Right_Ring_Find)
    {
        cv::Point mid = find_right_mid(115);
        if (L_right_mid_found && mid.y > Y_FIND_TO_BEGIN && !state_locking)
        {
            flag_ring      = Right_Ring_Begin;
            state_locking  = true;
            left_regain_count = 0;
            left_edge_phase   = EDGE_OK;
        }
    }
    else if (flag_ring == Right_Ring_Begin)
    {
        if (is_L_right_found)
            state_locking = false;

        update_edge_regain(t_pointsEdgeLeft_size, left_regain_count, left_edge_phase);

        if (left_regain_count >= 1 && !state_locking)
        {
            flag_ring     = Right_Ring_In;
            state_locking = true;
        }
    }
    else if (flag_ring == Right_Ring_In)
    {
        if (t_pointsEdgeLeft_size < LOST_LINE)
            state_locking = false;

        update_edge_regain(t_pointsEdgeLeft_size, left_regain_count, left_edge_phase);

        if (left_regain_count >= 2 && !state_locking)
        {
            flag_ring      = Right_Ring_Out;
            state_locking  = true;
            right_regain_count = 0;
            right_edge_phase   = EDGE_OK;
        }
    }
    else if (flag_ring == Right_Ring_Out)
    {
        update_edge_regain(t_pointsEdgeRight_size, right_regain_count, right_edge_phase);

        if (right_regain_count >= 1)
        {
            reset();
            ring_cooldown = RING_COOLDOWN_MAX;
        }
    }
}


void Ring::Ring_Run(std::vector<POINT> &t_pointsEdgeLeft,
                    std::vector<POINT> &t_pointsEdgeRight,
                    int &t_pointsEdgeLeft_size, int &t_pointsEdgeRight_size,
                    bool /*is_L_left_found*/, bool /*is_L_right_found*/,
                    int /*t_L_pointLeft_id*/, int /*t_L_pointRight_id*/,
                    cv::Mat /*imgBinary*/)
{
    if (flag_ring == Ring_None)
        return;

    /* 左环：Find/Out 巡右；Begin/In 巡左 */
    if (flag_ring == Left_Ring_Find || flag_ring == Left_Ring_Out)
    {
        t_pointsEdgeLeft.clear();
        t_pointsEdgeLeft_size = 0;
    }
    else if (flag_ring == Left_Ring_Begin || flag_ring == Left_Ring_In)
    {
        t_pointsEdgeRight.clear();
        t_pointsEdgeRight_size = 0;
    }

    /* 右环：Find/Out 巡左；Begin/In 巡右 */
    if (flag_ring == Right_Ring_Find || flag_ring == Right_Ring_Out)
    {
        t_pointsEdgeRight.clear();
        t_pointsEdgeRight_size = 0;
    }
    else if (flag_ring == Right_Ring_Begin || flag_ring == Right_Ring_In)
    {
        t_pointsEdgeLeft.clear();
        t_pointsEdgeLeft_size = 0;
    }
}


void Ring::ring_find_line(cv::Mat &img, int /*y_start*/)
{
    int step = 0;
    for (int row = rowstart; row > rowup; row--)
    {
        int base_x = (step == 0) ? (COLSIMAGE / 2) : pointsMid[step - 1].x;
        if (base_x < 0 || base_x >= COLSIMAGE) break;
        if (img.at<unsigned char>(row, base_x) < thresOTSU) break;

        /* 搜右线 */
        for (int x = base_x - 10; x < COLSIMAGE - 1; x++)
        {
            if (x > COLSIMAGE - 5)
            {
                pointsRight.emplace_back(COLSIMAGE, row);
                break;
            }
            else if (img.at<unsigned char>(row, x + 1) < thresOTSU)
            {
                pointsRight.emplace_back(x, row);
                break;
            }
        }
        /* 搜左线 */
        for (int x = base_x + 10; x > 0; x--)
        {
            if (x < 5)
            {
                pointsLeft.emplace_back(0, row);
                break;
            }
            else if (img.at<unsigned char>(row, x - 1) < thresOTSU)
            {
                pointsLeft.emplace_back(x, row);
                break;
            }
        }
        /* 计算中点 */
        if ((int)pointsLeft.size() > step && (int)pointsRight.size() > step)
        {
            pointsMid.emplace_back((pointsLeft[step].x + pointsRight[step].x) / 2, row);
            step++;
        }
        else
        {
            break;
        }
    }
    pointsLeft_size  = (int)pointsLeft.size();
    pointsRight_size = (int)pointsRight.size();
    pointsMid_size   = (int)pointsMid.size();
}


cv::Point Ring::find_left_down()
{
    if (pointsLeft_size < 4) return cv::Point(0, 0);
    if (pointsLeft[0].x != 0 && pointsLeft[1].x != 0 && pointsLeft[3].x != 0)
    {
        for (int i = 2; i < 100 && i + 1 < pointsLeft_size; i++)
        {
            if (std::abs(pointsLeft[i - 2].x - pointsLeft[i - 1].x) < 5
             && std::abs(pointsLeft[i].x - pointsLeft[i - 1].x) < 5
             && pointsLeft[i].x - pointsLeft[i + 1].x > 8
             && pointsLeft[i + 1].x < 180 && i + 1 < 140)
            {
                L_left_down_found = true;
                L_id_left_down    = i;
                return cv::Point(pointsLeft[i].x, pointsLeft[i].y);
            }
        }
    }
    return cv::Point(0, 0);
}


cv::Point Ring::find_right_down()
{
    if (pointsRight_size < 4) return cv::Point(0, 0);
    if (pointsRight[0].x != 0 && pointsRight[1].x != 0 && pointsRight[3].x != 0)
    {
        for (int i = 2; i < 100 && i + 1 < pointsRight_size; i++)
        {
            if (std::abs(pointsRight[i - 2].x - pointsRight[i - 1].x) < 8
             && std::abs(pointsRight[i].x - pointsRight[i - 1].x) < 8
             && pointsRight[i + 1].x - pointsRight[i].x > 5
             && pointsRight[i + 1].x > 180 && i + 1 < 140)
            {
                L_right_down_found = true;
                L_id_right_down    = i;
                return cv::Point(pointsRight[i].x, pointsRight[i].y);
            }
        }
    }
    return cv::Point(0, 0);
}


cv::Point Ring::find_left_mid(int start)
{
    int max_index       = 0;
    int no_size_counter = 0;
    for (int i = start; i < pointsLeft_size - 20; i++)
    {
        if (pointsLeft[i].x > pointsLeft[max_index].x)
        {
            max_index       = i;
            no_size_counter = 0;
        }
        if (pointsLeft[i].x < pointsLeft[max_index].x - 1
         && pointsLeft[i].x > pointsLeft[max_index].x - 5)
        {
            no_size_counter++;
        }
        if (no_size_counter > 4) break;
    }
    if (no_size_counter > 2 && max_index != 0
        && pointsLeft[max_index].x > 30 && pointsLeft[max_index].x < 160)
    {
        L_left_mid_found = true;
        L_id_left_mid    = max_index;
        return cv::Point(pointsLeft[max_index].x, pointsLeft[max_index].y);
    }
    return cv::Point(0, 0);
}


cv::Point Ring::find_right_mid(int start)
{
    int min_index       = 0;
    int no_size_counter = 0;
    for (int i = start; i < pointsRight_size - 20; i++)
    {
        if (pointsRight[i].x < pointsRight[min_index].x)
        {
            min_index       = i;
            no_size_counter = 0;
        }
        if (pointsRight[i].x > pointsRight[min_index].x + 1
         && pointsRight[i].x < pointsRight[min_index].x + 5)
        {
            no_size_counter++;
        }
        if (no_size_counter > 4) break;
    }
    if (no_size_counter > 2 && min_index != 0
        && pointsRight[min_index].x < 260 && pointsRight[min_index].x > 160)
    {
        L_right_mid_found = true;
        L_id_right_mid    = min_index;
        return cv::Point(pointsRight[min_index].x, pointsRight[min_index].y);
    }
    return cv::Point(0, 0);
}


cv::Point Ring::find_left_up()
{
    for (int i = pointsLeft_size - 30; i > 20 && i + 2 < pointsLeft_size; i--)
    {
        if (std::abs(pointsLeft[i + 2].x - pointsLeft[i + 1].x) < 5
         && std::abs(pointsLeft[i].x - pointsLeft[i + 1].x) < 5
         && pointsLeft[i].x - pointsLeft[i - 1].x > 8
         && pointsLeft[i].x < 180)
        {
            L_left_up_found = true;
            L_id_left_up    = i;
            return cv::Point(pointsLeft[L_id_left_up].x, pointsLeft[L_id_left_up].y);
        }
    }
    return cv::Point(0, 0);
}


cv::Point Ring::find_right_up()
{
    for (int i = pointsRight_size - 30; i > 10 && i + 2 < pointsRight_size; i--)
    {
        if (std::abs(pointsRight[i + 2].x - pointsRight[i + 1].x) < 5
         && std::abs(pointsRight[i].x - pointsRight[i + 1].x) < 5
         && pointsRight[i - 1].x - pointsRight[i].x > 8
         && pointsRight[i + 1].x > 160)
        {
            L_right_up_found = true;
            L_id_right_up    = i;
            return cv::Point(pointsRight[L_id_right_up].x, pointsRight[L_id_right_up].y);
        }
    }
    return cv::Point(0, 0);
}


void Ring::findline_lefthand_adaptive(cv::Mat &img, int /*block_size*/, int /*clip_value*/,
                                      int x, int y,
                                      std::vector<POINT> &pointsEdgeLeft,
                                      int &pointsEdgeLeft_size)
{
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while ((step < POINTS_MAX_LEN / 2)
        && half < x && x < (img.cols - half - 1)
        && half < y && y < (img.rows - half - 1)
        && turn < 4)
    {
        int local_thres = 128;
        int front_value     = img.at<unsigned char>(y + dir_front[dir][1],
                                                    x + dir_front[dir][0]);
        int frontleft_value = img.at<unsigned char>(y + dir_frontleft[dir][1],
                                                    x + dir_frontleft[dir][0]);
        if (front_value < local_thres) { dir = (dir + 1) % 4; turn++; }
        else if (frontleft_value < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pointsEdgeLeft.emplace_back(x, y);
            step++; turn = 0;
        }
        else
        {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pointsEdgeLeft.emplace_back(x, y);
            step++; turn = 0;
        }
    }
    pointsEdgeLeft_size = step;
}


void Ring::findline_righthand_adaptive(cv::Mat &img, int /*block_size*/, int /*clip_value*/,
                                       int x, int y,
                                       std::vector<POINT> &pointsEdgeRight,
                                       int &pointsEdgeRight_size)
{
    int step = 0, dir = 0, turn = 0;
    while ((step < POINTS_MAX_LEN / 2)
        && 0 < x && x < (img.cols - 1)
        && 0 < y && y < (img.rows - 1)
        && turn < 4)
    {
        int local_thres = 128;
        int front_value      = img.at<unsigned char>(y + dir_front[dir][1],
                                                     x + dir_front[dir][0]);
        int frontright_value = img.at<unsigned char>(y + dir_frontright[dir][1],
                                                     x + dir_frontright[dir][0]);
        if (front_value < local_thres) { dir = (dir + 3) % 4; turn++; }
        else if (frontright_value < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pointsEdgeRight.emplace_back(x, y);
            step++; turn = 0;
        }
        else
        {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pointsEdgeRight.emplace_back(x, y);
            step++; turn = 0;
        }
    }
    pointsEdgeRight_size = step;
}


void Ring::blur_points(int side, int kernel)
{
    int half = kernel / 2;
    if (side == 0)
    {
        for (int i = 0; i < t_far_entering_edge_size; i++)
        {
            b_t_far_entering_edge.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                int idx = general.clip(i + j, 0, t_far_entering_edge_size - 1);
                b_t_far_entering_edge[i].x +=
                    t_far_entering_edge[idx].x * (half + 1 - std::abs(j));
                b_t_far_entering_edge[i].y +=
                    t_far_entering_edge[idx].y * (half + 1 - std::abs(j));
            }
            b_t_far_entering_edge[i].x /= (2 * half + 2) * (half + 1) / 2;
            b_t_far_entering_edge[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    else
    {
        for (int i = 0; i < t_far_exiting_edge_size; i++)
        {
            b_t_far_exiting_edge.emplace_back(0, 0);
            for (int j = -half; j <= half; j++)
            {
                int idx = general.clip(i + j, 0, t_far_exiting_edge_size - 1);
                b_t_far_exiting_edge[i].x +=
                    t_far_exiting_edge[idx].x * (half + 1 - std::abs(j));
                b_t_far_exiting_edge[i].y +=
                    t_far_exiting_edge[idx].y * (half + 1 - std::abs(j));
            }
            b_t_far_exiting_edge[i].x /= (2 * half + 2) * (half + 1) / 2;
            b_t_far_exiting_edge[i].y /= (2 * half + 2) * (half + 1) / 2;
        }
    }
    b_t_far_entering_edge_size = (int)b_t_far_entering_edge.size();
    b_t_far_exiting_edge_size  = (int)b_t_far_exiting_edge.size();
}


void Ring::resample_points(std::vector<POINT> &in, int in_size,
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


void Ring::entering_track_far_line(cv::Mat imgBinary)
{
    far_entering_edge.clear();      far_entering_edge_size       = 0;
    t_far_entering_edge.clear();    t_far_entering_edge_size     = 0;
    b_t_far_entering_edge.clear();  b_t_far_entering_edge_size   = 0;
    s_b_t_far_entering_edge.clear(); s_b_t_far_entering_edge_size = 0;

    if (entering_x0 < 0 || entering_x0 >= COLSIMAGE
     || entering_y0 < 6 || entering_y0 >= ROI_H)
        return;

    int y0 = entering_y0 - 5;
    for (; y0 > 0; y0--)
    {
        if (imgBinary.at<unsigned char>(y0 - 1, entering_x0) < 128) break;
    }
    if (y0 > 0 && imgBinary.at<unsigned char>(y0, entering_x0) > 128)
    {
        if (flag_ring == Left_Ring_Begin)
            findline_righthand_adaptive(imgBinary, block_size, clip_value,
                                        entering_x0, y0,
                                        far_entering_edge, far_entering_edge_size);
        else if (flag_ring == Right_Ring_Begin)
            findline_lefthand_adaptive(imgBinary, block_size, clip_value,
                                       entering_x0, y0,
                                       far_entering_edge, far_entering_edge_size);
    }

    for (int i = 0; i < far_entering_edge_size; i++)
    {
        int a, b;
        if (general.transf(a, b, far_entering_edge[i].x, far_entering_edge[i].y))
            t_far_entering_edge.emplace_back(a, b);
        else
            break;
    }
    t_far_entering_edge_size = (int)t_far_entering_edge.size();

    blur_points(0, 11);
    b_t_far_entering_edge_size = t_far_entering_edge_size;

    resample_points(b_t_far_entering_edge, b_t_far_entering_edge_size,
                    s_b_t_far_entering_edge, s_b_t_far_entering_edge_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));
}


void Ring::exiting_track_far_line(cv::Mat imgBinary)
{
    far_exiting_edge.clear();      far_exiting_edge_size       = 0;
    t_far_exiting_edge.clear();    t_far_exiting_edge_size     = 0;
    b_t_far_exiting_edge.clear();  b_t_far_exiting_edge_size   = 0;
    s_b_t_far_exiting_edge.clear(); s_b_t_far_exiting_edge_size = 0;

    if (exiting_x0 < 0 || exiting_x0 >= COLSIMAGE
     || exiting_y0 < 6 || exiting_y0 >= ROI_H)
        return;

    int y0 = exiting_y0 - 5;
    for (; y0 > 0; y0--)
    {
        if (imgBinary.at<unsigned char>(y0 - 1, exiting_x0) < 128) break;
    }
    if (y0 > 0 && imgBinary.at<unsigned char>(y0, exiting_x0) > 128)
    {
        if (flag_ring == Left_Ring_Out)
            findline_righthand_adaptive(imgBinary, block_size, clip_value,
                                        exiting_x0, y0,
                                        far_exiting_edge, far_exiting_edge_size);
        else if (flag_ring == Right_Ring_Out)
            findline_lefthand_adaptive(imgBinary, block_size, clip_value,
                                       exiting_x0, y0,
                                       far_exiting_edge, far_exiting_edge_size);
    }

    for (int i = 0; i < far_exiting_edge_size; i++)
    {
        int a, b;
        if (general.transf(a, b, far_exiting_edge[i].x, far_exiting_edge[i].y))
            t_far_exiting_edge.emplace_back(a, b);
        else
            break;
    }
    t_far_exiting_edge_size = (int)t_far_exiting_edge.size();

    blur_points(1, 11);
    b_t_far_exiting_edge_size = t_far_exiting_edge_size;

    resample_points(b_t_far_exiting_edge, b_t_far_exiting_edge_size,
                    s_b_t_far_exiting_edge, s_b_t_far_exiting_edge_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));
}
