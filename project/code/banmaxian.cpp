/**
 * @file banmaxian.cpp
 * @brief 斑马线识别模块实现
 *
 * 算法：逐行扫描 ROI 二值图 bin_img（320x130，0/255），统计每一行在
 *       列区间 [BM_COL_LEFT, BM_COL_RIGHT) 内的黑白跳变次数；
 *       任一行跳变数 ≥ BM_TRANS_THRES 即视为该帧命中。
 * 解锁：必须先看到一次十字状态机的 Cross::Cross_Begin（由调用方传入），
 *       之后才真正开始扫描；解锁后即使十字结束也保持解锁，直到 banmaxian_reset()。
 * 去抖：连续 BM_CONFIRM_FRAMES 帧命中才置 banmaxian_flag=1（上升沿打印一次）；
 *       连续 BM_RELEASE_FRAMES 帧未命中才清 0。
 */

#include "banmaxian.hpp"
#include "cross.hpp"
#include "general.hpp"
#include <cstdio>


/* ====================== 调参常量 ====================== */

/** ROI 局部行号扫描上界（远车端，含） */
static const int BM_ROW_TOP        = 30;
/** ROI 局部行号扫描下界（近车端，不含） */
static const int BM_ROW_BOTTOM     = 125;
/** 列扫描下界（左侧 20 列不参与） */
static const int BM_COL_LEFT       = 20;
/** 列扫描上界（右侧 20 列不参与） */
static const int BM_COL_RIGHT      = COLSIMAGE - 20;
/** 单行黑白跳变阈值（≥ 即判定该行为疑似斑马线行） */
static const int BM_TRANS_THRES    = 6;
/** 去抖：连续命中帧数门限，达到则置 banmaxian_flag=1 */
static const int BM_CONFIRM_FRAMES = 2;
/** 去抖：连续未命中帧数门限，达到则清 banmaxian_flag=0 */
static const int BM_RELEASE_FRAMES = 8;


/* ====================== 全局状态 ====================== */

int banmaxian_flag      = 0;
int banmaxian_total_cnt = 0;


/* ====================== 模块内部状态 ====================== */

static bool s_armed         = false;  /* 是否已被 Cross_Begin 解锁 */
static int  s_hit_streak    = 0;      /* 连续命中帧数 */
static int  s_miss_streak   = 0;      /* 连续未命中帧数 */
static int  s_last_max_trans = 0;     /* 最近一帧扫描中单行最大跳变数 */
static int  s_last_hit_row   = -1;    /* 最近一帧最大跳变所在的 ROI 行 */


/**
 * @brief 复位斑马线模块：清解锁 latch、命中计数与去抖计数器
 * @return 无
 * @sample banmaxian_reset();
 * @note   与 s_cross.reset() / s_ring.reset() 同步调用
 */
void banmaxian_reset(void)
{
    s_armed          = false;
    s_hit_streak     = 0;
    s_miss_streak    = 0;
    s_last_max_trans = 0;
    s_last_hit_row   = -1;
    banmaxian_flag      = 0;
    banmaxian_total_cnt = 0;
}


/**
 * @brief 每帧调用一次：在 bin_img 上做斑马线识别
 * @param bin_img    输入 ROI 二值图（CV_8UC1，期望 320x130，0/255）
 * @param cross_flag 当前十字状态机 flag（Cross::flag_Cross_e 转 int）
 * @return 无；结果写入全局 banmaxian_flag / banmaxian_total_cnt
 * @sample banmaxian_check(bin_img, (int)s_cross.flag_cross);
 * @note   仅在见过一次 Cross_Begin 之后才真正扫描；
 *         扫描行 ∈ [30, 125)，列 ∈ [20, 300)；
 *         单行跳变 ≥ 6 即视为命中；
 *         去抖：2 帧命中置 1、8 帧未命中清 0；
 *         上升沿向 stdout printf 一次，不刷屏。
 */
void banmaxian_check(const cv::Mat &bin_img, int cross_flag)
{
    if (cross_flag == (int)Cross::Cross_Begin)
        s_armed = true;

    if (!s_armed || bin_img.empty() || bin_img.channels() != 1)
    {
        s_last_max_trans = 0;
        s_last_hit_row   = -1;
        s_hit_streak     = 0;
        if (s_miss_streak < BM_RELEASE_FRAMES) s_miss_streak++;
        if (banmaxian_flag == 1 && s_miss_streak >= BM_RELEASE_FRAMES)
            banmaxian_flag = 0;
        return;
    }

    const int rows = bin_img.rows;
    const int cols = bin_img.cols;

    const int y_lo = BM_ROW_TOP    < 0    ? 0    : BM_ROW_TOP;
    const int y_hi = BM_ROW_BOTTOM > rows ? rows : BM_ROW_BOTTOM;
    const int x_lo = BM_COL_LEFT   < 1    ? 1    : BM_COL_LEFT;
    const int x_hi = BM_COL_RIGHT  > cols ? cols : BM_COL_RIGHT;

    int best_trans = 0;
    int best_row   = -1;

    for (int y = y_hi - 1; y >= y_lo; --y)
    {
        const uchar *p = bin_img.ptr<uchar>(y);
        int trans = 0;
        for (int x = x_lo + 1; x < x_hi; ++x)
        {
            if (p[x] != p[x - 1])
            {
                ++trans;
                if (trans >= BM_TRANS_THRES)
                    break;
            }
        }
        if (trans > best_trans)
        {
            best_trans = trans;
            best_row   = y;
        }
        if (best_trans >= BM_TRANS_THRES)
            break;
    }

    s_last_max_trans = best_trans;
    s_last_hit_row   = best_row;

    const bool hit = (best_trans >= BM_TRANS_THRES);
    if (hit)
    {
        if (s_hit_streak < BM_CONFIRM_FRAMES) s_hit_streak++;
        s_miss_streak = 0;
    }
    else
    {
        if (s_miss_streak < BM_RELEASE_FRAMES) s_miss_streak++;
        s_hit_streak = 0;
    }

    if (banmaxian_flag == 0 && s_hit_streak >= BM_CONFIRM_FRAMES)
    {
        banmaxian_flag = 1;
        banmaxian_total_cnt++;
        printf("[banmaxian] 识别到斑马线! row=%d trans=%d total=%d\n",
               best_row, best_trans, banmaxian_total_cnt);
        fflush(stdout);
    }
    else if (banmaxian_flag == 1 && s_miss_streak >= BM_RELEASE_FRAMES)
    {
        banmaxian_flag = 0;
    }
}


/**
 * @brief 填充斑马线模块调试快照
 * @param out 输出快照结构体
 * @return 无
 */
void banmaxian_debug_fill(BanmaxianDebugSnapshot &out)
{
    out.armed          = s_armed ? 1 : 0;
    out.flag           = banmaxian_flag;
    out.total_cnt      = banmaxian_total_cnt;
    out.last_max_trans = s_last_max_trans;
    out.last_hit_row   = s_last_hit_row;
}
