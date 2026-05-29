/**
 * @file ring.cpp
 * @brief 环岛处理类实现
 *
 * 左环：None → Begin(巡右) → In(巡左) → Out(巡右) → None
 * 右环：镜像；识别用迷宫俯视图 t_pointsEdge + L 角点；丢复线见 update_edge_regain。
 */

#include "ring.hpp"
#include "findline_core.hpp"
#include <cmath>
#include <cstdio>


/**
 * @brief 外侧迷宫边线列坐标是否足够平直（预进环稳定性，对齐 temp_repo）
 * @param edge  俯视图边线点列
 * @param size  有效点数（不超过 edge.size()）
 * @return      true 表示外侧边线稳定
 */
static bool ring_outer_edge_stable(const std::vector<POINT> &edge, int size)
{
    const int n = (int)edge.size();
    if (size > n)
        size = n;
    if (size < 30)
        return false;
    const int i2 = size * 2 / 3;
    const int i3 = size / 3;
    return std::abs(edge[size - 1].x - edge[i2].x) < 30
        && std::abs(edge[i3].x - edge[0].x) < 30;
}

/** 最近一帧进环判据快照（供 display HUD 读取） */
static RingEntryDebugSnapshot s_ring_entry_dbg{};

/**
 * @brief 获取最近一帧 Ring_Check 的进环调试快照
 * @param out [out] 输出调试快照结构体
 * @return 无
 * @sample RingEntryDebugSnapshot snap{}; ring_debug_fill_entry(snap);
 * @note  仅复制缓存，不触发任何状态机迁移。
 */
void ring_debug_fill_entry(RingEntryDebugSnapshot &out)
{
    out = s_ring_entry_dbg;
}


Ring::Ring() {}


void Ring::reset()
{
    flag_ring = Ring_None;

    state_locking      = false;
    right_regain_count = 0;
    left_regain_count  = 0;
    right_edge_phase   = EDGE_OK;
    left_edge_phase    = EDGE_OK;
    left_entry_confirm_count  = 0;
    right_entry_confirm_count = 0;

    ring_break_left  = false;
    ring_break_right = false;
    ring_break_y_top[0] = -1; ring_break_y_top[1] = -1;
    ring_break_y_bot[0] = -1; ring_break_y_bot[1] = -1;
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


/**
 * @brief 原图行扫描：每行从 x=160 向两侧找首个白→黑跳变点
 * @param img  ROI 二值图（CV_8UC1，COLSIMAGE×ROI_H = 320×130，相机视角）
 * @return 无
 * @sample ring.ring_findline(bin_img);
 * @note  写入 ring_left_x[ROI_H] / ring_right_x[ROI_H] / ring_findline_valid[ROI_H]。
 *        遍历 y=ROI_H-1→0（近→远）；中心 (y,160) 连续 RING_FINDLINE_CENTER_BLACK_STOP 行黑则早停。
 *        未扫描远端行保持 valid=false 与哨兵 3/318。单行中心黑亦 valid=false。
 *        到边界仍未找到跳变点记 3/318 且 valid=true。
 */
void Ring::ring_findline(const cv::Mat &img)
{
    const int CX    = RING_FINDLINE_CENTER_X;          /* 160 */
    const int L_END = RING_EDGE_BAND - 2;              /* 3 */
    const int R_END = COLSIMAGE - 1 - (RING_EDGE_BAND - 2); /* 318 */
    const int THRES = RING_FINDLINE_THRES;

    for (int y = 0; y < ROI_H; ++y)
    {
        ring_left_x[y]         = L_END;
        ring_right_x[y]        = R_END;
        ring_findline_valid[y] = false;
    }

    if (img.empty() || img.cols < COLSIMAGE || img.rows < ROI_H)
        return;

    int black_streak = 0;

    for (int y = ROI_H - 1; y >= 0; --y)
    {
        const unsigned char *row = img.ptr<unsigned char>(y);

        if (row[CX] < THRES)
        {
            if (++black_streak >= RING_FINDLINE_CENTER_BLACK_STOP)
                break;
            continue;
        }

        black_streak = 0;

        int lx = L_END;
        for (int x = CX; x > L_END; --x)
        {
            if (row[x] >= THRES && row[x - 1] < THRES)
            {
                lx = x;
                break;
            }
        }
        ring_left_x[y] = lx;

        int rx = R_END;
        for (int x = CX; x < R_END; ++x)
        {
            if (row[x] >= THRES && row[x + 1] < THRES)
            {
                rx = x;
                break;
            }
        }
        ring_right_x[y] = rx;

        ring_findline_valid[y] = true;
    }
}


/**
 * @brief 断线-缺口-断线 形态匹配（环岛进环辅助判据）- 贪婪匹配改良版
 * @param side  0=左，1=右
 * @param[out] y_bot  命中时填断线模式下沿 y（未命中保持 -1）
 * @param[out] y_top  命中时填断线模式上沿 y（未命中保持 -1）
 * @return true=命中（正常>=5行 -> 断线>=5行 -> 正常>=5行 模式）
 * @note  3 段状态机：inside(>=N) → at_edge(>=N) → inside(>=N)，N=RING_BREAK_RUN_LEN=5 行。
 * 改良逻辑：不会在刚好等于5行时盲目跳跃，而是无限吃进当前状态行，直到遇到【真实状态切换】且【累计已达标】时，才进入下一阶段。
 * 容忍 RING_BREAK_BAD_TOL=3 行异常（噪点不会打断累积）。
 */
bool Ring::ring_breakline_check(int side, int &y_bot, int &y_top)
{
    y_bot = -1;
    y_top = -1;
    if (side != 0 && side != 1) return false;

    // 获取对应侧的边界数组
    const int *arr = (side == 0) ? ring_left_x : ring_right_x;

    // 阈值常量定义
    const int EDGE      = RING_EDGE_BAND;                              /* 5：判断为断线的阈值 */
    const int INSIDE_TH = RING_EDGE_BAND + RING_INSIDE_GAP;            /* 10：判断为正常线（赛道内）的阈值 */
    const int RUN       = RING_BREAK_RUN_LEN;                          /* 5：每个阶段要求的最低达标行数 */
    const int BAD_TOL   = RING_BREAK_BAD_TOL;                          /* 3：容忍的连续坏点行数 */
    const int Y_TOP     = RING_BREAK_SCAN_Y_TOP;                       /* 10：扫描的最远行界限 */

    // 状态机定义
    enum { S_INSIDE_BOT = 0, S_EDGE = 1, S_INSIDE_TOP = 2 };
    int state = S_INSIDE_BOT;
    int good = 0, bad = 0; // good:符合当前状态的有效行数，bad:不符合的异常行数

    // 记录各段的起止Y坐标（辅助变量，用于最终返回）
    int seg_bot_y = -1; 
    int seg_edge_bot_y = -1, seg_edge_top_y = -1;
    int seg_inside_top_top_y = -1; 

    // 从下往上（从近到远）扫描
    for (int y = ROI_H - 1; y >= Y_TOP; --y)
    {
        int x = arr[y];
        bool valid = ring_findline_valid[y]; // 该行是否成功扫到线（未被黑块早停）

        bool is_inside; // 标志位：是否为正常赛道线
        bool is_edge;   // 标志位：是否为断线/丢线
        
        if (side == 0) // 左边线
        {
            is_inside = valid && (x > INSIDE_TH);            /* 左侧正常：x > 10 */
            is_edge   = valid && (x <= EDGE);                /* 左侧断线：x <= 5 */
        }
        else // 右边线
        {
            is_inside = valid && (x < COLSIMAGE - 1 - INSIDE_TH); /* 右侧正常：x < 309 */
            is_edge   = valid && (x >= COLSIMAGE - 1 - EDGE);     /* 右侧断线：x >= 314 */
        }

        switch (state)
        {
            case S_INSIDE_BOT: // 【阶段一：底部正常赛道】
                if (is_inside)
                {
                    if (good == 0) seg_bot_y = y; // 记录最底端起始Y
                    good++;  // 贪婪匹配：只要是正常线就无限累加，无论是5行还是20行
                    bad = 0; // 清零坏点计数
                }
                else if (is_edge && good >= RUN)
                {
                    // 核心跨越逻辑：已经攒够了>=5行正常，且【真正遇到了断线】
                    state = S_EDGE; // 跳入阶段二
                    seg_edge_bot_y = y; // 记录断线下沿
                    seg_edge_top_y = y;
                    good = 1; // 当前这行就是阶段二的第1个有效行
                    bad = 0;
                }
                else
                {
                    // 未攒够5行就断了，或者遇到不属于正常也不属于断线的过渡杂区
                    bad++;
                    if (bad > BAD_TOL) // 超过容忍极限，彻底推翻重来
                    {
                        good = 0; bad = 0;
                        seg_bot_y = -1;
                    }
                }
                break;

            case S_EDGE: // 【阶段二：中间断线缺口】
                if (is_edge)
                {
                    seg_edge_top_y = y; // 持续更新断线的上沿
                    good++; // 贪婪匹配：只要是断线就无限累加
                    bad = 0;
                }
                else if (is_inside && good >= RUN)
                {
                    // 核心跨越逻辑：已经攒够了>=5行断线，且【真正再次遇到了正常线】
                    state = S_INSIDE_TOP; // 跳入阶段三
                    seg_inside_top_top_y = y;
                    good = 1;
                    bad = 0;
                }
                else
                {
                    bad++;
                    if (bad > BAD_TOL) 
                    {
                        // 发现是假缺口（断线后没接上正常赛道），状态机崩溃，退回阶段一
                        state = S_INSIDE_BOT;
                        good = 0; bad = 0;
                        seg_bot_y = -1;
                        seg_edge_bot_y = -1;
                        seg_edge_top_y = -1;
                        
                        // 容错：如果当前行恰好是正常线，直接作为阶段一的起点
                        if (is_inside)
                        {
                            seg_bot_y = y;
                            good = 1;
                        }
                    }
                }
                break;

            case S_INSIDE_TOP: // 【阶段三：顶部正常赛道】
                if (is_inside)
                {
                    seg_inside_top_top_y = y; 
                    good++;
                    bad = 0;
                    // 对于最后一段，一旦达标>=5行，就证明整个“夹心饼干”形态完整，直接判定成功
                    if (good >= RUN)
                    {
                        y_bot = seg_edge_bot_y; // 导出断线的下沿
                        y_top = seg_edge_top_y; // 导出断线的上沿
                        return true; // 命中环岛特征！
                    }
                }
                else
                {
                    bad++;
                    if (bad > BAD_TOL) // 顶部正常线不够5行就消失了，退回最初状态
                    {
                        state = S_INSIDE_BOT;
                        good = 0; bad = 0;
                        seg_bot_y = -1;
                        seg_edge_bot_y = -1;
                        seg_edge_top_y = -1;
                        seg_inside_top_top_y = -1;
                    }
                }
                break;
        }
    }

    // 压制编译器针对未使用变量的警告
    (void)seg_bot_y;
    (void)seg_inside_top_top_y;
    
    // 扫描完整个ROI也没能完整匹配三个阶段，返回失败
    return false;
}


/**
 * @brief 环岛识别（状态机迁移，迷宫俯视图边线 + L 角点）
 * @param imgBinary              二值图（CV_8UC1，ROI 320x130）
 * @param is_left_straight       左侧是否直道
 * @param is_right_straight      右侧是否直道
 * @param t_pointsEdgeLeft_size  左透视边线点数
 * @param t_pointsEdgeRight_size 右透视边线点数
 * @param is_L_left_found        左 L 角点找到标志
 * @param is_L_right_found       右 L 角点找到标志
 * @param t_L_pointLeft          左 L 角点（俯视图坐标）
 * @param t_L_pointRight         右 L 角点（俯视图坐标）
 * @param t_pointsEdgeLeft       左透视边线点列
 * @param t_pointsEdgeRight      右透视边线点列
 * @return 无
 * @sample ring.Ring_Check(bin_img, is_left_straight, is_right_straight, tl, tr, ll, lr, pL, pR, eL, eR);
 * @note  当 flag_ring==Ring_None 时会同步缓存进环条件快照，供屏显调试读取。
 */
void Ring::Ring_Check(cv::Mat &imgBinary,
                      bool is_left_straight, bool is_right_straight,
                      int t_pointsEdgeLeft_size, int t_pointsEdgeRight_size,
                      bool is_L_left_found, bool is_L_right_found,
                      cv::Point t_L_pointLeft, cv::Point t_L_pointRight,
                      std::vector<POINT> &t_pointsEdgeLeft,
                      std::vector<POINT> &t_pointsEdgeRight)
{
    RingEntryDebugSnapshot snap{};
    snap.ring_flag               = (int)flag_ring;
    snap.ring_cooldown           = ring_cooldown;
    snap.is_L_left_found         = is_L_left_found ? 1 : 0;
    snap.is_L_right_found        = is_L_right_found ? 1 : 0;
    snap.t_pointsEdgeLeft_size   = t_pointsEdgeLeft_size;
    snap.t_pointsEdgeRight_size  = t_pointsEdgeRight_size;
    snap.is_left_straight        = is_left_straight ? 1 : 0;
    snap.is_right_straight       = is_right_straight ? 1 : 0;
    snap.t_L_pointLeft_y         = t_L_pointLeft.y;
    snap.t_L_pointRight_y        = t_L_pointRight.y;
    snap.left_entry_confirm_cnt  = left_entry_confirm_count;
    snap.right_entry_confirm_cnt = right_entry_confirm_count;

    if (t_pointsEdgeLeft_size == 0 && t_pointsEdgeRight_size == 0)
    {
        s_ring_entry_dbg = snap;
        return;
    }

    if (flag_ring == Ring_None)
    {
        if (ring_cooldown > 0)
        {
            ring_cooldown--;
            snap.ring_cooldown = ring_cooldown;
            s_ring_entry_dbg = snap;
            return;
        }

        snap.eval_enabled = 1;

        const bool left_single_corner_ok = is_L_left_found && !is_L_right_found;
        const bool right_single_corner_ok = is_L_right_found && !is_L_left_found;
        const bool left_size_ok = t_pointsEdgeLeft_size < L_SMALL
                               && t_pointsEdgeRight_size > R_LARGE;
        const bool right_size_ok = t_pointsEdgeRight_size < L_SMALL
                                && t_pointsEdgeLeft_size > R_LARGE;

        /* 行扫描 + 断线模式：仅环岛进环候选帧执行（单 L + 点数失衡） */
        const bool ring_scan_candidate =
            (left_single_corner_ok && left_size_ok)
            || (right_single_corner_ok && right_size_ok);

        if (ring_scan_candidate)
        {
            ring_findline(imgBinary);
            int yb_l = -1, yt_l = -1;
            int yb_r = -1, yt_r = -1;
            ring_break_left  = ring_breakline_check(0, yb_l, yt_l);
            ring_break_right = ring_breakline_check(1, yb_r, yt_r);
            ring_break_y_bot[0] = yb_l; ring_break_y_top[0] = yt_l;
            ring_break_y_bot[1] = yb_r; ring_break_y_top[1] = yt_r;
        }
        else
        {
            ring_break_left  = false;
            ring_break_right = false;
            ring_break_y_bot[0] = -1; ring_break_y_top[0] = -1;
            ring_break_y_bot[1] = -1; ring_break_y_top[1] = -1;
        }

        const bool left_straight_ok = is_right_straight && !is_left_straight;
        const bool right_straight_ok = is_left_straight && !is_right_straight;
        const bool left_y_ok = t_L_pointLeft.y < 110 && t_L_pointLeft.y > 20;
        const bool right_y_ok = t_L_pointRight.y < 110 && t_L_pointRight.y > 20;

        const bool left_entry_cond = left_single_corner_ok
                                  && left_size_ok
                                  && left_straight_ok
                                  && left_y_ok
                                  && ring_break_left
                                  && !ring_break_right;
        const bool right_entry_cond = right_single_corner_ok
                                   && right_size_ok
                                   && right_straight_ok
                                   && right_y_ok
                                   && ring_break_right
                                   && !ring_break_left;

        snap.L_single_corner_ok = left_single_corner_ok ? 1 : 0;
        snap.R_single_corner_ok = right_single_corner_ok ? 1 : 0;
        snap.L_size_ok = left_size_ok ? 1 : 0;
        snap.R_size_ok = right_size_ok ? 1 : 0;
        snap.L_straight_ok = left_straight_ok ? 1 : 0;
        snap.R_straight_ok = right_straight_ok ? 1 : 0;
        snap.L_y_ok = left_y_ok ? 1 : 0;
        snap.R_y_ok = right_y_ok ? 1 : 0;
        snap.L_break_ok = ring_break_left ? 1 : 0;
        snap.R_break_ok = ring_break_right ? 1 : 0;
        snap.left_entry_cond = left_entry_cond ? 1 : 0;
        snap.right_entry_cond = right_entry_cond ? 1 : 0;

        if (left_entry_cond)
            left_entry_confirm_count++;
        else
            left_entry_confirm_count = 0;

        if (right_entry_cond)
            right_entry_confirm_count++;
        else
            right_entry_confirm_count = 0;

        if (left_entry_confirm_count >= RING_ENTRY_CONFIRM_FRAMES)
        {
            flag_ring                 = Left_Ring_Begin;
            state_locking             = true;
            left_regain_count         = 0;
            left_edge_phase           = EDGE_OK;
            right_regain_count        = 0;
            right_edge_phase          = EDGE_OK;
            left_entry_confirm_count  = 0;
            right_entry_confirm_count = 0;
        }
        else if (right_entry_confirm_count >= RING_ENTRY_CONFIRM_FRAMES)
        {
            flag_ring                 = Right_Ring_Begin;
            state_locking             = true;
            right_regain_count        = 0;
            right_edge_phase          = EDGE_OK;
            left_regain_count         = 0;
            left_edge_phase           = EDGE_OK;
            left_entry_confirm_count  = 0;
            right_entry_confirm_count = 0;
        }

        snap.ring_flag               = (int)flag_ring;
        snap.left_entry_confirm_cnt  = left_entry_confirm_count;
        snap.right_entry_confirm_cnt = right_entry_confirm_count;
        s_ring_entry_dbg = snap;
        return;
    }

    /* ===== 左圆环 ===== */
    if (flag_ring == Left_Ring_Begin)
    {
        update_edge_regain(t_pointsEdgeLeft_size, left_regain_count, left_edge_phase);

        if (left_regain_count >= 1)
        {
            flag_ring          = Left_Ring_In;
            state_locking      = true;
            right_regain_count = 0;
            right_edge_phase   = EDGE_OK;
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
    if (flag_ring == Right_Ring_Begin)
    {
        update_edge_regain(t_pointsEdgeRight_size, right_regain_count, right_edge_phase);

        if (right_regain_count >= 1)
        {
            flag_ring          = Right_Ring_In;
            state_locking      = true;
            left_regain_count  = 0;
            left_edge_phase    = EDGE_OK;
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

    snap.ring_flag               = (int)flag_ring;
    snap.ring_cooldown           = ring_cooldown;
    snap.left_entry_confirm_cnt  = left_entry_confirm_count;
    snap.right_entry_confirm_cnt = right_entry_confirm_count;
    s_ring_entry_dbg = snap;
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

    /* 左环：Begin/Out 巡右（清左）；In 巡左（清右） */
    if (flag_ring == Left_Ring_Begin || flag_ring == Left_Ring_Out)
    {
        t_pointsEdgeLeft.clear();
        t_pointsEdgeLeft_size = 0;
    }
    else if (flag_ring == Left_Ring_In)
    {
        t_pointsEdgeRight.clear();
        t_pointsEdgeRight_size = 0;
    }

    /* 右环：Begin/Out 巡左（清右）；In 巡右（清左） */
    if (flag_ring == Right_Ring_Begin || flag_ring == Right_Ring_Out)
    {
        t_pointsEdgeRight.clear();
        t_pointsEdgeRight_size = 0;
    }
    else if (flag_ring == Right_Ring_In)
    {
        t_pointsEdgeLeft.clear();
        t_pointsEdgeLeft_size = 0;
    }
}


#if 0 /* 旧版行扫描 ring_find_line / find_*，仅保留对照 */

void Ring::ring_find_line(cv::Mat &img, int /*y_start*/)
{
    int rowstart = (ROI_H - 5);
    int rowup    = 5;
    int thresOTSU = 128;
    std::vector<POINT> pointsLeft;
    std::vector<POINT> pointsRight;
    std::vector<POINT> pointsMid;
    int pointsLeft_size  = 0;
    int pointsRight_size = 0;
    int pointsMid_size   = 0;
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
    bool L_left_down_found = false;
    int L_id_left_down = 666;
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
    (void)L_left_down_found;
    (void)L_id_left_down;
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

#endif /* 旧版行扫描 */


void Ring::findline_lefthand_adaptive(cv::Mat &img, int /*block_size*/, int /*clip_value*/,
                                      int x, int y,
                                      std::vector<POINT> &pointsEdgeLeft,
                                      int &pointsEdgeLeft_size)
{
    findline_core::lefthand_fixed(img, x, y, pointsEdgeLeft, pointsEdgeLeft_size,
                                  POINTS_MAX_LEN / 2);
}


void Ring::findline_righthand_adaptive(cv::Mat &img, int /*block_size*/, int /*clip_value*/,
                                       int x, int y,
                                       std::vector<POINT> &pointsEdgeRight,
                                       int &pointsEdgeRight_size)
{
    findline_core::righthand_fixed(img, x, y, pointsEdgeRight, pointsEdgeRight_size,
                                   POINTS_MAX_LEN / 2);
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
    }
    t_far_entering_edge_size = (int)t_far_entering_edge.size();

    blur_points(0, 5);
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
    }
    t_far_exiting_edge_size = (int)t_far_exiting_edge.size();

    blur_points(1, 5);
    b_t_far_exiting_edge_size = t_far_exiting_edge_size;

    resample_points(b_t_far_exiting_edge, b_t_far_exiting_edge_size,
                    s_b_t_far_exiting_edge, s_b_t_far_exiting_edge_size,
                    (float)(SAMPLE_DIST * pixel_per_meter));
}
