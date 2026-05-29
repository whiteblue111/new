#ifndef RING_HPP
#define RING_HPP

/**
 * @file ring.hpp
 * @brief 环岛处理类
 *
 * 6 态状态机（左 3 + 右 3 + None）：
 *   左环：None → Begin(巡右) → In(巡左) → Out(巡右) → None
 *   右环：镜像（Begin/Out 巡左，In 巡右）
 *
 * Ring_Check 依赖 trackRecognition 输出的迷宫俯视图边线 t_pointsEdge 与 L 角点；
 * Begin→In 以内侧边丢线→复线（update_edge_regain）为判据；
 * Ring_Run 按状态裁边供 fitting() 单边中线。
 * 旧版行扫描 ring_find_line 已 #if 0 保留，运行时不再调用。
 */

#include "general.hpp"
#include <opencv2/opencv.hpp>

/**
 * @brief 环岛进环判定调试快照（对应 Ring_Check 中 left/right_entry_cond 子项）
 *
 * 说明：
 * - `L*` 前缀字段对应左环进环判据；
 * - `R*` 前缀字段对应右环进环判据；
 * - 字段值统一使用 0/1，便于 IPS HUD 与串口直接显示。
 */
struct RingEntryDebugSnapshot
{
    int eval_enabled             = 0;  /**< 1=本帧执行了 Ring_None 进环判据评估 */
    int ring_flag               = 0;  /**< 当前 Ring::flag_Ring_e */
    int ring_cooldown           = 0;  /**< 进环冷却计数 */

    int is_L_left_found         = 0;
    int is_L_right_found        = 0;
    int t_pointsEdgeLeft_size   = 0;
    int t_pointsEdgeRight_size  = 0;
    int is_left_straight        = 0;
    int is_right_straight       = 0;
    int t_L_pointLeft_y         = -1;
    int t_L_pointRight_y        = -1;

    int L_single_corner_ok      = 0;  /**< is_L_left_found && !is_L_right_found */
    int R_single_corner_ok      = 0;  /**< is_L_right_found && !is_L_left_found */
    int L_size_ok               = 0;  /**< tL<L_SMALL && tR>R_LARGE */
    int R_size_ok               = 0;  /**< tR<L_SMALL && tL>R_LARGE */
    int L_straight_ok           = 0;  /**< is_right_straight && !is_left_straight */
    int R_straight_ok           = 0;  /**< is_left_straight && !is_right_straight */
    int L_y_ok                  = 0;  /**< t_L_pointLeft.y in (30,100) */
    int R_y_ok                  = 0;  /**< t_L_pointRight.y in (30,100) */

    int left_entry_cond         = 0;
    int right_entry_cond        = 0;
    int left_entry_confirm_cnt  = 0;
    int right_entry_confirm_cnt = 0;

    int L_break_ok              = 0;  /**< ring_breakline_check 左侧命中（5-5-5 形态） */
    int R_break_ok              = 0;  /**< ring_breakline_check 右侧命中（5-5-5 形态） */
};

/**
 * @brief 获取最近一帧 Ring_Check 的进环调试快照
 * @param out [out] 输出调试快照结构体
 * @return 无
 * @sample RingEntryDebugSnapshot snap{}; ring_debug_fill_entry(snap);
 * @note  该接口只做只读拷贝，不会改变环岛状态机。
 */
void ring_debug_fill_entry(RingEntryDebugSnapshot &out);


/**
 * @brief 环岛处理类
 */
class Ring
{
public:
    General general;

    /** 边线相位：用于丢线→复线边沿检测 */
    enum EdgePhase_e
    {
        EDGE_OK   = 0,
        EDGE_LOST = 1
    };

    /** 环岛状态枚举 */
    enum flag_Ring_e
    {
        Ring_None = 0,
        Left_Ring_Begin,
        Left_Ring_In,
        Left_Ring_Out,
        Right_Ring_Begin,
        Right_Ring_In,
        Right_Ring_Out
    };
    flag_Ring_e flag_ring = Ring_None;

    /** 标定阈值（实车可改） */
    static const int LOST_LINE         = 10;
    static const int REGAIN_LINE       = 10;
    static const int L_SMALL           = 60;
    static const int R_LARGE           = 100;
    static const int RING_COOLDOWN_MAX         = 150;
    static const int RING_ENTRY_CONFIRM_FRAMES = 3;

    /** ring_findline / ring_breakline_check 标定参数 */
    static const int RING_FINDLINE_CENTER_X  = 160; /**< 行扫描中心起点（COLSIMAGE/2） */
    static const int RING_FINDLINE_THRES     = 128; /**< 二值阈值（与 OTSU 一致） */
    static const int RING_FINDLINE_CENTER_BLACK_STOP = 2; /**< 中心连续黑行数达到则停止向远端扫描 */
    static const int RING_EDGE_BAND          = 3;   /**< 贴边判定：左<=5 / 右>=314 */
    static const int RING_INSIDE_GAP         = 10;   /**< inside 与 edge 之间的安全间隙，左:x>10 右:x<309 */
    static const int RING_BREAK_RUN_LEN      = 6;   /**< 每段最少连续行数 */
    static const int RING_BREAK_BAD_TOL      = 5;   /**< 每段允许的异常行数 */
    static const int RING_BREAK_SCAN_Y_TOP   = 5;  /**< 扫描 y 上界（远端透视压缩区不参与） */

    Ring();

    /**
     * @brief 将环岛状态机复位为 Ring_None，并清零计数器
     * @return 无
     * @sample ring.reset();
     * @note  供按键/调试调用；不修改 ring_cooldown
     */
    void reset();

    /**
     * @brief 环岛识别（状态机迁移，迷宫俯视图边线 + L 角点）
     * @param imgBinary               二值图（CV_8UC1，ROI 320x130）
     * @param is_left_straight        左侧是否直道
     * @param is_right_straight       右侧是否直道
     * @param t_pointsEdgeLeft_size   左透视边线点数
     * @param t_pointsEdgeRight_size  右透视边线点数
     * @param is_L_left_found         左 L 角点找到
     * @param is_L_right_found        右 L 角点找到
     * @param t_L_pointLeft           左 L 角点（俯视图坐标）
     * @param t_L_pointRight          右 L 角点（俯视图坐标）
     * @param t_pointsEdgeLeft        左透视边线
     * @param t_pointsEdgeRight       右透视边线
     * @note  进环需连续 RING_ENTRY_CONFIRM_FRAMES 帧满足 L 角点与边线形态条件
     */
    void Ring_Check(cv::Mat &imgBinary,
                    bool is_left_straight, bool is_right_straight,
                    int t_pointsEdgeLeft_size, int t_pointsEdgeRight_size,
                    bool is_L_left_found, bool is_L_right_found,
                    cv::Point t_L_pointLeft, cv::Point t_L_pointRight,
                    std::vector<POINT> &t_pointsEdgeLeft,
                    std::vector<POINT> &t_pointsEdgeRight);

    /**
     * @brief 环岛执行（按状态改写 t_pointsEdge）
     * @param[in,out] t_pointsEdgeLeft      左透视边线
     * @param[in,out] t_pointsEdgeRight     右透视边线
     * @param[in,out] t_pointsEdgeLeft_size 左透视边线点数
     * @param[in,out] t_pointsEdgeRight_size 右透视边线点数
     * @param         is_L_left_found       左 L 角点找到标志
     * @param         is_L_right_found      右 L 角点找到标志
     * @param         t_L_pointLeft_id      左 L 角点下标
     * @param         t_L_pointRight_id     右 L 角点下标
     * @param         imgBinary             二值图
     */
    void Ring_Run(std::vector<POINT> &t_pointsEdgeLeft,
                  std::vector<POINT> &t_pointsEdgeRight,
                  int &t_pointsEdgeLeft_size, int &t_pointsEdgeRight_size,
                  bool is_L_left_found, bool is_L_right_found,
                  int t_L_pointLeft_id, int t_L_pointRight_id,
                  cv::Mat imgBinary);

    int entering_x0 = 0;
    int entering_y0 = 0;
    int exiting_x0  = 0;
    int exiting_y0  = 0;

    std::vector<POINT> ring_points;
    std::vector<POINT> last_inner_side;
    std::vector<POINT> last_outer_side;

    int ring_points_size = 0;
    int inside_ring_points_size = 0;

    std::vector<POINT> far_entering_edge;
    std::vector<POINT> far_exiting_edge;
    std::vector<POINT> t_far_entering_edge;
    std::vector<POINT> t_far_exiting_edge;
    std::vector<POINT> b_t_far_entering_edge;
    std::vector<POINT> b_t_far_exiting_edge;
    std::vector<POINT> s_b_t_far_entering_edge;
    std::vector<POINT> s_b_t_far_exiting_edge;

    int far_entering_edge_size       = 0;
    int far_exiting_edge_size        = 0;
    int t_far_entering_edge_size     = 0;
    int t_far_exiting_edge_size      = 0;
    int b_t_far_entering_edge_size   = 0;
    int b_t_far_exiting_edge_size    = 0;
    int s_b_t_far_entering_edge_size = 0;
    int s_b_t_far_exiting_edge_size  = 0;

    bool state_locking      = false;
    int  right_regain_count = 0;
    int  left_regain_count  = 0;
    int  right_edge_phase   = EDGE_OK;
    int  left_edge_phase    = EDGE_OK;
    int  ring_cooldown            = 0;
    int  left_entry_confirm_count  = 0;
    int  right_entry_confirm_count = 0;

    /** 行扫描结果与断线模式检测输出（最近一帧 Ring_None 调用刷新） */
    int  ring_left_x[ROI_H]        = {0};
    int  ring_right_x[ROI_H]       = {0};
    bool ring_findline_valid[ROI_H] = {false};
    bool ring_break_left           = false;
    bool ring_break_right          = false;
    /** [0]=左 [1]=右，断线模式上/下沿 y（命中时有效，未命中为 -1） */
    int  ring_break_y_top[2]       = {-1, -1};
    int  ring_break_y_bot[2]       = {-1, -1};

    int block_size = 9;
    int clip_value = 3;
    double pixel_per_meter = 88.88;
    double SAMPLE_DIST     = 0.02;
    double ROAD_WIDTH      = 0.45;

    const int dir_front[4][2]      = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2]  = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    /**
     * @brief 原图行扫描：每行从 x=160 向两侧找首个白→黑跳变点
     * @param img  ROI 二值图（CV_8UC1，COLSIMAGE×ROI_H = 320×130，相机视角）
     * @return 无
     * @sample ring.ring_findline(bin_img);
     * @note  写入 ring_left_x[ROI_H] / ring_right_x[ROI_H] / ring_findline_valid[ROI_H]。
     *        遍历 y=ROI_H-1→0（近→远）；中心 (y,160) 连续 RING_FINDLINE_CENTER_BLACK_STOP 行黑则早停。
     *        未扫描的远端行保持 valid=false 与哨兵 3/318。单行中心黑亦 valid=false。
     *        到边界仍未找到跳变点记 3/318 且 valid=true。用 img.ptr<uchar>(y) 行指针访问。
     */
    void ring_findline(const cv::Mat &img);

    /**
     * @brief 断线-缺口-断线 形态匹配（环岛进环辅助判据）
     * @param side  0=左，1=右
     * @param[out] y_bot  命中时填断线模式下沿 y（未命中保持 -1）
     * @param[out] y_top  命中时填断线模式上沿 y（未命中保持 -1）
     * @return true=命中（5-5-5 模式）
     * @sample int yb=-1, yt=-1; bool ok = ring.ring_breakline_check(0, yb, yt);
     * @note  调用前需先执行 ring_findline。3 段状态机：
     *        inside(N) → at_edge(N) → inside(N)，每段长度 RING_BREAK_RUN_LEN=5
     *        且容忍 RING_BREAK_BAD_TOL=1 行异常。扫描方向：y=ROI_H-1 → RING_BREAK_SCAN_Y_TOP=40。
     *        左侧 inside=x>10 / at_edge=x<=5；右侧 inside=x<309 / at_edge=x>=314。
     */
    bool ring_breakline_check(int side, int &y_bot, int &y_top);

    void findline_lefthand_adaptive(cv::Mat &img, int block_size, int clip_value,
                                    int x, int y,
                                    std::vector<POINT> &pointsEdgeLeft,
                                    int &pointsEdgeLeft_size);
    void findline_righthand_adaptive(cv::Mat &img, int block_size, int clip_value,
                                     int x, int y,
                                     std::vector<POINT> &pointsEdgeRight,
                                     int &pointsEdgeRight_size);

    void blur_points(int side, int kernel);
    void resample_points(std::vector<POINT> &in, int in_size,
                         std::vector<POINT> &out, int &out_size, float dist);

    void entering_track_far_line(cv::Mat imgBinary);
    void exiting_track_far_line(cv::Mat imgBinary);

#if 0 /* 旧版行扫描（ring_find_line），仅保留对照，运行时不用 */
    int rowstart = (ROI_H - 5);
    int rowup    = 5;
    int thresOTSU = 128;

    std::vector<POINT> pointsLeft;
    std::vector<POINT> pointsRight;
    std::vector<POINT> pointsMid;
    int pointsLeft_size  = 0;
    int pointsRight_size = 0;
    int pointsMid_size   = 0;

    bool L_left_down_found  = false;
    bool L_left_mid_found   = false;
    bool L_left_up_found    = false;
    bool L_right_down_found = false;
    bool L_right_mid_found  = false;
    bool L_right_up_found   = false;
    int  L_id_left_down  = 666;
    int  L_id_left_mid   = 666;
    int  L_id_left_up    = 666;
    int  L_id_right_down = 666;
    int  L_id_right_mid  = 666;
    int  L_id_right_up   = 666;

    void ring_find_line(cv::Mat &img, int y_start);
    cv::Point find_left_down();
    cv::Point find_right_down();
    cv::Point find_left_mid(int start);
    cv::Point find_right_mid(int start);
    cv::Point find_left_up();
    cv::Point find_right_up();
#endif

private:
    /**
     * @brief 边线丢线→复线边沿计数
     * @param size          当前边线点数
     * @param regain_count  累计复线次数（输出累加）
     * @param phase         边线相位 EDGE_OK / EDGE_LOST
     * @return 无
     * @note  size < LOST_LINE 记丢线；size >= REGAIN_LINE 且曾丢线则 regain_count++
     */
    void update_edge_regain(int size, int &regain_count, int &phase);
};

#endif /* RING_HPP */
