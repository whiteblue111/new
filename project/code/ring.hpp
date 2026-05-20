#ifndef RING_HPP
#define RING_HPP

/**
 * @file ring.hpp
 * @brief 环岛处理类
 *
 * 8 态状态机（左 4 + 右 4 + None）：
 *   左环：None → Find(巡右) → Begin(巡左) → In(巡左) → Out(巡右) → None
 *   右环：镜像（Find 巡左，Begin/In 巡右，Out 巡左）
 *
 * Ring_Check 负责状态迁移；Ring_Run 按状态裁边供 fitting() 单边中线。
 */

#include "general.hpp"
#include <opencv2/opencv.hpp>


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
        Left_Ring_Find,
        Left_Ring_Begin,
        Left_Ring_In,
        Left_Ring_Out,
        Right_Ring_Find,
        Right_Ring_Begin,
        Right_Ring_In,
        Right_Ring_Out
    };
    flag_Ring_e flag_ring = Ring_None;

    /** 标定阈值（实车可改） */
    static const int LOST_LINE         = 10;
    static const int REGAIN_LINE       = 10;
    static const int L_SMALL           = 15;
    static const int R_LARGE           = 25;
    static const int Y_FIND_TO_BEGIN   = 65;
    static const int RING_COOLDOWN_MAX = 150;

    Ring();

    /**
     * @brief 将环岛状态机复位为 Ring_None，并清零计数器与角点缓存
     * @return 无
     * @sample ring.reset();
     * @note  供按键/调试调用；不修改 ring_cooldown
     */
    void reset();

    /**
     * @brief 环岛识别（状态机迁移）
     * @param imgBinary               二值图（CV_8UC1，320x240）
     * @param is_left_straight        左侧是否直道
     * @param is_right_straight       右侧是否直道
     * @param L_left_found            左 L 角点找到（外部）
     * @param L_right_found           右 L 角点找到（外部）
     * @param t_pointsEdgeLeft_size   左透视边线点数
     * @param t_pointsEdgeRight_size  右透视边线点数
     * @param is_L_left_found         左 L 角点找到
     * @param is_L_right_found        右 L 角点找到
     * @param t_L_pointLeft_id        左 L 角点下标
     * @param t_L_pointRight_id       右 L 角点下标
     * @param t_pointsEdgeLeft        左透视边线
     * @param t_pointsEdgeRight       右透视边线
     */
    void Ring_Check(cv::Mat &imgBinary,
                    bool is_left_straight, bool is_right_straight,
                    bool L_left_found, bool L_right_found,
                    int t_pointsEdgeLeft_size, int t_pointsEdgeRight_size,
                    bool is_L_left_found, bool is_L_right_found,
                    int t_L_pointLeft_id, int t_L_pointRight_id,
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

    /* ===== 行扫描参数（bin_img 320x130，ROI 局部行号） ===== */
    int rowstart = (ROI_H - 5);
    int rowup    = 5;

    int thresOTSU = 128;

    int entering_x0 = 0;
    int entering_y0 = 0;
    int exiting_x0  = 0;
    int exiting_y0  = 0;

    std::vector<POINT> pointsLeft;
    std::vector<POINT> pointsRight;
    std::vector<POINT> pointsMid;
    std::vector<POINT> ring_points;
    std::vector<POINT> last_inner_side;
    std::vector<POINT> last_outer_side;

    int pointsLeft_size  = 0;
    int pointsRight_size = 0;
    int pointsMid_size   = 0;
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

    bool state_locking      = false;
    int  right_regain_count = 0;
    int  left_regain_count  = 0;
    int  right_edge_phase   = EDGE_OK;
    int  left_edge_phase    = EDGE_OK;
    int  ring_cooldown      = 0;

    int block_size = 9;
    int clip_value = 3;
    double pixel_per_meter = 88.88;
    double SAMPLE_DIST     = 0.02;
    double ROAD_WIDTH      = 0.45;

    const int dir_front[4][2]      = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2]  = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    void ring_find_line(cv::Mat &img, int y_start);

    cv::Point find_left_down();
    cv::Point find_right_down();
    cv::Point find_left_mid(int start);
    cv::Point find_right_mid(int start);
    cv::Point find_left_up();
    cv::Point find_right_up();

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
