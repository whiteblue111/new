#ifndef RING_HPP
#define RING_HPP

/**
 * @file ring.hpp
 * @brief 环岛处理类（移植自 temp_repo/track/basic/ring.h）
 *
 * 10 态状态机（左 + 右 + None）：
 *   None → (L|R)_pre_Entering → (L|R)_Entering → (L|R)_Inside
 *        → (L|R)_Exiting → (L|R)_Finish → None
 *
 * Ring_Check 仅负责状态机的迁移（基于角点 / 直道 / 边线长度等线索判定），
 * Ring_Run   负责按状态修改 t_pointsEdgeLeft / t_pointsEdgeRight 边线点集，
 *            进 / 出环岛时分别 entering_track_far_line / exiting_track_far_line。
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

    /** 环岛状态枚举 */
    enum flag_Ring_e
    {
        Ring_None = 0,
        Left_Ring_pre_Entering,
        Left_Ring_Entering,
        Left_Ring_Inside,
        Left_Ring_Exiting,
        Left_Ring_Finish,
        Right_Ring_pre_Entering,
        Right_Ring_Entering,
        Right_Ring_Inside,
        Right_Ring_Exiting,
        Right_Ring_Finish
    };
    flag_Ring_e flag_ring = Ring_None;

    Ring();

    /** 重置内部状态（计数器、角点查找标志等） */
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
     * @param is_L_left_found         左 L 角点找到（重复参数，与原 API 对齐）
     * @param is_L_right_found        右 L 角点找到（重复参数）
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
    int rowstart = (ROI_H - 5);         /* 125，局部底行 */
    int rowup    = 5;                   /* 局部顶行缓冲 */

    int thresOTSU = 128;

    int entering_x0 = 0;
    int entering_y0 = 0;
    int exiting_x0  = 0;
    int exiting_y0  = 0;

    /* 行扫描得到的“整行左右白边”点集 */
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

    /* 进 / 出环岛远线工作缓存 */
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

    /* L 角点查找结果 */
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

    cv::Point exit_left_mid;
    cv::Point exit_left_up;
    cv::Point exit_right_up;
    cv::Point exit_right_mid;
    cv::Point last_Left_up;

    int ring_inside_counter   = 0;
    int pre_counter           = 0;
    int exitingnum            = 0;
    int ring_entering_counter = 0;
    int inside_exit_flag      = 0;
    int pre_entering_flag     = 0;

    int left_no_size  = 0;
    int right_no_size = 0;

    int mid_up_dis      = 0;
    int mid_block       = 0;
    int ring_points_flag = 0;

    bool entering_lose_line_flag = false;
    bool exiting_loseline_flag   = false;

    int block_size = 9;
    int clip_value = 3;
    double pixel_per_meter = 88.88;
    double SAMPLE_DIST     = 0.02;
    double ROAD_WIDTH      = 0.45;

    const int dir_front[4][2]      = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2]  = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    /**
     * @brief 行扫描：从中点向左右两侧找黑白跳变得到左右行白边
     * @param img     二值图
     * @param y_start 起始行（实际未使用，保留与 temp_repo 兼容）
     * @note          扫描行范围 [rowup, rowstart)，写入 pointsLeft/Right/Mid
     */
    void ring_find_line(cv::Mat &img, int y_start);

    cv::Point find_left_down();
    cv::Point find_right_down();
    cv::Point find_left_mid(int start);
    cv::Point find_right_mid(int start);
    cv::Point find_left_up();
    cv::Point find_right_up();

    /** 左手迷宫法 */
    void findline_lefthand_adaptive(cv::Mat &img, int block_size, int clip_value,
                                    int x, int y,
                                    std::vector<POINT> &pointsEdgeLeft,
                                    int &pointsEdgeLeft_size);
    /** 右手迷宫法 */
    void findline_righthand_adaptive(cv::Mat &img, int block_size, int clip_value,
                                     int x, int y,
                                     std::vector<POINT> &pointsEdgeRight,
                                     int &pointsEdgeRight_size);

    /** side: 0=entering, 1=exiting */
    void blur_points(int side, int kernel);
    void resample_points(std::vector<POINT> &in, int in_size,
                         std::vector<POINT> &out, int &out_size, float dist);

    /** 进环远线巡线（pivot=entering_x0/y0） */
    void entering_track_far_line(cv::Mat imgBinary);
    /** 出环远线巡线（pivot=exiting_x0/y0） */
    void exiting_track_far_line(cv::Mat imgBinary);
};

#endif /* RING_HPP */
