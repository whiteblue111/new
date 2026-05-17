#ifndef CROSS_HPP
#define CROSS_HPP

/**
 * @file cross.hpp
 * @brief 十字路口处理类（移植自 temp_repo/track/basic/cross.h）
 *
 * 三段式状态机：
 *  - Cross_None  : 非十字
 *  - Cross_Begin : 已识别到十字下半区（双 L 角点），截断 t_pointsEdge 到角点行
 *  - Cross_Out   : 出十字，调用 cross_find_farline 重新巡远线
 *
 * 与原 elements.cpp 的 4 点补线方案完全不同：
 *  - 十字判定依赖 standard 主流水线已找到的 L 角点（t_L_pointLeft / t_L_pointRight），
 *    不再用 border 单调突变检测
 *  - 出十字时基于角点位置外推一个起始点，再次跑迷宫法找远端边线
 *  - 自带 blur / resample / 角度 / NMS / 找拐点的完整后处理
 */

#include "general.hpp"
#include <opencv2/opencv.hpp>


/**
 * @brief 十字路口处理类
 * @sample Cross cross;
 *         cross.Cross_Check(...);
 *         cross.Cross_Run(...);
 */
class Cross
{
public:
    General general;

    /** 十字主状态 */
    enum flag_Cross_e
    {
        Cross_None  = 0,    /* 非十字 */
        Cross_Begin = 1,    /* 进十字（已识别） */
        Cross_Out   = 2     /* 出十字（远线重寻中） */
    };
    flag_Cross_e flag_cross = Cross_None;

    /** 十字进入方向（保留，状态机暂未用到） */
    enum state_Cross_e
    {
        Cross_Left,
        Cross_Right,
        Cross_Both
    };
    state_Cross_e state_cross = Cross_Both;

    Cross();

    /**
     * @brief 十字识别（仅状态转移，不修改边线）
     * @param is_L_left_found   左 L 角点是否找到
     * @param is_L_right_found  右 L 角点是否找到
     * @param imgBinary         二值图（CV_8UC1，320x240）
     * @param t_L_pointLeft     左 L 角点（透视后坐标）
     * @param t_L_pointRight    右 L 角点（透视后坐标）
     * @param t_L_pointLeft_id  左 L 角点在 t_pointsEdgeLeft 中的下标
     * @param t_L_pointRight_id 右 L 角点在 t_pointsEdgeRight 中的下标
     * @param t_pointsEdgeLeft_size  左透视边线点数
     * @param t_pointsEdgeRight_size 右透视边线点数
     * @sample cross.Cross_Check(L_l_found, L_r_found, bin, L_pt_l, L_pt_r, L_l_id, L_r_id, lsz, rsz);
     */
    void Cross_Check(bool is_L_left_found, bool is_L_right_found,
                     cv::Mat imgBinary,
                     cv::Point t_L_pointLeft, cv::Point t_L_pointRight,
                     int t_L_pointLeft_id, int t_L_pointRight_id,
                     int t_pointsEdgeLeft_size, int t_pointsEdgeRight_size);

    /**
     * @brief 十字执行（Cross_Begin 截断边线、Cross_Out 巡远线）
     * @param[in,out] t_pointsEdgeLeft  透视后左边线（可能被改写）
     * @param[in,out] t_pointsEdgeRight 透视后右边线（可能被改写）
     * @param         img                二值图
     * @param         is_L_left_found    左 L 角点找到标志
     * @param         is_L_right_found   右 L 角点找到标志
     * @param         t_L_pointLeft_id   左 L 角点下标
     * @param         t_L_pointRight_id  右 L 角点下标
     * @param[in,out] t_pointsEdgeLeft_size  左边线点数（可能被改写）
     * @param[in,out] t_pointsEdgeRight_size 右边线点数（可能被改写）
     */
    void Cross_Run(std::vector<POINT> &t_pointsEdgeLeft, std::vector<POINT> &t_pointsEdgeRight,
                   cv::Mat img,
                   bool is_L_left_found, bool is_L_right_found,
                   int t_L_pointLeft_id, int t_L_pointRight_id,
                   int &t_pointsEdgeLeft_size, int &t_pointsEdgeRight_size);

public:
    int Cross_counter = 0;

    /* ----------- 远线巡线工作缓存 ---------- */
    std::vector<POINT> far_pointsEdgeLeft;
    std::vector<POINT> far_pointsEdgeRight;
    std::vector<POINT> far_t_pointsEdgeLeft;
    std::vector<POINT> far_t_pointsEdgeRight;
    std::vector<POINT> far_b_t_pointsEdgeLeft;
    std::vector<POINT> far_b_t_pointsEdgeRight;
    std::vector<POINT> far_s_b_t_pointsEdgeLeft;
    std::vector<POINT> far_s_b_t_pointsEdgeRight;
    std::vector<POINT> far_a_t_pointsEdgeLeft;
    std::vector<POINT> far_a_t_pointsEdgeRight;
    std::vector<POINT> far_n_a_t_pointsEdgeLeft;
    std::vector<POINT> far_n_a_t_pointsEdgeRight;

    int far_pointsEdgeLeft_size       = 0;
    int far_pointsEdgeRight_size      = 0;
    int far_t_pointsEdgeLeft_size     = 0;
    int far_t_pointsEdgeRight_size    = 0;
    int far_b_t_pointsEdgeLeft_size   = 0;
    int far_b_t_pointsEdgeRight_size  = 0;
    int far_s_b_t_pointsEdgeLeft_size = 0;
    int far_s_b_t_pointsEdgeRight_size = 0;
    int far_a_t_pointsEdgeLeft_size   = 0;
    int far_a_t_pointsEdgeRight_size  = 0;
    int far_n_a_t_pointsEdgeLeft_size = 0;
    int far_n_a_t_pointsEdgeRight_size = 0;

    int far_t_L_pointLeft_id  = 0;
    int far_t_L_pointRight_id = 0;

    /* L 角点置信度门限（角度，单位：度） */
    int Lconf_Min = 70;
    int Lconf_Max = 130;

    int left_cross_in_counter  = 0;
    int right_cross_in_counter = 0;
    int both_L_find_counter    = 0;

    int far_left_x0  = COLSIMAGE / 4;
    int far_left_y0  = ROI_TOP + 30;
    int far_right_x0 = COLSIMAGE * 3 / 4;
    int far_right_y0 = ROI_TOP + 30;

    int    approx_num      = 3;
    int    block_size      = 9;
    int    clip_value      = 3;
    double pixel_per_meter = 222.222;
    double SAMPLE_DIST     = 0.02;
    double ROAD_WIDTH      = 0.45;
    double dist            = pixel_per_meter * ROAD_WIDTH / 2.0;

    bool is_far_t_L_pointLeft_find  = false;
    bool is_far_t_L_pointRight_find = false;
    bool L_left_found  = false;
    bool L_right_found = false;
    bool is_make_line  = false;

    int no_line_counter = 0;

    /** 迷宫法方向向量（参考 temp_repo） */
    const int dir_front[4][2]      = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2]  = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    /**
     * @brief 用最小二乘法计算 pointsEdge[begin, end) 区间斜率
     * @param begin     起点（含）
     * @param end       终点（不含）
     * @param pointsEdge 点序列
     * @return           斜率（若除数为 0 返回上一次值）
     */
    float Slope_Calculate(int begin, int end, std::vector<POINT> &pointsEdge);

    /**
     * @brief 计算 pointsEdge[start, end) 的斜率和截距
     * @param start       起点（含）
     * @param end         终点（不含）
     * @param pointsEdge  点序列
     * @param[out] slope_rate 斜率
     * @param[out] intercept  截距
     */
    void calculate_s_i(int start, int end, std::vector<POINT> &pointsEdge,
                       float &slope_rate, float &intercept);

private:
    /**
     * @brief 远线巡线主流程：基于 L 角点外推起点，迷宫法巡新边线 + 后处理
     * @param img               二值图
     * @param is_L_left_found   左 L 角点找到标志
     * @param is_L_right_found  右 L 角点找到标志
     * @param t_pointsEdgeLeft  原透视左边线
     * @param t_pointsEdgeRight 原透视右边线
     * @param t_L_pointLeft_id  左 L 角点下标
     * @param t_L_pointRight_id 右 L 角点下标
     */
    void cross_find_farline(cv::Mat &img, bool is_L_left_found, bool is_L_right_found,
                            std::vector<POINT> t_pointsEdgeLeft,
                            std::vector<POINT> t_pointsEdgeRight,
                            int t_L_pointLeft_id, int t_L_pointRight_id);

    /** 左手迷宫法（自适应阈值版） */
    void findline_lefthand_adaptive(cv::Mat &img, int block_size, int clip_value,
                                    int x, int y,
                                    std::vector<POINT> &pointsEdgeLeft,
                                    int &pointsEdgeLeft_size);
    /** 右手迷宫法（自适应阈值版） */
    void findline_righthand_adaptive(cv::Mat &img, int block_size, int clip_value,
                                     int x, int y,
                                     std::vector<POINT> &pointsEdgeRight,
                                     int &pointsEdgeRight_size);

    /** 边线三角窗口平滑 */
    void blur_points(int side, int kernel);

    /** 边线等距采样 */
    void resample_points(std::vector<POINT> &in, int in_size,
                         std::vector<POINT> &out, int &out_size, float dist);

    /** 找远端 L 角点 */
    void find_corners();

    /** 局部角度计算 */
    void local_angle_points(std::vector<POINT> pointsEdgeIn, int size,
                            std::vector<POINT> &pointsEdgeOut, int dist);

    /** 角度非极大值抑制 */
    void nms_angle(std::vector<POINT> &in, int in_size,
                   std::vector<POINT> &out, int kernel);
};

#endif /* CROSS_HPP */
