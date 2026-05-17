#ifndef GENERAL_HPP
#define GENERAL_HPP

/**
 * @file general.hpp
 * @brief 巡线通用工具类（移植自 temp_repo/track/standard/general.h）
 *
 * 提供：
 *  - POINT 结构体（vector<POINT> 在整个巡线流水线里使用）
 *  - 巡线相关公用宏（图像尺寸 / 舵机 PWM 极限 / 点序列上限等）
 *  - General 类（clip / Bezier / sigma / filter / pid_realize_a / transf / Reverse_transf）
 *
 * @note 透视矩阵 src_points / dst_points 来自 temp_repo 的 Edgeboard 摄像头标定，
 *       本车摄像头视角不同，几何变换会失真。先按 temp_repo 数值占位，
 *       TODO: 用本车摄像头采图 + 棋盘格重新标定。
 */

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string>


/* ====================== 图像 / 控制全局参数 ====================== */
#ifndef COLSIMAGE
#define COLSIMAGE       320     /* 图像宽（列） */
#endif
#ifndef ROWSIMAGE
#define ROWSIMAGE       240     /* 图像高（行） */
#endif
#ifndef ROI_TOP
#define ROI_TOP         40      /* 巡线 ROI：裁去图像顶部行数 */
#endif
#ifndef ROI_BOTTOM_CUT
#define ROI_BOTTOM_CUT  80      /* 巡线 ROI：裁去图像底部行数 */
#endif
#ifndef ROI_BOTTOM
#define ROI_BOTTOM      (ROWSIMAGE - ROI_BOTTOM_CUT)    /* 巡线 ROI 底边 y（不含） = 160 */
#endif
#ifndef ROI_H
#define ROI_H           (ROI_BOTTOM - ROI_TOP)          /* 巡线 ROI 高度 = 120 */
#endif

/* 单边巡线最大点数（迷宫法步数上限） */
#ifndef POINTS_MAX_LEN
#define POINTS_MAX_LEN  180
#endif

/* 舵机 PWM 极限（与 motor.cpp 接口保持解耦，仅作占位） */
#ifndef PWMSERVOMAX
#define PWMSERVOMAX     4700
#endif
#ifndef PWMSERVOMID
#define PWMSERVOMID     3840
#endif
#ifndef PWMSERVOMIN
#define PWMSERVOMIN     3000
#endif

#ifndef PI
#define PI              (3.1415926535898f)
#endif

  extern const unsigned short Array_forward_bird_row[38400];
  extern const unsigned short Array_forward_bird_col[38400];
  extern const unsigned short Array_backward_bird_row[38400];
  extern const unsigned short Array_backward_bird_col[38400];
/**
 * @brief 二维边线点结构体
 * @note 在巡线流水线中复用：原图坐标 / 透视坐标 / 角度计算结果 都用同一结构
 *       x,y 为像素坐标；angle 由 local_angle_points 写入；slope 预留给曲率拟合
 */
struct POINT
{
    int   x      = 0;
    int   y      = 0;
    float slope  = 0.0f;
    float angle  = 0.0f;

    POINT() = default;
    POINT(int xx, int yy) : x(xx), y(yy) {}
};


/**
 * @brief 通用工具类：透视变换 + 数值工具
 * @note  - 构造时根据 src_points / dst_points 计算 change_un_Mat / Re_change_un_Mat；
 *        - transf()       : 原图坐标 → 俯视图坐标
 *        - Reverse_transf(): 俯视图坐标 → 原图坐标
 *        - 透视矩阵参数从 temp_repo/track/standard/general.h 平移，TODO 重新标定
 */
class General
{
public:
    cv::Mat rotation;

    /**
     * @brief 构造函数：根据 src_points/dst_points 自动计算正反透视矩阵
     * @note 4 个角点取自 temp_repo 的 Edgeboard 摄像头标定，TODO 重新标定到本车摄像头
     */
    General()
    {
        cv::Point2f src_points[4];
        cv::Point2f dst_points[4];

        /* ---- 反向矩阵 Re_change_un_Mat：俯视 → 原图 ---- */
        dst_points[0] = cv::Point2f(123.0f, 105.0f);
        dst_points[1] = cv::Point2f(195.0f, 104.0f);
        dst_points[2] = cv::Point2f(102.0f, 135.0f);
        dst_points[3] = cv::Point2f(211.0f, 134.0f);
        src_points[0] = cv::Point2f(110.0f,  40.0f);
        src_points[1] = cv::Point2f(210.0f,  40.0f);
        src_points[2] = cv::Point2f(110.0f, 140.0f);
        src_points[3] = cv::Point2f(210.0f, 140.0f);
        rotation = cv::getPerspectiveTransform(src_points, dst_points);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Re_change_un_Mat[i][j] = rotation.at<double>(i, j);

        /* ---- 正向矩阵 change_un_Mat：原图 → 俯视 ---- */
        src_points[0] = cv::Point2f(123.0f, 105.0f);
        src_points[1] = cv::Point2f(195.0f, 104.0f);
        src_points[2] = cv::Point2f(102.0f, 135.0f);
        src_points[3] = cv::Point2f(211.0f, 134.0f);
        dst_points[0] = cv::Point2f(110.0f,  40.0f);
        dst_points[1] = cv::Point2f(210.0f,  40.0f);
        dst_points[2] = cv::Point2f(110.0f, 140.0f);
        dst_points[3] = cv::Point2f(210.0f, 140.0f);
        rotation = cv::getPerspectiveTransform(src_points, dst_points);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                change_un_Mat[i][j] = rotation.at<double>(i, j);
    }

    /**
     * @brief 整数限幅
     * @param x   待限幅值
     * @param low 下界（含）
     * @param up  上界（含）
     * @return    限幅后的值
     * @sample    int idx = general.clip(raw, 0, size - 1);
     */
    int clip(int x, int low, int up) const
    {
        return x > up ? up : (x < low ? low : x);
    }

    /**
     * @brief 阶乘
     * @param x  非负整数
     * @return   x!
     * @note     Bezier 的二项式系数会用到，n ≤ 6 时无溢出风险
     */
    int factorial(int x) const
    {
        int f = 1;
        for (int i = 1; i <= x; i++) f *= i;
        return f;
    }

    /**
     * @brief 贝塞尔曲线采样
     * @param dt    采样步长（0~1，越小越密）
     * @param input 控制点序列（n+1 阶贝塞尔即 input.size() = n+1）
     * @return      采样得到的曲线点序列
     * @sample      auto pts = general.Bezier(0.01, ctrl4);
     * @note        用于双边巡线拟合中线（track_both_edge）
     */
    std::vector<POINT> Bezier(double dt, std::vector<POINT> input) const
    {
        std::vector<POINT> output;
        double t = 0;
        while (t <= 1)
        {
            POINT  p;
            double x_sum = 0.0;
            double y_sum = 0.0;
            int    n     = (int)input.size() - 1;
            for (int i = 0; i <= n; i++)
            {
                double k = factorial(n) / (double)(factorial(i) * factorial(n - i))
                         * std::pow(t, i) * std::pow(1 - t, n - i);
                x_sum += k * input[i].x;
                y_sum += k * input[i].y;
            }
            p.x = (int)x_sum;
            p.y = (int)y_sum;
            output.push_back(p);
            t += dt;
        }
        return output;
    }

    /**
     * @brief 计算指定子区间内 x 方差
     * @param vec  点序列
     * @param n    起始下标（含）
     * @param m    结束下标（不含）
     * @return     [n, m) 区间内 x 方差；当区间无效或长度 < 1 返回 0
     */
    double sigma(std::vector<POINT> vec, int n, int m) const
    {
        if (vec.size() < 1 || m <= n) return 0;
        double sum = 0;
        for (int i = n; i < m; i++) sum += vec[i].x;
        double aver = (double)sum / (m - n);
        double s = 0;
        for (int i = n; i < m; i++) s += (vec[i].x - aver) * (vec[i].x - aver);
        s /= (double)(m - n);
        return s;
    }

    /**
     * @brief 3 点滑动平均滤波器（用于偏差量）
     * @param value 当前帧偏差
     * @return      滤波后的偏差
     * @note        内部 static 缓冲跨调用保存历史，多实例间会串扰，使用时注意
     */
    float filter(float value)
    {
        static float filter_buf[3] = {0};
        filter_buf[2] = filter_buf[1];
        filter_buf[1] = filter_buf[0];
        filter_buf[0] = value;
        return (filter_buf[2] + filter_buf[1] + filter_buf[0]) / 3.0f;
    }

    /**
     * @brief 位置式 PD（角度外环），含不完全微分
     * @param actual 当前测量值
     * @param set    目标值
     * @param _p     比例系数
     * @param _d     微分系数
     * @return       PD 输出
     * @sample       float u = general.pid_realize_a(aim_angle, 0.0f, kp, kd);
     */
    float pid_realize_a(float actual, float set, float _p, float _d)
    {
        static float last_error = 0.0f;
        static float last_out_d = 0.0f;

        float error = set - actual;
        /* 不完全微分 */
        float out_d  = 0.7f * _d * (error - last_error) + 0.3f * last_out_d;
        float output = _p * error + out_d;
        last_error = error;
        last_out_d = out_d;
        return output;
    }

    /**
     * @brief 原图坐标 → 俯视图坐标
     * @param[out] ri 俯视图列坐标
     * @param[out] rj 俯视图行坐标
     * @param      i  原图列坐标
     * @param      j  原图行坐标
     * @return        true 表示落在俯视图有效区 [0, COLSIMAGE)×[0, ROWSIMAGE)；
     *                false 表示越界（ri/rj 不更新）
     * @sample        int a, b;
     *                if (general.transf(a, b, x, y)) { ... }
     */
    // 建议加上 inline 关键字，进一步提升高频调用的速度
    bool transf(int &ri, int &rj, int i, int j) const
    {
        // 安全防御：防止传入的 i, j 超出打表范围导致内存越界
        // 如果你在外层循环严格控制了 i 和 j 的范围，这两行可以注释掉以压榨极致性能
        if (i < 0 || i >= COLSIMAGE || j < 0 || j >= ROI_H) 
                return false;
        int index = j * COLSIMAGE + i;
        int mapped_x = Array_forward_bird_row[index];
        int mapped_y = Array_forward_bird_col[index];

        // 如果表里存的是 -1，说明初始化时判定该点无效或越界
        if (mapped_x == -1) 
        {
            return false;
        }

        // 赋值并返回成功
        ri = mapped_x;
        rj = mapped_y;
        return true;
    }
    

    /**
     * @brief 俯视图坐标 → 原图坐标
     * @param[out] ri 原图列坐标（始终更新，即使越界）
     * @param[out] rj 原图行坐标（始终更新，即使越界）
     * @param      i  俯视图列坐标
     * @param      j  俯视图行坐标
     * @return        恒为 true（保留 temp_repo 行为）
     * @note          落在 [0, COLSIMAGE)×[0, ROWSIMAGE) 才视为有效，但越界也会
     *                把计算结果写回（与 temp_repo 行为一致）
     */
    bool Reverse_transf(int &ri, int &rj, int i, int j) const
    {
        if (i < 0 || i >= COLSIMAGE || j < 0 || j >= ROI_H) 
                return false;
        int index = j * COLSIMAGE + i;
        int mapped_x = Array_backward_bird_row[index];
        int mapped_y = Array_backward_bird_col[index];

        // 如果表里存的是 -1，说明初始化时判定该点无效或越界
        if (mapped_x == -1) 
        {
            return false;
        }

        // 赋值并返回成功
        ri = mapped_x;
        rj = mapped_y;
        return true;
    }

private:
    double change_un_Mat[3][3]{};      /* 原图 → 俯视 */
    double Re_change_un_Mat[3][3]{};   /* 俯视 → 原图 */
};

#endif /* GENERAL_HPP */
