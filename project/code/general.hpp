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
 * @note 点透视 transf / Reverse_transf 使用本车实车标定单应矩阵 H（与 temp_repo 相同分式形式）；
 *       整图俯视 image_correction 仍用 Array_forward_bird_* 打表。
 */

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string>


/* ====================== 图像 / 控制全局参数 ====================== */

#define COLSIMAGE       320     /* 图像宽（列） */


#define ROWSIMAGE       240     /* 图像高（行） */


#define ROI_TOP         30      /* 巡线 ROI：裁去图像顶部行数 */


#define ROI_BOTTOM_CUT  80      /* 巡线 ROI：裁去图像底部行数 */


#define ROI_BOTTOM      (ROWSIMAGE - ROI_BOTTOM_CUT)    /* 巡线 ROI 底边 y（不含） = 160 */


#define ROI_H           (ROI_BOTTOM - ROI_TOP)          /* 巡线 ROI 高度 = 130 */
#define IMAGE_CUT_H     (ROI_BOTTOM - ROI_TOP)


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

  extern const unsigned short Array_forward_bird_row[41600];
  extern const unsigned short Array_forward_bird_col[41600];
  extern const unsigned short Array_backward_bird_row[41600];
  extern const unsigned short Array_backward_bird_col[41600];
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
 * @note  - change_un_Mat：本车实车标定 H，原图 ROI (i,j) → 俯视 (ri,rj)
 *        - Re_change_un_Mat：cv::invert(H)，俯视 → 原图
 *        - transf / Reverse_transf 与 temp_repo 相同 3×3 分式，边界用 ROI_H
 */
class General
{
public:
    cv::Mat rotation;

    /**
     * @brief 构造函数：载入实车单应矩阵 H 并求逆
     * @note  H 为原图列 i、行 j → 俯视列 x、行 y；与 temp_repo transf 形式一致
     */
    General()
    {
        change_un_Mat[0][0] =  1.1358;
        change_un_Mat[0][1] =  5.8272;
        change_un_Mat[0][2] = -17.4815;
        change_un_Mat[1][0] =  0.0000;
        change_un_Mat[1][1] = 5.2593;
        change_un_Mat[1][2] = -20.8889;
        change_un_Mat[2][0] =  0.000000;
        change_un_Mat[2][1] = 0.0358;
        change_un_Mat[2][2] =  1.000000;

        cv::Mat Hmat = (cv::Mat_<double>(3, 3) <<
            change_un_Mat[0][0], change_un_Mat[0][1], change_un_Mat[0][2],
            change_un_Mat[1][0], change_un_Mat[1][1], change_un_Mat[1][2],
            change_un_Mat[2][0], change_un_Mat[2][1], change_un_Mat[2][2]);
        cv::Mat Hinv;
        cv::invert(Hmat, Hinv);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                Re_change_un_Mat[r][c] = Hinv.at<double>(r, c);
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
     * @brief 原图 ROI 坐标 → 俯视图坐标（实车矩阵 H，对齐 temp_repo）
     * @param[out] ri 俯视图列坐标
     * @param[out] rj 俯视图行坐标
     * @param      i  原图列坐标
     * @param      j  原图行坐标（ROI 局部 0..ROI_H-1）
     * @return        true 落在 [0,COLSIMAGE)×[0,ROI_H)；false 越界或 w≈0
     * @sample        int a, b; if (general.transf(a, b, x, y)) { ... }
     */
    bool transf(int &ri, int &rj, int i, int j) const
    {
        if (i < 0 || i >= COLSIMAGE || j < 0 || j >= ROI_H)
            return false;

        const double w = change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2];
        if (std::fabs(w) < 1e-9)
            return false;

        const int local_x = static_cast<int>(
            (change_un_Mat[0][0] * i + change_un_Mat[0][1] * j + change_un_Mat[0][2]) / w);
        const int local_y = static_cast<int>(
            (change_un_Mat[1][0] * i + change_un_Mat[1][1] * j + change_un_Mat[1][2]) / w);

        if (local_x >= 0 && local_x < COLSIMAGE && local_y >= 0 && local_y < ROI_H)
        {
            ri = local_x;
            rj = local_y;
            return true;
        }
        return false;
    }

    /**
     * @brief 俯视图坐标 → 原图 ROI 坐标（H 的逆矩阵）
     * @param[out] ri 原图列坐标
     * @param[out] rj 原图行坐标
     * @param      i  俯视图列坐标
     * @param      j  俯视图行坐标
     * @return        true 落在 [0,COLSIMAGE)×[0,ROI_H)；false 越界或 w≈0
     * @sample        int a, b; general.Reverse_transf(a, b, tx, ty);
     */
    bool Reverse_transf(int &ri, int &rj, int i, int j) const
    {
        if (i < 0 || i >= COLSIMAGE || j < 0 || j >= ROI_H)
            return false;

        const double w = Re_change_un_Mat[2][0] * i + Re_change_un_Mat[2][1] * j + Re_change_un_Mat[2][2];
        if (std::fabs(w) < 1e-9)
            return false;

        const int local_x = static_cast<int>(
            (Re_change_un_Mat[0][0] * i + Re_change_un_Mat[0][1] * j + Re_change_un_Mat[0][2]) / w);
        const int local_y = static_cast<int>(
            (Re_change_un_Mat[1][0] * i + Re_change_un_Mat[1][1] * j + Re_change_un_Mat[1][2]) / w);

        if (local_x >= 0 && local_x < COLSIMAGE && local_y >= 0 && local_y < ROI_H)
        {
            ri = local_x;
            rj = local_y;
            return true;
        }
        return false;
    }

private:
    double change_un_Mat[3][3]{};      /* 原图 → 俯视 */
    double Re_change_un_Mat[3][3]{};   /* 俯视 → 原图 */
};

#endif /* GENERAL_HPP */
