#ifndef IMGPROC_HPP
#define IMGPROC_HPP

/**
 * @file imgproc.hpp
 * @brief 图像预处理类（移植自 temp_repo/track/imgprocess/imgprocess.h）
 *
 * 提供：
 *  - ImageProcess::processImage()   : BGR → 灰度 → OTSU 二值化 → 5x5 矩形核闭运算
 *  - ImageProcess::mapPerspective() : 与 General::transf/Reverse_transf 等价的备用透视
 *  - ImageProcess::matilluminationChange() : 高光去除（去强光斑反光，调试时可选用）
 *
 * @note 本工程的主流水线把灰度化 + OTSU + 闭运算直接写在 image.cpp 的 image_process()
 *       里以减少 cv::Mat 拷贝；ImageProcess 类保留作为独立调用入口（如 NCNN 前的
 *       BGR→Gray 转换）。
 */

#include "general.hpp"
#include <opencv2/opencv.hpp>
extern unsigned short const Array_forward_bird_row[41600];
extern unsigned short const Array_forward_bird_col[41600];
extern unsigned short const Array_backward_bird_row[41600];
extern unsigned short const Array_backward_bird_col[41600];


/**
 * @brief 图像预处理类
 * @sample ImageProcess ip; cv::Mat bin = ip.processImage(rgb_img);
 */
class ImageProcess
{
private:
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

public:
    ImageProcess();

    /**
     * @brief BGR 图像 → 二值图（OTSU + 5x5 闭运算）
     * @param src_img 输入 BGR 图像（通常为 320x240）
     * @return        OTSU 二值化 + 闭运算后的灰度二值图
     * @sample        cv::Mat bin = ip.processImage(rgb_img);
     */
    cv::Mat processImage(const cv::Mat &src_img);
    cv::Mat image_correction(cv::Mat raw );

    /**
     * @brief 透视变换（备用，与 General::transf/Reverse_transf 等价）
     * @param x    输入坐标 x
     * @param y    输入坐标 y
     * @param loc  [out] 变换后坐标 [x', y']
     * @param mode 0：原图 → 俯视；1：俯视 → 原图
     * @note  本工程主流水线统一用 General 类，本接口仅为兼容 temp_repo 保留
     */
    void mapPerspective(float x, float y, float loc[2], uint8_t mode);

    /**
     * @brief 高光抑制（调试用：当摄像头出现严重反光时启用）
     * @param src BGR 图像
     * @return    去高光后的 BGR 图像
     * @note      内部基于 OpenCV illuminationChange，CPU 较重，正常巡线不调用
     */
    cv::Mat matilluminationChange(cv::Mat src);
};

#endif /* IMGPROC_HPP */
