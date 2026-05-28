#ifndef BANMAXIAN_HPP
#define BANMAXIAN_HPP

/**
 * @file banmaxian.hpp
 * @brief 斑马线识别模块（仅检测，不参与控制）
 *
 * 思路来源：传统巡线代码 stop() 的逐行跳变点扫描。
 * 本工程适配：
 *  - 直接在 ROI 二值图 bin_img（320x130，OTSU+闭运算）上扫描；
 *  - 用户要求左右 20 列像素不参与（避开边线毛刺）；
 *  - 必须在十字状态机至少出现过一次 Cross_Begin 之后才解锁工作；
 *  - 命中后仅在 0→1 上升沿向终端 printf 一次，不动 motor / scene / 边线。
 */

#include <opencv2/opencv.hpp>


/* ====================== 全局状态（供 HUD / 调试读取） ====================== */

/** @brief 当前帧（去抖后）是否识别到斑马线，0 / 1 */
extern int banmaxian_flag;

/** @brief 累计触发次数（每个 0→1 上升沿 +1） */
extern int banmaxian_total_cnt;


/* ====================== 调试快照 ====================== */

/**
 * @brief 斑马线模块调试快照
 */
struct BanmaxianDebugSnapshot
{
    int armed;            /**< 是否已被 Cross_Begin 解锁，0 / 1 */
    int flag;             /**< 当前帧去抖后的命中标志 */
    int total_cnt;        /**< 累计上升沿次数 */
    int last_max_trans;   /**< 最近一次扫描中单行最大跳变数 */
    int last_hit_row;     /**< 最近一次扫描中跳变数最大行的 ROI y 坐标，未命中为 -1 */
};


/* ====================== 接口 ====================== */

/**
 * @brief 复位斑马线模块：清解锁 latch、命中计数与去抖计数器
 * @return 无
 * @sample banmaxian_reset();
 * @note   供 track_elements_reset() 调用，与 s_cross.reset() / s_ring.reset() 同步
 */
void banmaxian_reset(void);

/**
 * @brief 每帧调用一次：在 bin_img 上做斑马线识别
 * @param bin_img    [in] ROI 二值图（CV_8UC1，320x130，0/255）
 * @param cross_flag [in] 当前十字状态机 flag（Cross::flag_Cross_e 转 int）
 * @return 无；结果写入全局 banmaxian_flag / banmaxian_total_cnt
 * @sample banmaxian_check(bin_img, (int)s_cross.flag_cross);
 * @note   仅在 cross_flag 至少出现过一次 Cross_Begin 之后才真正扫描；
 *         扫描范围：行 ∈ [30, 125)（ROI 局部行号），列 ∈ [20, 300)；
 *         判定阈值：单行跳变 ≥ 6；
 *         去抖：连续 2 帧命中才置 1，连续 8 帧未命中才清 0；
 *         打印：仅在 0→1 上升沿向 stdout printf 一次。
 */
void banmaxian_check(const cv::Mat &bin_img, int cross_flag);

/**
 * @brief 填充斑马线模块调试快照
 * @param out [out] 输出快照结构体
 * @return 无
 * @sample BanmaxianDebugSnapshot s{}; banmaxian_debug_fill(s);
 */
void banmaxian_debug_fill(BanmaxianDebugSnapshot &out);

#endif /* BANMAXIAN_HPP */
