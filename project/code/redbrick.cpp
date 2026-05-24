#include "redbrick.hpp"
#include "general.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>


/**
 * @brief 红砖避障状态机构造函数，初始化阈值与缓存状态
 * @param 无
 * @return 无
 * @note 触发线从全图 y=160 转到 ROI 坐标后做边界夹紧，避免 320x130 下越界。
 */
RedBlockAvoider::RedBlockAvoider() {
    current_state = RB_STATE_NORMAL;
    force_track = FORCE_NONE;
    current_avoid_offset = 0.0f;
    no_red_bottom_frames = 0;
    bottom_red_present = false;
    last_bottom_y = -1;
    last_center_x = -1;
    last_area = 0;
    last_aspect = 0.0f;
    last_filter_ok = 0;

    const int trigger_y_roi = 160 - ROI_TOP;
    trigger_y_threshold = std::clamp(trigger_y_roi, 0, ROI_H - 2);
    
    // 参数可以适当放宽，因为按行扫描已经很强了
    min_contour_area = 2000;       
    min_aspect_ratio = 0.9f;     /* 高/宽 下限，等价于原宽/高 max=4.0 */
    max_aspect_ratio = 1.5f;      /* 高/宽 上限，等价于原宽/高 min=1.0 */
}


/**
 * @brief 红砖检测：按行红色占比锁定底部，再做几何与阈值过滤
 * @param frame     [in]  输入 RGB 裁剪图
 * @param best_rect [out] 过滤通过后的红砖包围框
 * @return 通过面积与高宽比过滤返回 true，否则 false
 * @note  bottom_red_present 仅表示“底部是否有红色行”，用于 ACTIVE 退出计数，不等价于最终检测通过。
 */
bool RedBlockAvoider::detect_red_brick(const cv::Mat& frame, cv::Rect& best_rect) {
    // 【修改点1】彻底删掉 HSV 转换代码
    // cv::Mat hsv_img;
    // cv::cvtColor(frame, hsv_img, cv::COLOR_BGR2HSV);

    // 1. 忽略左右边缘 10% 的区域
    int scan_x_start = (int)(frame.cols * 0.1f);
    int scan_x_end = (int)(frame.cols * 0.9f);
    int scan_width = scan_x_end - scan_x_start;

    // 2. 统计每行的红色像素比例
    std::vector<float> row_ratios(frame.rows, 0.0f);
    for (int y = 0; y < frame.rows; y++) {
        int red_cnt = 0;
        // 【修改点2】直接获取原图 frame 这一行的 const 指针
        const cv::Vec3b* ptr = frame.ptr<cv::Vec3b>(y);
        for (int x = scan_x_start; x < scan_x_end; x++) {
            // 【修改点3】使用 BGR 判定
            if (is_red_bgr(ptr[x])) red_cnt++;
        }
        row_ratios[y] = (float)red_cnt / scan_width;
    }

    // 3. 从下往上找红砖底部 (连续 RED_CONFIRM_ROWS 行达标)
    int confirmed_bottom = -1;
    int red_row_count = 0;
    for (int y = frame.rows - 1; y >= 0; y--) {
        if (row_ratios[y] >= RB_RED_ROW_RATIO) {
            red_row_count++;
            if (red_row_count >= RB_RED_CONFIRM_ROWS) {
                confirmed_bottom = y + RB_RED_CONFIRM_ROWS - 1;
                break;
            }
        } else {
            red_row_count = 0; 
        }
    }
    bottom_red_present = (confirmed_bottom >= 0);

    if (confirmed_bottom < 0) {
        last_bottom_y = -1;
        last_center_x = -1;
        last_area = 0;
        last_aspect = 0.0f;
        last_filter_ok = 0;
        red_area = 0;
        aspect_ratio = 0.0f;
        return false;
    }

    // 4. 从底部继续往上找顶部 (允许中间断层 2 行)
    int blk_top = confirmed_bottom;
    int gap = 0;
    for (int y = confirmed_bottom; y >= 0; y--) {
        if (row_ratios[y] >= RB_RED_ROW_RATIO) {
            blk_top = y;
            gap = 0;
        } else { 
            if (++gap > 2) break; 
        }
    }

    // 5. 在红砖中间的一行，往左右找边界
    int mid_y = (blk_top + confirmed_bottom) / 2;
    int lx = -1, rx = -1;
    
    // 【修改点4】获取原图 frame 中间行的 const 指针
    const cv::Vec3b* mid_ptr = frame.ptr<cv::Vec3b>(mid_y);
    for (int x = scan_x_start; x < scan_x_end; x++) {
        if (is_red_bgr(mid_ptr[x])) { lx = x; break; }
    }
    for (int x = scan_x_end - 1; x >= scan_x_start; x--) {
        if (is_red_bgr(mid_ptr[x])) { rx = x; break; }
    }

    if (lx < 0 || rx < 0 || rx <= lx) {
        last_bottom_y = -1;
        last_center_x = -1;
        last_area = 0;
        last_aspect = 0.0f;
        last_filter_ok = 0;
        red_area = 0;
        aspect_ratio = 0.0f;
        return false;
    }

    // 6. 计算尺寸、面积和高宽比（高/宽）
    int block_w = rx - lx;
    int block_h = confirmed_bottom - blk_top;
    
    if (block_h == 0) {
        last_bottom_y = -1;
        last_center_x = -1;
        last_area = 0;
        last_aspect = 0.0f;
        last_filter_ok = 0;
        red_area = 0;
        aspect_ratio = 0.0f;
        return false;
    }

    last_bottom_y = confirmed_bottom;
    last_center_x = (lx + rx) / 2;

    int area = block_w * block_h;
    float current_aspect = (float)block_h / (float)block_w;
    last_area = area;
    last_aspect = current_aspect;

    // 7. 使用长宽比和面积进行最终过滤
    if (area > min_contour_area && current_aspect >= min_aspect_ratio && current_aspect <= max_aspect_ratio) {
        
        // 生成最终的紧凑矩形框
        best_rect = cv::Rect(lx, blk_top, block_w, block_h);
        
        red_area = area;
        aspect_ratio = current_aspect;
        last_filter_ok = 1;
        
        return true;
    }

    last_filter_ok = 0;
    return false;
}

/**
 * @brief 红砖两态状态机：识别即 ACTIVE，底部无红连续 30 帧退出
 * @param frame         [in,out] 输入 RGB 裁剪图（可选调试绘制）
 * @param is_draw_debug [in] 是否在图上绘制检测框
 * @return 无
 * @sample g_brick_avoider.process(rgb_cut_img, false);
 * @note ACTIVE 时保持 force_track 方向；若 bottom_red_present 连续 false 达到 RB_NO_RED_EXIT_FRAMES 则复位为 NORMAL。
 */
void RedBlockAvoider::process(cv::Mat& frame, bool is_draw_debug) {
    cv::Rect brick_rect;
    bool is_detected = detect_red_brick(frame, brick_rect);

    if (is_detected) {
        current_state = RB_STATE_ACTIVE;
        no_red_bottom_frames = 0;

        const int brick_center_x = brick_rect.x + brick_rect.width / 2;
        if (brick_center_x > frame.cols / 2)
            force_track = FORCE_LEFT_LINE;
        else
            force_track = FORCE_RIGHT_LINE;
    } else if (current_state == RB_STATE_ACTIVE) {
        if (bottom_red_present) {
            no_red_bottom_frames = 0;
        } else {
            no_red_bottom_frames++;
            if (no_red_bottom_frames >= RB_NO_RED_EXIT_FRAMES) {
                current_state = RB_STATE_NORMAL;
                force_track = FORCE_NONE;
                no_red_bottom_frames = 0;
            }
        }
    } else {
        force_track = FORCE_NONE;
        no_red_bottom_frames = 0;
    }
    current_avoid_offset = 0.0f;

    // 绘制调试信息 (如果不需要可以传入 false 关闭)
    if (is_draw_debug) {
        // ... 画线和画框逻辑不变 ...
        // cv::line(frame, cv::Point(0, trigger_y_threshold), cv::Point(frame.cols, trigger_y_threshold), cv::Scalar(255, 0, 0), 1);
        if (is_detected) {
            cv::rectangle(frame, brick_rect, cv::Scalar(0, 0, 255), 2);
            cv::circle(frame, cv::Point(brick_rect.x + brick_rect.width / 2, brick_rect.y + brick_rect.height / 2), 4, cv::Scalar(0, 255, 255), -1);
        }
    }
}

/**
 * @brief 从 RedBlockAvoider 填充红砖调试快照
 * @param av  [in]  红砖避障实例
 * @param out [out] 输出结构体
 * @return  无
 */
void redbrick_debug_fill(const RedBlockAvoider &av, RedbrickDebugSnapshot &out)
{
    out.state    = (int)av.get_state();
    out.area     = av.last_area;
    out.aspect   = av.last_aspect;
    out.offset   = av.get_avoid_offset();
    out.force    = (int)av.get_force_track_type();
    out.bot_y    = av.last_bottom_y;
    out.cx       = av.last_center_x;
    out.detected = av.last_filter_ok;
    out.trg_y    = av.trigger_y_threshold;
    out.min_area = av.min_contour_area;
    out.asp_min  = av.min_aspect_ratio;
    out.asp_max  = av.max_aspect_ratio;
}

