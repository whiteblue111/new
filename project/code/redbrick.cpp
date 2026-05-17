#include "redbrick.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>


RedBlockAvoider::RedBlockAvoider() {
    current_state = RB_STATE_NORMAL;
    force_track = FORCE_NONE;
    avoid_counter = 0;
    return_counter = 0;
    current_avoid_offset = 0.0f;
    
    max_avoid_offset = 25.0f;     
    trigger_y_threshold = 160;    
    
    // 参数可以适当放宽，因为按行扫描已经很强了
    min_contour_area = 200;       
    min_aspect_ratio = 1.0f;      
    max_aspect_ratio = 4.0f;      
}


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

    if (confirmed_bottom < 0) return false; // 没找到底部

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

    if (lx < 0 || rx < 0 || rx <= lx) return false;

    // 6. 计算尺寸、面积和长宽比
    int block_w = rx - lx;
    int block_h = confirmed_bottom - blk_top;
    
    if (block_h == 0) return false;

    int area = block_w * block_h;
    float current_aspect = (float)block_w / (float)block_h;

    // 7. 使用长宽比和面积进行最终过滤，并传给全局变量供屏幕显示
    if (area > min_contour_area && current_aspect >= min_aspect_ratio && current_aspect <= max_aspect_ratio) {
        
        // 生成最终的紧凑矩形框
        best_rect = cv::Rect(lx, blk_top, block_w, block_h);
        
        // 更新供 main.cpp 屏幕显示的全局变量
        red_area = area;
        aspect_ratio = current_aspect;
        
        return true;
    }

    return false;
}

// process 函数的状态机逻辑完全不用变
void RedBlockAvoider::process(cv::Mat& frame, bool is_draw_debug) {
    cv::Rect brick_rect;
    bool is_detected = detect_red_brick(frame, brick_rect);

    switch (current_state) {
        case RB_STATE_NORMAL:
            if (is_detected) {
                current_state = RB_STATE_APPROACHING;
            }
            break;

        case RB_STATE_APPROACHING:
            if (is_detected) {
                int brick_bottom_y = brick_rect.y + brick_rect.height;
                int brick_center_x = brick_rect.x + brick_rect.width / 2;

                if (brick_bottom_y > trigger_y_threshold) {
                    current_state = RB_STATE_AVOIDING;
                    avoid_counter = 0;
                    if (brick_center_x > frame.cols / 2) force_track = FORCE_LEFT_LINE;
                    else force_track = FORCE_RIGHT_LINE;
                }
            } else {
                current_state = RB_STATE_NORMAL;
            }
            break;

        case RB_STATE_AVOIDING:
            avoid_counter++;
            if (current_avoid_offset < max_avoid_offset) current_avoid_offset += 2.0f; 
            if (avoid_counter > 30) { 
                current_state = RB_STATE_RETURNING;
                return_counter = 0;
            }
            break;

        case RB_STATE_RETURNING:
            return_counter++;
            if (current_avoid_offset > 0.0f) current_avoid_offset -= 1.5f;
            else current_avoid_offset = 0.0f;
            
            if (return_counter > 20 && current_avoid_offset <= 0.0f) {
                current_state = RB_STATE_NORMAL;
                force_track = FORCE_NONE;
                current_avoid_offset = 0.0f;
            }
            break;
    }

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

