#ifndef REDBRICK_HPP
#define REDBRICK_HPP
#include "zf_common_headfile.hpp"
#include <opencv2/opencv.hpp>

extern float aspect_ratio;
extern int red_area;

// 避障状态机枚举
enum RedBlockState {
    RB_STATE_NORMAL = 0,
    RB_STATE_APPROACHING,
    RB_STATE_AVOIDING,
    RB_STATE_RETURNING
};

enum TrackForceType {
    FORCE_NONE = 0,
    FORCE_LEFT_LINE,
    FORCE_RIGHT_LINE
};

// 🔴 新增：按行扫描法的配置参数
const float RB_RED_ROW_RATIO = 0.05f;     // 一行中红色像素占比超过 5% 认为该行是红砖的一部分
const int   RB_RED_CONFIRM_ROWS = 2;      // 必须连续 2 行达标才确认找到底部

class RedBlockAvoider {
public:
    RedBlockAvoider();
    void process(cv::Mat& frame, bool is_draw_debug = true);
    RedBlockState get_state() const { return current_state; }
    float get_avoid_offset() const { return current_avoid_offset; }
    TrackForceType get_force_track_type() const { return force_track; }

private:
    RedBlockState current_state;
    TrackForceType force_track;
    int avoid_counter;
    int return_counter;
    float current_avoid_offset; 
    float max_avoid_offset;     

    // 面积和长宽比过滤依然保留，作为最后一道防线
    int min_contour_area;       
    int trigger_y_threshold;    
    float min_aspect_ratio;     
    float max_aspect_ratio;     

    inline bool is_red_bgr(const cv::Vec3b& bgr) {
        int b = bgr[0];
        int g = bgr[1];
        int r = bgr[2];
        
        // 经验阈值：剔除暗礁噪点，且 R 通道必须显著大于 G 和 B
        return (r > 80) && ((r - g) > 40) && ((r - b) > 40);
    }

    bool detect_red_brick(const cv::Mat& frame, cv::Rect& best_rect);

};
#endif

