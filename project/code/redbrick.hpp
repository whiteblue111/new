#ifndef REDBRICK_HPP
#define REDBRICK_HPP
#include "zf_common_headfile.hpp"
#include <opencv2/opencv.hpp>

extern float aspect_ratio;
extern int red_area;

// 红砖状态机枚举（两态）
enum RedBlockState {
    RB_STATE_NORMAL = 0,
    RB_STATE_ACTIVE
};

enum TrackForceType {
    FORCE_NONE = 0,
    FORCE_LEFT_LINE,
    FORCE_RIGHT_LINE
};

// 🔴 新增：按行扫描法的配置参数
const float RB_RED_ROW_RATIO = 0.05f;     // 一行中红色像素占比超过 5% 认为该行是红砖的一部分
const int   RB_RED_CONFIRM_ROWS = 2;      // 必须连续 2 行达标才确认找到底部
const int   RB_NO_RED_EXIT_FRAMES = 30;   // 底部连续无红达到该帧数后退出 ACTIVE

/**
 * @brief 红砖避障调试快照（只读，供 HUD / 串口）
 */
struct RedbrickDebugSnapshot
{
    int   state;     /**< 状态机 0=Normal 1=Active */
    int   area;      /**< 检测面积（过滤前原始值） */
    float aspect;    /**< 高宽比 h/w（过滤前原始值） */
    float offset;    /**< 避障偏移 */
    int   force;     /**< 强制巡线侧 0/1/2 */
    int   bot_y;     /**< 砖块底边 y，未找到为 -1 */
    int   cx;        /**< 砖块中心 x，未找到为 -1 */
    int   detected;  /**< 本帧过滤通过 0/1 */
    int   trg_y;     /**< 触发避障 y 阈值 */
    int   min_area;  /**< 最小面积阈值 */
    float asp_min;   /**< 高/宽 下限 */
    float asp_max;   /**< 高/宽 上限 */
};

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
    float current_avoid_offset; 
    int no_red_bottom_frames;
    bool bottom_red_present;

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

    int last_bottom_y;
    int last_center_x;
    int last_area;
    float last_aspect;
    int last_filter_ok;

    friend void redbrick_debug_fill(const RedBlockAvoider &av, RedbrickDebugSnapshot &out);
};

/**
 * @brief 从 RedBlockAvoider 填充红砖调试快照
 * @param av  [in]  红砖避障实例
 * @param out [out] 输出结构体
 * @return  无
 */
void redbrick_debug_fill(const RedBlockAvoider &av, RedbrickDebugSnapshot &out);

#endif

