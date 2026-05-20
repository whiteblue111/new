#include "zf_common_headfile.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>
#include <cstdio>
#include <sys/time.h>
#include <csignal>

using namespace std;
using namespace cv;

int red_area = 0; // 全局变量，记录当前检测到的红色区域面积
float aspect_ratio = 0.0f; // 全局变量，记录当前检测到

volatile sig_atomic_t g_dbg_stage_id = 0;
bool  g_is_bypassing_binoculars = false;
int   g_bypass_timer            = 0;
const int   BYPASS_MAX_FRAMES   = 80;
const float BYPASS_OFFSET       = 30.0f;

// 实例化一个本文件内部使用的检测器
static RedRectDetector my_detector;

int block_w = 0; // 全局变量，记录当前检测到的红色块宽度
int block_h = 0; // 全局变量，记录当前检测到的红色

// ==========================================
// RedRectDetector 类的方法实现
// ==========================================
bool RedRectDetector::find_red_block(Mat& img, int& block_top, int& block_bottom, int& x_left, int& x_right) {
    // 【修改点1】彻底删除 HSV 转换代码
    // 之前这里的 cvtColor(img, hsv_img, COLOR_BGR2HSV); 已经被干掉了！

    int scan_x_start = (int)(img.cols * 0.1f);
    int scan_x_end = (int)(img.cols * 0.9f);
    int scan_width = scan_x_end - scan_x_start;

    vector<float> row_ratios(img.rows, 0.0f);
    for (int y = 0; y < img.rows; y++) {
        int red_cnt = 0;
        // 【修改点2】直接获取原图(BGR)这一行的指针
        Vec3b* ptr = img.ptr<Vec3b>(y);
        for (int x = scan_x_start; x < scan_x_end; x++) {
            // 【修改点3】调用新的极速 BGR 判断逻辑
            if (is_red_bgr(ptr[x])) red_cnt++;
        }
        row_ratios[y] = (float)red_cnt / scan_width;
    }

    int confirmed_bottom = -1;
    int red_row_count = 0;
    for (int y = img.rows - 1; y >= 0; y--) {
        if (row_ratios[y] >= RED_ROW_RATIO) {
            red_row_count++;
            if (red_row_count >= RED_CONFIRM_ROWS) {
                confirmed_bottom = y + RED_CONFIRM_ROWS - 1;
                break;
            }
        }
        else { red_row_count = 0; }
    }

    if (confirmed_bottom < 0) return false;

    int blk_top = confirmed_bottom;
    int gap = 0;
    for (int y = confirmed_bottom; y >= 0; y--) {
        if (row_ratios[y] >= RED_ROW_RATIO) {
            blk_top = y;
            gap = 0;
        }
        else { if (++gap > 2) break; }
    }

    int mid_y = (blk_top + confirmed_bottom) / 2;
    int lx = -1, rx = -1;
    
    // 【修改点4】找左右边界时，也直接用原图指针
    Vec3b* mid_ptr = img.ptr<Vec3b>(mid_y);
    for (int x = scan_x_start; x < scan_x_end; x++) {
        if (is_red_bgr(mid_ptr[x])) { lx = x; break; }
    }
    for (int x = scan_x_end - 1; x >= scan_x_start; x--) {
        if (is_red_bgr(mid_ptr[x])) { rx = x; break; }
    }

    if (lx < 0 || rx < 0 || rx <= lx) return false;

    block_top = blk_top; block_bottom = confirmed_bottom;
    x_left = lx; x_right = rx;
    return true;
}

bool RedRectDetector::model_roi_cut(Mat& img, Mat& roi, bool is_draw) {
    int blk_top, blk_bottom, x_left, x_right;
    if (!find_red_block(img, blk_top, blk_bottom, x_left, x_right)) return false;

    block_w = x_right - x_left;
    block_h = blk_bottom - blk_top;
    // 强制紧贴红框裁剪，不再受限于 MODEL_INPUT_WIDTH
    int side = max({ (block_w + 10), (block_h + 10) });

    int cx = (x_left + x_right) / 2;
    int x1 = cx - side / 2;
    int y2 = blk_bottom + CROP_EDGE_PADDING;
    int y1 = y2 - side;

    target_rect = Rect(x1, y1, side, side) & Rect(0, 0, img.cols, img.rows);
    if (target_rect.width <= 0 || target_rect.height <= 0) return false;

    Mat cropped = img(target_rect);
    resize(cropped, roi, Size(MODEL_INPUT_WIDTH, MODEL_INPUT_WIDTH), 0, 0, INTER_AREA);
    //如果红块太小（比如宽度小于 20 像素），说明离得太远或者是个噪点，直接放弃识别
    if (block_w < 20 || block_h < 5 ||block_w > 50 || block_h > 20) {
        return false; 
    }

    if (is_draw) {
        rectangle(img, target_rect, Scalar(0, 255, 0), 2); 
        rectangle(img, Rect(x_left, blk_top, block_w, block_h), Scalar(0, 0, 255), 2); 
    }
    return true;
}
/**
 * @brief 视觉模块初始化：加载 NCNN 分类模型并配置推理选项
 * @param 无
 * @return 无（加载失败 NCNN 内部 printf 报错，对象保持空状态）
 * @sample vision_init();
 * @note LS2K0300 无 FP16 加速，强制 FP32；2 线程对应 LA264 双核
 */
 void vision_init(void)
 {
     printf("[vision] 正在加载 NCNN 模型...\n");
     my_net.opt.num_threads        = 2;
     my_net.opt.use_fp16_arithmetic = false;
     my_net.opt.use_fp16_storage    = false;
     my_net.load_param("tiny_classifier_fp32.ncnn.param");
     my_net.load_model("tiny_classifier_fp32.ncnn.bin");
     printf("[vision] NCNN 模型加载完毕！\n");
 }

// =============================================================================
// 流水线主函数实现
// =============================================================================
void process_car_vision(cv::Mat& frame) {
    g_dbg_stage_id = 31;
    // #region agent log
    static int s_dbg_vision_entry_cnt = 0;
    if (s_dbg_vision_entry_cnt < 8) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
        FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
        if (debug_log) {
            fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H5\",\"location\":\"vision.cpp:process_car_vision:entry\",\"message\":\"vision entry frame info\",\"data\":{\"rows\":%d,\"cols\":%d,\"channels\":%d,\"empty\":%d},\"timestamp\":%lld}\n", frame.rows, frame.cols, frame.channels(), frame.empty() ? 1 : 0, ts_ms);
            fclose(debug_log);
        }
        s_dbg_vision_entry_cnt++;
    }
    // #endregion

    cv::Mat roi;
    
    const bool has_roi = my_detector.model_roi_cut(frame, roi, true);
    g_dbg_stage_id = 32;
    // #region agent log
    static int s_dbg_vision_roi_cnt = 0;
    if (s_dbg_vision_roi_cnt < 8) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
        FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
        if (debug_log) {
            fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H1\",\"location\":\"vision.cpp:process_car_vision:roi\",\"message\":\"roi cut result\",\"data\":{\"has_roi\":%d,\"roi_rows\":%d,\"roi_cols\":%d,\"block_w\":%d,\"block_h\":%d},\"timestamp\":%lld}\n", has_roi ? 1 : 0, roi.rows, roi.cols, block_w, block_h, ts_ms);
            fclose(debug_log);
        }
        s_dbg_vision_roi_cnt++;
    }
    // #endregion

    if (has_roi) {
        g_dbg_stage_id = 33;
        // ✨ [新增] 调试抓拍逻辑开始
        // static int snapshot_cnt = 0; // 静态计数器
        // if (snapshot_cnt < 5) {
        //     // 生成文件名，比如 debug_crop_0.jpg
        //     std::string filename = "debug_crop_" + std::to_string(snapshot_cnt) + ".jpg";
            
        //     // 将 64x64 的 roi 保存到本地
        //     cv::imwrite(filename, roi);
            
        //     // 把画了红框、绿框的原图也保存下来，方便对比！
        //     std::string full_filename = "debug_full_" + std::to_string(snapshot_cnt) + ".jpg";
        //     cv::imwrite(full_filename, frame);
            
        //     printf("[调试] 抓拍成功！已保存: %s 和 %s\n", filename.c_str(), full_filename.c_str());
        //     snapshot_cnt++;
        // }
        // // ✨ 调试抓拍逻辑结束
        
        ncnn::Mat in = ncnn::Mat::from_pixels(roi.data, ncnn::Mat::PIXEL_BGR2RGB, roi.cols, roi.rows);
        
        const float mean_vals[3] = {123.675f, 116.28f, 103.53f};
        const float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
        in.substract_mean_normalize(mean_vals, norm_vals);

        ncnn::Extractor ex = my_net.create_extractor();
        const int in_ret = ex.input("in0", in); 
        ncnn::Mat out;
        const int out_ret = ex.extract("out0", out);
        g_dbg_stage_id = 34;
        // #region agent log
        static int s_dbg_vision_ncnn_cnt = 0;
        if (s_dbg_vision_ncnn_cnt < 8) {
            struct timeval tv;
            gettimeofday(&tv, nullptr);
            const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
            FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
            if (debug_log) {
                fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H1\",\"location\":\"vision.cpp:process_car_vision:ncnn\",\"message\":\"ncnn io status\",\"data\":{\"in_ret\":%d,\"out_ret\":%d,\"out_w\":%d,\"out_h\":%d,\"out_c\":%d},\"timestamp\":%lld}\n", in_ret, out_ret, out.w, out.h, out.c, ts_ms);
                fclose(debug_log);
            }
            s_dbg_vision_ncnn_cnt++;
        }
        // #endregion

        // 前面的 NCNN 提取结果代码保持不变...
        float max_prob = -100.0f;
        int max_index = -1;
        for (int i = 0; i < out.w; i++) {
            if (out[i] > max_prob) {
                max_prob = out[i];
                max_index = i;
            }
        }
        g_dbg_stage_id = 35;
        // #region agent log
        static int s_dbg_vision_pred_cnt = 0;
        if (s_dbg_vision_pred_cnt < 8) {
            struct timeval tv;
            gettimeofday(&tv, nullptr);
            const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
            FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
            if (debug_log) {
                fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H4\",\"location\":\"vision.cpp:process_car_vision:pred\",\"message\":\"prediction result before debounce\",\"data\":{\"max_index\":%d,\"max_prob\":%.6f,\"out_w\":%d},\"timestamp\":%lld}\n", max_index, max_prob, out.w, ts_ms);
                fclose(debug_log);
            }
            s_dbg_vision_pred_cnt++;
        }
        // #endregion

        // ✨ [修改点] 引入消抖滤波机制的三个静态变量
        static int confirmed_index = -2; // 真正被确认并正在执行的“官方结果”
        static int candidate_index = -2; // 正在考察期的“候选结果”
        static int consecutive_cnt = 0;  // “候选结果”连续出现的次数

        // 1. 考察候选结果
        if (max_index == candidate_index) {
            // 如果这一帧的结果和上一帧一样，计数器加 1
            consecutive_cnt++;
        } else {
            // 如果出现了一个新的结果，打断施法，重新开始计数
            candidate_index = max_index;
            consecutive_cnt = 1;
        }

        // 2. 判断候选结果是否熬过“考察期”（连续 3 帧）
        // 注意这里用 >= 3，因为如果车一直看着这个目标，计数器会一直加下去
        // 2. 判断候选结果是否熬过“考察期”（连续 3 帧）
        if (consecutive_cnt >= 3) {
            
            // 3. 如果这个新确认的结果，和我们之前一直在执行的“官方结果”不一样，才触发动作和打印
            if (candidate_index != confirmed_index) {
                g_dbg_stage_id = 36;
                
                std::vector<std::string> labels = {"Ambulance", "Armored vehicle", "Binoculars", "Grenade", "Guns", "medical"};
                // #region agent log
                static int s_dbg_vision_label_cnt = 0;
                if (s_dbg_vision_label_cnt < 8) {
                    struct timeval tv;
                    gettimeofday(&tv, nullptr);
                    const long long ts_ms = (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
                    FILE* debug_log = fopen("/home/lq/LS2K0300_Library/.cursor/debug-4df1ba.log", "a");
                    if (debug_log) {
                        fprintf(debug_log, "{\"sessionId\":\"4df1ba\",\"runId\":\"pre-fix\",\"hypothesisId\":\"H4\",\"location\":\"vision.cpp:process_car_vision:label\",\"message\":\"before labels index access\",\"data\":{\"candidate_index\":%d,\"labels_size\":%zu,\"confirmed_index\":%d,\"consecutive_cnt\":%d},\"timestamp\":%lld}\n", candidate_index, labels.size(), confirmed_index, consecutive_cnt, ts_ms);
                        fclose(debug_log);
                    }
                    s_dbg_vision_label_cnt++;
                }
                // #endregion
                
                // ... 前面的代码不变 ...
                if (candidate_index >= 0) {
                    g_dbg_stage_id = 37;
                    // #region agent log
                    if ((candidate_index >= (int)labels.size()) || (candidate_index < 0)) {
                        fprintf(stderr, "[termdbg] vision invalid label index=%d labels_size=%zu\n", candidate_index, labels.size());
                    }
                    // #endregion
                    std::cout << "[智能视觉] 连续 3 帧确认目标: " << labels[candidate_index] << std::endl;
                    if (candidate_index == 2 && !g_is_bypassing_binoculars) {
                        std::cout << "🔭 发现望远镜！准备向左侧绕行！" << std::endl;
                        g_is_bypassing_binoculars = true; // 开启绕行状态
                        g_bypass_timer = 0;               // 计时器清零
                        
                        // 可选：在这里调用蜂鸣器滴一声，方便调试
                        // beep_on(); 
                    }
                    
                    // ✨ 加入总数上限限制（比如最多只存 5 次）
                    static int valid_hit_cnt = 0; 
                    if (valid_hit_cnt < 5) {
                        g_dbg_stage_id = 38;
                        std::string roi_name = "target_" + std::to_string(valid_hit_cnt) + "_" + labels[candidate_index] + "_roi.jpg";
                        std::string full_name = "target_" + std::to_string(valid_hit_cnt) + "_full.jpg";
                        
                        cv::imwrite(roi_name, roi);
                        g_dbg_stage_id = 39;
                        cv::imwrite(full_name, frame);
                        g_dbg_stage_id = 40;
                        
                        std::cout << "📸 抓拍成功！已保存: " << roi_name << " (进度: " << valid_hit_cnt + 1 << "/5)" << std::endl;
                        valid_hit_cnt++;
                    } else {
                        std::cout << "⚠️ 抓拍名额已满(5张)，本次不再保存图片。" << std::endl;
                    }

                    // 💡 你的小车动作代码...
                }
                 else {
                    std::cout << "[智能视觉] 确认目标已丢失" << std::endl;
                }

                // 4. 将候选结果转正，更新为官方结果
                confirmed_index = candidate_index;
                g_dbg_stage_id = 41;
            }
        }
    }
    g_dbg_stage_id = 42;
}