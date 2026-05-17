#include "zf_common_headfile.hpp" 
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>

// ==========================================
// 1. 无模型的基础环境测试 (验证 .so 和头文件)
// ==========================================
void my_ncnn_env_test(void)
{
    try {
        std::cout << "========== NCNN 环境自检 ==========" << std::endl;
        ncnn::Net net;
        net.opt.num_threads = 2; // 使用龙芯双核

        ncnn::Mat input(8, 8, 3);
        input.fill(1.0f); // 填充测试数据

        ncnn::Extractor ex = net.create_extractor();
        int ret = ex.input("data", input);

        if (ret != 0) {
            std::cout << "[PASS] ncnn 基础环境可用 (运行时正常加载)。" << std::endl;
        } else {
            std::cout << "[WARN] 环境可能存在问题。" << std::endl;
        }
    } catch (...) {
        std::cerr << "[FAIL] ncnn 环境自检异常，请检查库文件！" << std::endl;
    }
}

// ==========================================
// 2. 真实模型加载与图片推理测试
// ==========================================
void my_ncnn_photo_demo(void)
{
    std::cout << "========== NCNN 真实模型推理测试 ==========" << std::endl;

    // --- 1. 配置你的模型路径和图片路径 ---
    // ⚠️重要：运行时请确保这三个文件和你的可执行程序在同一个目录下！
    std::string model_param = "tiny_classifier_fp32.ncnn.param";
    std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";
    std::string test_image  = "test.jpg"; // 随便找一张测试图放进去

    // --- 2. 加载模型 ---
    ncnn::Net net;
    net.opt.num_threads = 2; // 开启多线程加速
    
    if (net.load_param(model_param.c_str()) != 0) {
        std::cerr << "[错误] 找不到或无法解析 " << model_param << "!" << std::endl;
        return;
    }
    if (net.load_model(model_bin.c_str()) != 0) {
        std::cerr << "[错误] 找不到或无法解析 " << model_bin << "!" << std::endl;
        return;
    }
    std::cout << "[成功] 模型 " << model_param << " 加载完毕!" << std::endl;

    // --- 3. 读取图片 ---
    cv::Mat image = cv::imread(test_image);
    if (image.empty()) {
        std::cerr << "[错误] 无法读取测试图片 " << test_image << "!" << std::endl;
        return;
    }
    std::cout << "[成功] 读取图片尺寸: " << image.cols << "x" << image.rows << std::endl;

    // --- 4. 图像预处理 (缩放、RGB转换、归一化) ---
    // 你的模型输入需要 96x96
    cv::Mat resized_img;
    cv::resize(image, resized_img, cv::Size(64, 64));

    // OpenCV 默认是 BGR，转换为 NCNN 用的 RGB (包含归一化参数)
    ncnn::Mat in = ncnn::Mat::from_pixels(resized_img.data, ncnn::Mat::PIXEL_BGR2RGB, resized_img.cols, resized_img.rows);
    
    // 减去均值并乘上比例因子 (使用你提供的 ImageNet 参数)
    const float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    const float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    // --- 5. 开始推理 ---
    std::cout << "[推理] 开始执行..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    ncnn::Extractor ex = net.create_extractor();
    ex.input("in0", in); // ⚠️注意：这里的 "in0" 是模型输入层名字，如果报错，请用 Netron 软件打开 param 文件确认真实名字！

    ncnn::Mat out;
    ex.extract("out0", out); // ⚠️注意：这里的 "out0" 是输出层名字，同样需要确认！

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // --- 6. 解析结果 ---
    std::cout << "[耗时] " << duration.count() << " ms" << std::endl;
    
    // 找出概率最大的一项
    float max_prob = -1.0f;
    int max_index = -1;
    for (int i = 0; i < out.w; i++) {
        if (out[i] > max_prob) {
            max_prob = out[i];
            max_index = i;
        }
        std::cout << "类别 " << i << " 概率: " << out[i] << std::endl;
    }

    // 对应你代码里的类别
    std::vector<std::string> labels = {"Ambulance", "Armored vehicle", "Binoculars","Grenade","Guns","medical"};
    if (max_index >= 0 && max_index < labels.size()) {
        std::cout << "\n>>> 最终预测结果: " << labels[max_index] << " (概率 " << max_prob << ") <<<" << std::endl;
    }
}