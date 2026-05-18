#ifndef DISPLAY_HPP_  
#define DISPLAY_HPP_  
  
#include "zf_common_headfile.hpp"  
//逐飞助手
#define SERVER_IP "192.168.137.1"    
#define PORT      8086   
  
// 初始化显示模块（保存屏幕对象指针）  
void display_init(zf_device_ips200* ips);  
void draw_cross(int x, int y, uint16 color);
void display_show_overlay(int left_org_num, int right_org_num,  int sx_l, int sy_l, int sx_r, int sy_r,  float err_img);  
void draw_points(float pts[][2], int num, int y_off, uint16 color);

/**
 * @brief 在 IPS200 上显示未透视二值 ROI 与最终中线
 * @return 无
 * @sample display_init(&ips200); display_show_track();
 * @note   读取全局 bin_img、CenterEdge；X 缩放到屏宽 240，Y 与 ROI 行号对齐
 */
void display_show_track(void);
  
#endif  