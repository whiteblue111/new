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
  
#endif  