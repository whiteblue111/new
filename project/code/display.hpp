#ifndef DISPLAY_HPP_  
#define DISPLAY_HPP_  
  
#include "zf_common_headfile.hpp"  

class RedBlockAvoider;
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
 * @note   读取全局 bin_img；X 缩放到屏宽 240；绿/蓝为 ROI 边线，黄为 t_pointsEdge、红为 t_CenterEdge 俯视红点（不连线）
 */
void display_show_track(void);

/**
 * @brief 阶段 C HUD：原图/俯视图边线点数与 L 角点标志
 * @return 无
 * @sample display_show_debug_hud_phase_c();
 * @note  需在 image_process() 之后调用；文字显示在 y=200、265
 */
void display_show_debug_hud_phase_c(void);

/**
 * @brief 阶段 D~G HUD：偏差角、归一化、巡线侧、场景与十字/环岛标志
 * @return 无
 * @sample display_show_debug_hud_phase_d();
 * @note  需在 image_process() 与 Image_Error_Get() 之后调用；
 *        y=250 为 Cross:/Ring: 可读状态；y=280/295 为数值 HUD（含 X:/R:）
 */
void display_show_debug_hud_phase_d(void);

/**
 * @brief 红砖避障 HUD：检测量、状态机与阈值（y=140/155/170）
 * @param av [in] 红砖避障实例
 * @return 无
 * @sample display_show_debug_hud_redbrick(g_brick_avoider);
 * @note  需在 g_brick_avoider.process() 之后调用；位于 ROI 与 phase C HUD 之间
 */
void display_show_debug_hud_redbrick(const RedBlockAvoider &av);

#endif  