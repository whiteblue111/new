#include "display.hpp"  
#include "zf_common_headfile.hpp"  
#include <cmath>  
#include <cstdint>  
  
 
  
static zf_device_ips200* s_ips = nullptr;  
  
static inline int iroundf_local(float v)  
{  
    return (int)(v + (v >= 0.0f ? 0.5f : -0.5f));  
}  
  
void display_init(zf_device_ips200* ips)  
{  
    s_ips = ips;  
}  
  
/* -------------------- 小工具：画一组点 -------------------- */  
void draw_points(float pts[][2], int num, int y_off, uint16 color)  
{  
    if (!s_ips || !pts || num <= 0) return;  
    for (int i = 0; i < num; i++) {  
        int x = iroundf_local(pts[i][0]);  
        int y = iroundf_local(pts[i][1]) + y_off;  
        if (x <= 1 || x >= UVC_WIDTH - 2 || y <= 1 || y >= 319) continue;  
        s_ips->draw_point(x, y, color);  
    }  
}  
  
/* -------------------- 小工具：画起始十字 -------------------- */  
void draw_cross(int x, int y, uint16 color)  
{  
    if (!s_ips) return;  
    if (x > 2 && x < UVC_WIDTH - 3 && y > 2 && y < UVC_HEIGHT - 3) {  
        s_ips->draw_line(x - 2, y, x + 2, y, color);  
        s_ips->draw_line(x, y - 2, x, y + 2, color);  
    }  
}  
  

  

  
/* -------------------- 3) 画文本和起点 -------------------- */  
void display_show_overlay(int left_org_num, int right_org_num,  int sx_l, int sy_l, int sx_r, int sy_r,  float err_img)  
{  
    if (!s_ips) return;  
  
    s_ips->show_string(0, 250, "O-L:");  
    s_ips->show_int(35, 250, left_org_num, 3);  
    s_ips->show_string(75, 250, "O-R:");  
    s_ips->show_int(110, 250, right_org_num, 3);  
  
    s_ips->show_string(170, 250, "ERR:");  
    s_ips->show_float(205, 250, err_img, 4, 1);  
  
    draw_cross(sx_l, sy_l, RGB565_GREEN);  
    draw_cross(sx_r, sy_r, RGB565_66CCFF);  
}  
  



