/**
 * @file elements.cpp
 * @brief 元素状态保留实现（重构后大幅精简）
 *
 * 仅保留 motor / redbrick / 其他模块需要的桩变量。原 Cross_Find / shizi /
 * Lengthen_*_Boundry / Find_Angle_Point / check_cross / run_cross 等函数
 * 全部移除——新版十字与环岛逻辑请见 cross.hpp / ring.hpp 与 image.cpp。
 */

#include "elements.hpp"


/* ====================== 桩变量 ====================== */
float degree_y         = 0.0f;
float real_speed_mm    = 0.0f;
int   cebian_flag      = 0;
int   danbianqiao_flag = 0;
int   jump_flag        = 0;
int   yuansu_flag      = 0;
int   yuansu_count     = 0;
