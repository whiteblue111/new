#ifndef BIRD_LUT_HPP
#define BIRD_LUT_HPP

/**
 * @file bird_lut.hpp
 * @brief ROI 原图 ↔ 俯视图透视查表（初始化一次，替代逐点浮点 transf）
 */

#include "general.hpp"

/**
 * @brief 构建 forward / reverse 透视 LUT（幂等，首次调用有效）
 * @return 无
 * @sample bird_lut_init();
 */
void bird_lut_init(void);

/**
 * @brief 原图 ROI 坐标 → 俯视图坐标（查表）
 * @param[out] ri  俯视图列
 * @param[out] rj  俯视图行
 * @param      i   原图列（ROI 局部）
 * @param      j   原图行（ROI 局部 0..ROI_H-1）
 * @return         true 有效映射
 * @sample         int a,b; if (bird_lut_transf(a,b,x,y)) { ... }
 */
bool bird_lut_transf(int &ri, int &rj, int i, int j);

/**
 * @brief 俯视图坐标 → 原图 ROI 坐标（查表）
 * @param[out] ri  原图列
 * @param[out] rj  原图行
 * @param      i   俯视图列
 * @param      j   俯视图行
 * @return         true 有效映射
 * @sample         int a,b; bird_lut_reverse(a,b,tx,ty);
 */
bool bird_lut_reverse(int &ri, int &rj, int i, int j);

#endif /* BIRD_LUT_HPP */
