#ifndef ELEMENTS
#define ELEMENTS

/**
 * @file elements.hpp
 * @brief 元素状态保留头（重构后大幅精简）
 *
 * 历史的十字补线 / 角点搜索 / 边界延长函数均已下沉到 image.cpp + cross.cpp +
 * ring.cpp 内部，本头文件仅保留几个供 motor / redbrick / 其他模块作为传感器
 * 桩使用的全局变量，作为后续接入真实数据前的占位。
 *
 * 真正的元素状态：
 *   - 十字状态 ⇒ image.cpp 内部 Cross 类（s_cross.flag_cross）
 *   - 环岛状态 ⇒ image.cpp 内部 Ring  类（s_ring.flag_ring）
 *   - 场景标识 ⇒ image.hpp 的 extern int scene
 */

#include "zf_common_headfile.hpp"
#include "image.hpp"
#include <cstdint>


#define RAD_TO_DEG  (57.295779513082320876798154814105f)


/* ====================== 桩变量（待接入真实传感器） ====================== */

/** @brief IMU yaw 角（°），暂为占位，需在 imu0.cpp 中赋值后改成真实读出 */
extern float degree_y;
/** @brief 实际车速（mm/s），暂为占位，需在 motor.cpp 中赋值后改成真实读出 */
extern float real_speed_mm;
/** @brief 侧边桥标志，0 默认；接入实际元素检测后赋值 */
extern int   cebian_flag;
/** @brief 单边桥标志，0 默认；接入实际元素检测后赋值 */
extern int   danbianqiao_flag;
/** @brief 坡道/跳变标志，0 默认；接入实际元素检测后赋值 */
extern int   jump_flag;
/** @brief 元素通用计数器占位 */
extern int   yuansu_flag;
/** @brief 元素通用计数器占位 */
extern int   yuansu_count;

#endif /* ELEMENTS */
