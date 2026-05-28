#pragma once

/** 1=主循环 [dbg] stage= 探针 + image 边线不一致 abort；0=关闭（默认） */
#ifndef APP_TERMINAL_DEBUG
#define APP_TERMINAL_DEBUG 0
#endif

/* 覆盖 CMake -DENABLE_TERMINAL_DEBUG=1，以源码为准 */
#ifdef ENABLE_TERMINAL_DEBUG
#undef ENABLE_TERMINAL_DEBUG
#endif
#define ENABLE_TERMINAL_DEBUG APP_TERMINAL_DEBUG

/** 1=角速度环独立调参：不启 18ms 角度环，VOFA RX1 映射 g_target_yaw_spd；0=三串环 */
#ifndef YAW_SPD_TUNE_MODE
#define YAW_SPD_TUNE_MODE 0
#endif

/** 1=主循环 process_car_vision（红块裁切 + NCNN 6 类）；0=关闭 */
#ifndef ENABLE_VISION_NCNN
#define ENABLE_VISION_NCNN 0
#endif
/** 1=识别结果映射为中线绕行；0=仅识别/显示，不改巡线中线 */
#ifndef ENABLE_VISION_BYPASS
#define ENABLE_VISION_BYPASS 0
#endif
