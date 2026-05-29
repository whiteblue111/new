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

/** 1=红块预检通过后裁切 + NCNN 6 类；无红块时跳过裁切/推理；0=关闭（比赛巡线优先） */
#ifndef ENABLE_VISION_NCNN
#define ENABLE_VISION_NCNN 1
#endif
/** 1=识别结果映射为中线绕行；0=仅识别/显示，不改巡线中线 */
#ifndef ENABLE_VISION_BYPASS
#define ENABLE_VISION_BYPASS 0
#endif

/** 1=红砖避障；0=关闭（比赛巡线优先） */
#ifndef ENABLE_VISION_BRICK_CFG
#define ENABLE_VISION_BRICK_CFG 1
#endif

/** 屏显/HUD：0=关 1=HUD 2=HUD+串口（CMake -DTRACK_DEBUG_LEVEL 可覆盖） */
#ifndef APP_TRACK_DEBUG_LEVEL
#define APP_TRACK_DEBUG_LEVEL 0
#endif

/** 1=Normal 直道裁剪 blur/角点/NMS 等后处理；0=每帧完整链 */
#ifndef TRACK_FAST_PATH
#define TRACK_FAST_PATH 0
#endif

/** 1=终端打印图像 FPS 与三环 PIT 周期统计；0=关闭 */
#ifndef ENABLE_PERF_STATS
#define ENABLE_PERF_STATS 1
#endif
/** 1=image_process 子阶段耗时汇总（依赖 ENABLE_PERF_STATS） */
#ifndef ENABLE_PERF_IMAGE_STAGES
#define ENABLE_PERF_IMAGE_STAGES 1
#endif
/** perf_stats_poll_print 打印间隔（毫秒） */
#ifndef PERF_STATS_PRINT_INTERVAL_MS
#define PERF_STATS_PRINT_INTERVAL_MS 1000
#endif

/*
 * 场测回归（直道/十字/环岛/弱光）：
 * 1. 终端 [perf] img: fps 与 stages(ms) 对比优化前后
 * 2. 直道：img_err 抖动 < ±3px，TRACK_FAST_PATH=1 时十字仍能 2 帧确认
 * 3. 十字 Cross_Out：远线固定阈值 128，出弯不丢线
 * 4. 环岛进环：单 L + ring_findline 候选帧仍能触发
 * 5. 弱光：若 OTSU 不稳，暂设 TRACK_FAST_PATH=0 或后续改固定阈值
 */
