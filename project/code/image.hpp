#ifndef IMAGE_HPP
#define IMAGE_HPP

/**
 * @file image.hpp
 * @brief 巡线主流水线（移植自 temp_repo/track/standard/standard.cpp）
 *
 * 完整流水线（image_process）：
 *   ImageProcess::processImage(rgb_img) → bin_img (OTSU + 闭运算)
 *   trackRecognition(gray, bin)         → floodFill + 最长白列 + 迷宫法 + 透视
 *                                          + blur + resample + 角度 + NMS + 找拐点
 *   Cross_Check / Cross_Run            → 十字状态机
 *   Ring_Check  / Ring_Run             → 环岛状态机
 *   fitting() → normalizeCenterEdge → CenterEdge
 *   Image_Error_Get() → 中线像素偏差 → img_err
 *
 * 关键参数：
 *   - 摄像头分辨率 320x240；bin_img / gray_cut 为 ROI 裁剪 320x130
 *   - 巡线 ROI：全图 y∈[ROI_TOP, ROI_BOTTOM) = [30, 160)
 *   - rowCutBottom/rowCutUp 为全图行号；trackRecognition 使用 rowCut*_roi 局部行号
 *   - floodFill 种子点局部行 seed_y_roi = ROI_H - 1 = 129
 */

#include "general.hpp"
#include "imgproc.hpp"
#include "cross.hpp"
#include "ring.hpp"
#include "lq_camera_ex.hpp"
#include <opencv2/opencv.hpp>
#include <vector>


/* ====================== ROI / 巡线常量 ====================== */
#define rowCutBottom     (ROI_BOTTOM - 5)   /* 155，全图行号 */
#define rowCutUp         (ROI_TOP + 5)      /* 45，全图行号 */
#define rowCutBottom_roi (ROI_H - 5)        /* 125，ROI 局部行号（bin 320x130） */
#define rowCutUp_roi     5                 /* 15，ROI 局部行号 */
#define seed_y_roi       (ROI_H - 1)       /* 129，floodFill 种子局部行 */
#define WIDTH            COLSIMAGE
#define LCONF_MIN        70
#define LCONF_MAX        130



/**
 * @brief 巡线场景（与 Cross / Ring 状态切换联动）
 */
enum class Scene
{
    NormalScene = 0,
    CrossScene  = 1,
    RingScene   = 2
};


/* ====================== 全局图像 ====================== */
extern cv::Mat rgb_img;     /* 摄像头 BGR 原图（NCNN 输入） */
extern cv::Mat gray_img;    /* 灰度图（与 rgb_img 同尺寸） */
extern cv::Mat gray_cut_img; /* 灰度图裁剪（巡线主流水线输入） */
extern cv::Mat bin_img;     /* OTSU + 闭运算后的二值图 */
extern cv::Mat bin_bird_img; /* 透视后的二值图 */
extern cv::Mat imgShow;     /* 可视化用图（en_show=false 时仅作占位） */



/* ====================== 全局点集 ====================== */
extern std::vector<POINT> pointsEdgeLeft, pointsEdgeRight;          /* 原图坐标，迷宫法输出 */
extern std::vector<POINT> t_pointsEdgeLeft, t_pointsEdgeRight;      /* 透视后边线 */
extern std::vector<POINT> b_t_pointsEdgeLeft, b_t_pointsEdgeRight;  /* 滤波 */
extern std::vector<POINT> s_b_t_pointsEdgeLeft, s_b_t_pointsEdgeRight; /* 等距采样 */
extern std::vector<POINT> a_t_pointsEdgeLeft, a_t_pointsEdgeRight;  /* 角度计算 */
extern std::vector<POINT> n_a_t_pointsEdgeLeft, n_a_t_pointsEdgeRight; /* NMS */
extern std::vector<POINT> t_left_CenterEdge, t_right_CenterEdge;    /* 单边中线 */
extern std::vector<POINT> t_CenterEdge;                             /* 最终选定中线（点多选边） */
extern std::vector<POINT> CenterEdge;                               /* 原图坐标的最终中线（送显） */

extern int  pointsEdgeLeft_size,        pointsEdgeRight_size;
extern int  t_pointsEdgeLeft_size,      t_pointsEdgeRight_size;
extern int  b_t_pointsEdgeLeft_size,    b_t_pointsEdgeRight_size;
extern int  s_b_t_pointsEdgeLeft_size,  s_b_t_pointsEdgeRight_size;
extern int  a_t_pointsEdgeLeft_size,    a_t_pointsEdgeRight_size;
extern int  n_a_t_pointsEdgeLeft_size,  n_a_t_pointsEdgeRight_size;
extern int  t_left_CenterEdge_size,     t_right_CenterEdge_size;
extern int  t_CenterEdge_size;
extern int  CenterEdge_size;

/* L 角点 */
extern bool      is_t_L_pointLeft_find,  is_t_L_pointRight_find;
extern int       t_L_pointLeft_id,       t_L_pointRight_id;
extern cv::Point t_L_pointLeft,          t_L_pointRight;

/* 直道 / 弯道判断 */
extern bool is_left_straight,  is_right_straight;
extern bool is_left_curve,     is_right_curve;

/* 控制偏差 */
extern float aim_angle;
extern int   scene;     /* NormalScene / CrossScene / RingScene */


/* ====================== 函数接口 ====================== */

/**
 * @brief 摄像头取帧
 * @param camera     摄像头对象引用
 * @param raw_img    [out] BGR 320x240 原图（NCNN 输入）
 * @param gray_img   [out] 灰度 320x240（巡线主流水线输入）
 * @return           取帧成功为 true
 * @note             内部对 raw_img / gray_img 做 flip(-1)（沿用本工程相机安装方向）
 */
bool image_get(lq_camera_ex &camera, cv::Mat &raw_img, cv::Mat &gray_img, cv::Mat &cut_gray_img);

/**
 * @brief 巡线主流水线
 * @sample image_get(camera, rgb_img, gray_img, cut_gray_img); image_process();
 * @note   读 rgb_img / gray_img 全局；偏差在 Image_Error_Get() 中计算
 *         不依赖任何 1D 边线数组，所有结果在 vector<POINT> 全局中
 */
void image_process(void);

/**
 * @brief 将十字、环岛状态机复位为 None，scene 置 NormalScene
 * @return 无
 * @sample track_elements_reset();
 * @note  供按键/调试调用；下一帧 image_process 会按普通巡线继续
 */
void track_elements_reset(void);

/**
 * @brief 中线横向像素偏差，写入 img_err 并返回 aim_angle
 * @return 横向偏差（像素），COLSIMAGE/2 - mean(t_CenterEdge[15..20].x)
 * @sample image_process(); float err = Image_Error_Get();
 * @note   三轮车后轮差速；中线偏右时返回值为负；实车需按像素量级重调 pd_yaw
 */
float Image_Error_Get(void);


/* ====================== 阶段 C 调试（透视边线 + L 角点） ====================== */

/**
 * @brief 阶段 C 调试快照（只读，供 HUD / 串口）
 */
struct TrackDebugSnapshotC
{
    int pl;   /**< pointsEdgeLeft_size */
    int pr;   /**< pointsEdgeRight_size */
    int tl;   /**< t_pointsEdgeLeft_size */
    int tr;   /**< t_pointsEdgeRight_size */
    int ll;   /**< 左 L 角点 0/1 */
    int lr;   /**< 右 L 角点 0/1 */
    int lxid; /**< 左 L id，未找到为 -1 */
    int lyid;
    int rxid;
    int ryid;
    int lx;   /**< 左 L 俯视图 x，未找到为 -1 */
    int ly;
    int rx;
    int ry;
};

/**
 * @brief 从全局巡线变量填充阶段 C 快照
 * @param out [out] 输出结构体
 * @return  无
 */
void track_debug_fill_phase_c(TrackDebugSnapshotC &out);

/**
 * @brief 阶段 C 低频串口输出（TRACK_DEBUG_LEVEL>=2 时调用）
 * @return 无
 * @note  约每 30 帧打印一次；L 角点 0→1 时额外打印坐标
 */
void track_debug_print_phase_c_throttled(void);


/* ====================== 阶段 D~G 调试（偏差 / 巡线侧 / 元素状态机） ====================== */

/**
 * @brief 阶段 D~G 调试快照（只读，供 HUD / 串口）
 */
struct TrackDebugSnapshotD
{
    float aim_angle;   /**< 中线横向像素偏差 */
    int   center_ok;   /**< normalizeCenterEdge 成功 1/0 */
    int   track_cx;    /**< 车体参考点列（俯视图） */
    int   track_cy;    /**< 车体参考点行（俯视图） */
    int   track_state; /**< 0=NONE 1=LEFT 2=RIGHT */
    int   scene;       /**< NormalScene/CrossScene/RingScene */
    int   cross_flag;  /**< Cross::flag_Cross_e */
    int   ring_flag;   /**< Ring::flag_ring_e */
    int   straight_l;  /**< 左直道 0/1 */
    int   straight_r;  /**< 右直道 0/1 */
    int   curve_l;     /**< 左弯道 0/1 */
    int   curve_r;     /**< 右弯道 0/1 */
};

/**
 * @brief 从全局巡线变量填充阶段 D~G 快照
 * @param out [out] 输出结构体
 * @return  无
 */
void track_debug_fill_phase_d(TrackDebugSnapshotD &out);

/**
 * @brief 阶段 D~G 低频串口输出（TRACK_DEBUG_LEVEL>=2 时调用）
 * @return 无
 * @note  约每 30 帧打印一次；十字/环岛状态变化时额外打印
 */
void track_debug_print_phase_d_throttled(void);

#endif /* IMAGE_HPP */
