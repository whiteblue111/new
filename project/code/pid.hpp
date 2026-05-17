#ifndef PID_HPP  
#define PID_HPP  
  
/* ====================== 增量式PID ====================== */  
typedef struct  
{  
    float P;  
    float I;  
    float D;  
  
    float LastError;  // 上一次误差  
    float PrevError;  // 上上次误差  
} PID_Inc_Datatypedef;  
  
/**  
 * @brief 增量式PID计算  
 * @param sptr PID结构体指针  
 * @param error 当前误差  
 * @return 输出增量  
 */  
float PID_Inc(PID_Inc_Datatypedef *sptr, float error);  
  
/* ====================== 位置式PID ====================== */  
typedef struct  
{  
    float P;  
    float I;  
    float D;  
  
    float Integral;       // 积分累计值  
    float IntegralLimit;  // 积分限幅（防止积分饱和）  
    float Last_Err;       // 上一次误差  
} PID_Pos_Datatypedef; 
typedef struct 
{
    float P1;
    float P2;
    float D1;
    float D2;
    float err_last;
} PD_Double;

/**  
 * @brief 位置式PID计算  
 * @param sptr PID结构体指针  
 * @param error 当前误差（目标值 - 实际值）  
 * @return PID输出  
 */  
float PID_Pos(PID_Pos_Datatypedef *sptr, float error);  
  
/* ====================== 初始化辅助宏 ====================== */  
  
/** 增量式PID初始化 */  
#define PID_INC_INIT(p, i, d) \  
    { (p), (i), (d), 0.0f, 0.0f }  
  
/** 位置式PID初始化（含积分限幅） */  
#define PID_POS_INIT(p, i, d, int_lim) \  
    { (p), (i), (d), 0.0f, (int_lim), 0.0f }  
 
/** 双PD初始化（P1, P2, D1, D2, err_last） */  
#define PD_DOUBLE_INIT(p1, p2, d1, d2) \  
    { (p1), (p2), (d1), (d2), 0.0f }  

  
/* ====================== 重置函数 ====================== */  
  
/**  
 * @brief 重置增量式PID历史状态  
 */  
static inline void PID_Inc_Reset(PID_Inc_Datatypedef *sptr)  
{  
    sptr->LastError = 0.0f;  
    sptr->PrevError = 0.0f;  
}  
  
/**  
 * @brief 重置位置式PID积分和历史状态  
 */  
static inline void PID_Pos_Reset(PID_Pos_Datatypedef *sptr)  
{  
    sptr->Integral = 0.0f;  
    sptr->Last_Err = 0.0f;  
}  
float PD_Loc_Ctrl_2PD(PD_Double *pd, float err, float gyro_z);
  
#endif // PID_HPP  