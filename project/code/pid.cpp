#include "pid.hpp"
#include "zf_common_headfile.hpp" 
  
/* ====================== 增量式PID ====================== */  
float PID_Inc(PID_Inc_Datatypedef *sptr, float error)  
{  
    float Increase;  
  
    Increase = sptr->P * (error - sptr->LastError)  
             + sptr->I * error  
             + sptr->D * (error - 2.0f * sptr->LastError + sptr->PrevError);  
  
    sptr->PrevError = sptr->LastError;  
    sptr->LastError = error;  
  
    return Increase;  
}  
  
/* ====================== 位置式PID ====================== */  
float PID_Pos(PID_Pos_Datatypedef *sptr, float error)  
{  
    float Pout, Iout, Dout;   
    Pout = sptr->P * error;  
    //积分限幅  
    sptr->Integral += error;  
    if (sptr->IntegralLimit > 0.0f)  
    {  
        if      (sptr->Integral >  sptr->IntegralLimit) sptr->Integral =  sptr->IntegralLimit;  
        else if (sptr->Integral < -sptr->IntegralLimit) sptr->Integral = -sptr->IntegralLimit;  
    }  
    Iout = sptr->I * sptr->Integral;  
  
    /* 微分项 */  
    Dout = sptr->D * (error - sptr->Last_Err);  
  
    /* 更新历史 */  
    sptr->Last_Err = error;  
  
    return Pout + Iout + Dout;  
}  


  
/*************************************************************************  
*  函数名称：PD_Loc_Ctrl_2PD  
*  功能说明：双PD位置控制（一次P + 二次P + 误差D + 陀螺仪D）  
*  参数说明：  
*    @param pd       双PD参数结构体  
*    @param err      当前误差  
*    @param gyro_z   陀螺仪Z轴角速度  
*    @param out_min  输出下限  
*    @param out_max  输出上限  
*  返回值：控制输出  
*************************************************************************/  
float PD_Loc_Ctrl_2PD(PD_Double *pd, float err, float gyro_z)  
{  
    // 一次比例  
    float p1_out = pd->P1 * err;  
  
    // 二次比例（保留符号：err*|err|）  
    float p2_out = pd->P2 * err * fabsf(err);  
  
    // 误差微分  
    float d1_out = pd->D1 * (err - pd->err_last);  
  
    // 陀螺仪阻尼（通常取负反馈方向，符号按你的实际方向再确认）  
    float d2_out = - pd->D2 * gyro_z;  
  
    // 更新历史误差  
    pd->err_last = err;  
    // 总输出限幅  
    float out = p1_out + p2_out + d1_out + d2_out;  
    return out;  
}  

