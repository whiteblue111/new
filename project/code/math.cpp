 #include "zf_common_headfile.hpp"
 #include "math.hpp"


//求绝对值函数
int my_abs(int value)
{
    if (value >= 0)
        return value;
    else
        return -value;
}
//浮点数限幅函数
float limit_float(float a, float min, float max)
{
    if (a < min)
        a = min;
    if (a > max)
        a = max;
    return a;
}
//整数限幅函数
int limit_int(int x,int min, int max)
{
    if (x < min)
        x = min;
    if (x > max)
        x = max;
    return x;
}

//取最小值函数
int limit1(int x, int y)
{
    if (x > y)
        return y;
    else if (x < -y)
        return -y;
    else
        return x;
}