#include "MathTool.h"


/**********************************************************************************************************
*函 数 名: constrainFloat
*功能说明: 浮点型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
float ConstrainFloat(float amt, float low, float high)
{
    if (isnan(amt))
    {
        return (low+high)*0.5f;
    }
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**********************************************************************************************************
*函 数 名: ConstrainInt16
*功能说明: 16位有符号整型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
int16_t ConstrainInt16(int16_t amt, int16_t low, int16_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**********************************************************************************************************
*函 数 名: ConstrainUint16
*功能说明: 16位无符号整型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
uint16_t ConstrainUint16(uint16_t amt, uint16_t low, uint16_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**********************************************************************************************************
*函 数 名: ConstrainInt32
*功能说明: 32位整型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
int32_t ConstrainInt32(int32_t amt, int32_t low, int32_t high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


/**********************************************************************************************************
*函 数 名: Sq
*功能说明: 求平方
*形    参: 输入值
*返 回 值: 输入值的平方
**********************************************************************************************************/
float Sq(float v)
{
    return v*v;
}

/**********************************************************************************************************
*函 数 名: Pythagorous2
*功能说明: 2维向量长度
*形    参: 变量a 变量b
*返 回 值: 模值大小
**********************************************************************************************************/
float Pythagorous2(float a, float b)
{
    return sqrtf(Sq(a)+Sq(b));
}

/**********************************************************************************************************
*函 数 名: Pythagorous3
*功能说明: 3维向量长度
*形    参: 变量a 变量b 变量c
*返 回 值: 模值大小
**********************************************************************************************************/
float Pythagorous3(float a, float b, float c)
{
    return sqrtf(Sq(a)+Sq(b)+Sq(c));
}

/**********************************************************************************************************
*函 数 名: Pythagorous4
*功能说明: 4维向量长度
*形    参: 变量a 变量b 变量c 变量d
*返 回 值: 模值大小
**********************************************************************************************************/
float Pythagorous4(float a, float b, float c, float d)
{
    return sqrtf(Sq(a)+Sq(b)+Sq(c)+Sq(d));
}


