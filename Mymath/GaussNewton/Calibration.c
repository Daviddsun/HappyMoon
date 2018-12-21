#include "Calibration.h"
/**********************************************************************************************************
*函 数 名: ResetMatrices
*功能说明: 矩阵初始化
*参    数: 传入矩阵值
*返 回 值: 无
**********************************************************************************************************/
static void ResetMatrices(float JtR[6], float JtJ[6][6])
{
    int16_t j,k;
    for(j=0; j<6; j++) 
    {
        JtR[j] = 0.0f;
        for(k=0; k<6; k++) 
        {
            JtJ[j][k] = 0.0f;
        }
    }
}

/**********************************************************************************************************
*函 数 名: UpdateMatrices
*功能说明: 更新函数
*参    数: 传入矩阵值
*返 回 值: 无
**********************************************************************************************************/
static void UpdateMatrices(float JtR[6], float JtJ[6][6], float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    
    for(j=0; j<3; j++) 
    { 
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        //计算残差 (传感器误差方程的误差)
        residual -= b*b*dx*dx;
        
        //计算雅可比矩阵
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }
    
    for(j=0; j<6; j++) 
    {
        //计算函数梯度
        JtR[j] += jacobian[j]*residual;
        
        for(k=0; k<6; k++) 
        {
            //计算Hessian矩阵（简化形式，省略二阶偏导），即雅可比矩阵与其转置的乘积
            JtJ[j][k] += jacobian[j]*jacobian[k];
        }
    }
}
/**********************************************************************************************************
*函 数 名: GaussEliminateSolveDelta
*功能说明: 高斯消元法求解函数
*参    数: 传入矩阵值
*返 回 值: 无
**********************************************************************************************************/
static void GaussEliminateSolveDelta(float JtR[6], float JtJ[6][6], float delta[6])
{
    int16_t i,j,k;
    float mu;
   
    //逐次消元，将线性方程组转换为上三角方程组
    for(i=0; i<6; i++)
    {
        //若JtJ[i][i]不为0，将该列在JtJ[i][i]以下的元素消为0
        for(j=i+1; j<6; j++)
        {
            mu = JtJ[i][j] / JtJ[i][i];
            if(mu != 0.0f) 
            {
                JtR[j] -= mu * JtR[i];
                for(k=j; k<6; k++)
                {
                    JtJ[k][j] -= mu * JtJ[k][i];
                }
            }
        }
    }

    //回代得到方程组的解
    for(i=5; i>=0; i--)
    {
        JtR[i] /= JtJ[i][i];
        JtJ[i][i] = 1.0f;
        
        for(j=0; j<i; j++)
        {
            mu = JtJ[i][j];
            JtR[j] -= mu * JtR[i];
            JtJ[i][j] = 0.0f;
        }
    }

    for(i=0; i<6; i++)
    {
        delta[i] = JtR[i];
    }
}

/**********************************************************************************************************
*函 数 名: GaussNewton
*功能说明: 高斯牛顿法求解传感器误差方程，得到校准参数
*参    数: 传感器采样数据（6组） 零偏差指针 比例误差指针 数据向量长度
*返 回 值: 无
**********************************************************************************************************/
void GaussNewton(Vector3f_t inputData[6], Vector3f_t* offset, Vector3f_t* scale, char length)
{
    uint32_t cnt    = 0;
    double   eps    = 0.0000001;
    double   change = 100.0;
    float    data[3];  
    float    beta[6];      //方程解
    float    delta[6];     //迭代步长
    float    JtR[6];       //梯度矩阵
    float    JtJ[6][6];    //Hessian矩阵
   
    //设定方程解初值
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1 / length;

    //开始迭代 当迭代步长小于eps时结束，得到方程近似最优解
    while(change > eps) 
    {
        //矩阵初始化
        ResetMatrices(JtR, JtJ);

        //计算误差方程函数的梯度JTR和Hessian矩阵JTJ
        for(uint8_t i=0; i<6; i++) 
        {
            data[0] = inputData[i].x;
            data[1] = inputData[i].y;
            data[2] = inputData[i].z;
            UpdateMatrices(JtR, JtJ, beta, data);
        }

        //高斯消元法求解方程：JtJ * delta = JtR，得到delta
        GaussEliminateSolveDelta(JtR, JtJ, delta);

        //计算迭代步长
        change = delta[0]*delta[0] +
                 delta[0]*delta[0] +
                 delta[1]*delta[1] +
                 delta[2]*delta[2] +
                 delta[3]*delta[3] / (beta[3]*beta[3]) +
                 delta[4]*delta[4] / (beta[4]*beta[4]) +
                 delta[5]*delta[5] / (beta[5]*beta[5]);

        //更新方程解
        for(uint8_t i=0; i<6; i++) 
        {
            beta[i] -= delta[i];
        }
            
        //限制迭代次数
        if(cnt++ > 100)
            break;
    }

    //更新校准参数
    scale->x  = beta[3] * length;
    scale->y  = beta[4] * length;
    scale->z  = beta[5] * length;
    offset->x = beta[0];
    offset->y = beta[1];
    offset->z = beta[2];
}


