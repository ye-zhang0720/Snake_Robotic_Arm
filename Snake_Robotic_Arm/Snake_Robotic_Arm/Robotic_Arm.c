//
//  Robotic_Arm.c
//  snake_arm
//
//  Created by 西北偏北 on 2020/8/8.
//  Copyright © 2020 西北偏北. All rights reserved.
//

#include "Robotic_Arm.h"

//*************************************************
//改进D-H建
//DH_Para[theta, d, a, alp]
//theta:     Joint Angles (theta)
//d:     Link offset
//a:     Link Length
//alp:   Link Twist (alpha)
//T:     关节旋转矩阵，参考定义如下
//Matrix* T[6];
//for (int i = 0; i < 6; i ++) {
//    T[i] = InitMatrix(T[i], 4, 4);
//    IdentityMatrix(T[i]);
//}
//*************************************************
void Forward_Kinematics(Matrix* DH_Para, Matrix* T[],int n)
{
    
    for (int i = 0; i < n; i ++) {
  
        //第一行
        ValueOneMatrix(T[i], cos(GetValue(DH_Para, i, 0)), 0, 0);
        ValueOneMatrix(T[i], -sin(GetValue(DH_Para, i, 0)), 0, 1);
        ValueOneMatrix(T[i], 0, 0, 2);
        ValueOneMatrix(T[i], GetValue(DH_Para, i, 2), 0, 3);
        
        //第二行
        ValueOneMatrix(T[i], sin(GetValue(DH_Para, i, 0))*cos(GetValue(DH_Para, i, 3)), 1, 0);
        ValueOneMatrix(T[i], cos(GetValue(DH_Para, i, 0))*cos(GetValue(DH_Para, i, 3)), 1, 1);
        ValueOneMatrix(T[i], -sin(GetValue(DH_Para, i, 3)), 1, 2);
        ValueOneMatrix(T[i], -sin(GetValue(DH_Para, i, 3))*GetValue(DH_Para, i, 1), 1, 3);
        
        //第三行
        ValueOneMatrix(T[i], sin(GetValue(DH_Para, i, 0))*sin(GetValue(DH_Para, i, 3)), 2, 0);
        ValueOneMatrix(T[i], cos(GetValue(DH_Para, i, 0))*sin(GetValue(DH_Para, i, 3)), 2, 1);
        ValueOneMatrix(T[i], cos(GetValue(DH_Para, i, 3)), 2, 2);
        ValueOneMatrix(T[i], cos(GetValue(DH_Para, i, 3))*GetValue(DH_Para, i, 1), 2, 3);
        
        //第四行
        ValueOneMatrix(T[i], 0, 3, 0);
        ValueOneMatrix(T[i], 0, 3, 1);
        ValueOneMatrix(T[i], 0, 3, 2);
        ValueOneMatrix(T[i], 1, 3, 3);
        if (i != 0)
            T[i] = MulMatrix(T[i-1],T[i]);
    }
}


//*************************************************************
//改进D-H法建模
//该方法求出的雅可比矩阵是相对于基座标系{0}的
//T为旋转矩阵  J[i]第i个关节，第一列为线速度，第二列为角速度  n机械臂自由度
//**************************************************************
void Jacobian(Matrix* T[], Matrix* J, int n)
{
    Matrix *k = InitMatrix(k, 3, 1);
    Matrix *R = InitMatrix(R, 3, 3);
    Matrix *T_i_n = InitMatrix(T_i_n, 4, 4);
    Matrix *p_i_n = InitMatrix(p_i_n, 3, 1);
    Matrix *z_i = InitMatrix(z_i, 3, 1);
    Matrix *temp = InitMatrix(temp, 3, 1);
    ZerosMatrix(k);
    ValueOneMatrix(k, 1, 2, 0);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                ValueOneMatrix(R, GetValue(T[i], j, k), j, k);
            }
        }
        
        T_i_n = MulMatrix(Inv(T[i]), T[n-1]);
        
        for (int j = 0; j < 3; j++) {
            p_i_n->data[j] = GetValue(T_i_n, j, 3);
        }
        

        z_i = MulMatrix(R, k);
        
        temp = cross(z_i, MulMatrix(R, p_i_n));
        
        
        ValueOneMatrix(J, temp->data[0], 0, i);
        ValueOneMatrix(J, temp->data[1], 1, i);
        ValueOneMatrix(J, temp->data[2], 2, i);
        
        ValueOneMatrix(J, z_i->data[0], 3, i);
        ValueOneMatrix(J, z_i->data[1], 4, i);
        ValueOneMatrix(J, z_i->data[2], 5, i);
    }
}

