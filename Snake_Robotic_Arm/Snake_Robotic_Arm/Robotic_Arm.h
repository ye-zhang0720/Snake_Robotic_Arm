//
//  Robotic_Arm.h
//  snake_arm
//
//  Created by 西北偏北 on 2020/8/8.
//  Copyright © 2020 西北偏北. All rights reserved.
//

#ifndef Robotic_Arm_h
#define Robotic_Arm_h

#include <stdio.h>
#include "Matrix.h"


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
void Forward_Kinematics(Matrix* DH_Para, Matrix* T[],int n);  //机械臂正运动学

//*************************************************************
//改进D-H法建模
//该方法求出的雅可比矩阵是相对于基座标系{0}的
//T为旋转矩阵  J[i]第i个关节，第一列为线速度，第二列为角速度  n机械臂自由度
//**************************************************************
void Jacobian(Matrix* T[], Matrix* J, int n);



#endif /* Robotic_Arm_h */
