//
//  main.c
//  snake_arm
//
//  Created by 西北偏北 on 2020/8/8.
//  Copyright © 2020 西北偏北. All rights reserved.
//

#include <stdio.h>
#include "Matrix.h"

#define PI 3.141592653

#define FREEDOM 7



int main(int argc, const char * argv[]) {
    // insert code here...
    printf("Hello, World!\n");
    
    //DH参数定义
    float L1=0;
    float L2=0.052;
    float Lh=0.059;
    float th1=PI/2;
    float th2=0;

    float du=PI/180;

    /*定义DH参数*/
    /*               theta,   d,     a,    alpha                     */
    double DH_par[FREEDOM][4] = {{-10*du,     0,     0,     0},
                                 {30*du,     0,     L2,     th1},
                                 {40*du,     0,     L2,     th1},
                                 {30*du,     0,     L2,     th1},
                                 {80*du,     0,     L2,     th1},
                                 {30*du,     0,     L2,     th1},
                                 {0,         0,     Lh,     th1}};
    
    
    Matrix *DH = InitMatrix(DH, FREEDOM, 4);
    
    ValueMatrix(DH, DH_par);
    
    Matrix* T[FREEDOM];
    for (int i = 0; i < FREEDOM; i ++) {
        T[i] = InitMatrix(T[i], 4, 4);
        IdentityMatrix(T[i]);
    }
    
    Forward_Kinematics(DH, T, FREEDOM);
    
    
    
    Matrix* J;
    J = InitMatrix(J, 6, FREEDOM-1);
          
    Jacobian(T, J, FREEDOM);
    
    
    PrintMatrix(J);
    
    printf("\n\n\n");
    
    double speed[]={0,0,0,0.04,0,0};  //末端运动速度（只有x轴旋转）
    Matrix *Speed = InitMatrix(Speed, 6, 1);
    ValueMatrix(Speed, speed);
    
    float detT=0.1;           //时间步长
    
    Matrix *detq = MulMatrix(Inv(J), Speed);
    PrintMatrix(detq);
    printf("\n\n\n");
    
    TransMatrix(detq);
    NumMulMatrix(detq, detT);
    PrintMatrix(detq);
    printf("\n\n\n");
    
    
    double S[] = {-10*du,30*du,40*du,30*du,80*du,30*du};
    
    for (int i = 0; i < detq->column; i++) {
        ValueOneMatrix(detq, GetValue(detq, 0, i)+S[i], 0, i);
    }
    
    PrintMatrix(detq);
    
    return 0;
}
