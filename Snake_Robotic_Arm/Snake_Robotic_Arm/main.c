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

#define FREEDOM 6

int main(int argc, const char * argv[]) {
    // insert code here...
    printf("Hello, World!\n");
    
    /*               theta,   d,     a,    alpha                     */
    double DH[FREEDOM][4] = {{0,     0,     0.052,     PI/2},
                        {0,     0,     0.052,     PI/2},
                        {0,     0,     0.052,     PI/2},
                        {0,     0,     0.052,     PI/2},
                        {0,     0,     0.052,     PI/2},
                        {0,     0,     0.059,     PI/2},};
    
    //double b[2][2] = {{3,2},{2,1}};
    
    Matrix *A = InitMatrix(A, FREEDOM, 4);
    //Matrix *B = InitMatrix(B, 1, 3);
    
    ValueMatrix(A, DH);
    //ValueMatrix(B, b);
    //printf("%f",GetValue(A, 1, 3));
    
    
    Matrix* T[FREEDOM];
    for (int i = 0; i < FREEDOM; i ++) {
        T[i] = InitMatrix(T[i], 4, 4);
        IdentityMatrix(T[i]);
    }
    
    Forward_Kinematics(A, T, FREEDOM);
    
    
    
    Matrix* J;
    J = InitMatrix(J, 6, FREEDOM);
          
    Jacobian(T, J, FREEDOM);
    
    
    
//    for (int i = 0; i < FREEDOM; i++) {
//        PrintMatrix(T[i]);
//        printf("\n\n\n");
//    }
    
    PrintMatrix(J);

    
    //printf("\n\n\n");
    
    
    //PrintMatrix(MulMatrix(A,B));
    
//    //printf("\n\n\n");
//    PrintMatrix(B);
//    printf("\n\n\n");
//    PrintMatrix(Inv(B));
    
    
    return 0;
}
