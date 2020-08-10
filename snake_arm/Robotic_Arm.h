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

void Forward_Kinematics(Matrix* DH_Para, Matrix* T[],int n);  //机械臂正运动学




#endif /* Robotic_Arm_h */
