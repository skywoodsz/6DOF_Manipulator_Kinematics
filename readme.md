## 1. 描述  Description  
本程序基于*POE*对通用六轴机械臂进行正运动学结算, 利用*Paden-Kahan subproblem*对逆运动学进行结算, 最终对机械臂进行运动学控制.  
We use the *POE* and the *Paden-Kahan subproblem* to solve the forward kinematics and the inverse kinematics of the 6 DOF manipulator.  

## 2. 依赖  Dependence  
Eigen3  only  

## 3. 运行  Run    
`
kuka_control.cpp/h: 6DOF kinematics class  
  
robotics_math.cpp/h: robotics math function
`
