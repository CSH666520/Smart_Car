#ifndef _PID_2_H
#define _PID_2_H
 

//转向环
extern float turn_Yaw_kp = 3; //4
extern float turn_Yaw_kd = 0.01; //0.01
extern float blance_turn = 0.0;

float PID_Turn(float kp , float kd, float Now_yaw, float blance_yaw, float gz);

//void steering_ring(void);//转向环
//void Track_ring(void);//寻迹环
//void steering_ring(void);           //转向环
//void Track_ring(void);              //寻迹环
 
#endif
