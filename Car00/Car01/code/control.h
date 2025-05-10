#ifndef __CONTROL_H__
#define __CONTROL_H__


extern int PWM_A;
extern int PWM_B;

//×ªÏò»·
extern float turn_kp ; //4
extern float turn_kd ; //0.01
extern float blance_Yaw ;

float PID_Turn(float kp , float kd, float Now_yaw, float blance_yaw,int gz);
void PID_Control(void);
void Speed_Limit();


#endif
