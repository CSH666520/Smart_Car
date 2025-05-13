#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "headfile.h"

typedef struct{

	float kp;
	float ki;
	float kd;
	float err;
	float last_err;   //�ϴ����
	float err_sum;    //I�����ۼ�ֵ      
	float kp_out;     //P�������ֵ     
	float ki_out;     //I�������ֵ      
	float kd_out;     //D�������ֵ 
        int16_t  output;

}PID;

extern float turn_kp;
extern float turn_kd;
extern float blance_turn ;
extern float Now_turn;

extern int Speed_left ;
extern int Speed_right  ;


extern PID  PID_Angle,PID_Left,PID_right;

extern int PWM_A;
extern int PWM_B;


void Speed_Limit();
void PID_init(PID* PID,float kp,float ki,float kd);
float Angle_PID (PID*PID,float Now_Angle, float target_Angle);
int16 position_PID(PID* PID,float Now_speed,int16_t target_speed);
void Position_Velocity_Controller(int16 velocity_right,int16_t velocity_left);
float PID_Turn(float kp , float kd, float Now_yaw, float blance_yaw);




#endif
