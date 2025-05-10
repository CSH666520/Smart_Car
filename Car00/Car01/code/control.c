#include "motor.h"
#include "Get_Angle.h"
#include "control.h"

int PWM_A = 0;
int PWM_B = 0;

//转向环
float turn_kp = 0; //4
float turn_kd = 0; //0.01
float blance_yaw = 0.0;

//速度环
float Speed_kp_Left= 0;
float Speed_ki_Left= 0.0;
float Speed_kd_Left = 0;
float Tat_Left = 0;

float Err_left;
float sum_left;
float last_err_left;	


float Speed_kp_Right= 0;
float Speed_ki_Right= 0;
float Speed_kd_Right= 0;
float Tar_Right = 0;

float Err_right;
float sum_right;
float last_err_right;

//
void Speed_Limit(void);
void PID_Control(void);
float PID_Turn(float kp , float kd, float Now_yaw, float blance_yaw,int gz);
float PID_Normal_Right(float kp, float ki, float kd, float speed1, float Tar1);
float PID_Normal_Left(float kp, float ki, float kd, float speed1, float Tar1);
//转向环
float PID_Turn(float kp , float kd, float Now_yaw, float blance_yaw,int gz)
{
	float bias_yaw;
	bias_yaw = Now_yaw - blance_yaw;
	if(bias_yaw > 100) 

	{
		bias_yaw = -10;
	}
	if(bias_yaw < -100)
	{
		bias_yaw = 10;
	}
	return kp*bias_yaw + kd*gz;
}

//速度环
float PID_Normal_Left(float kp, float ki, float kd, float speed1, float Tar1)
{
	float bias;
	Err_left=Tar1-speed1;
        sum_left+=Err_left;
	bias=Err_left-last_err_left;
	if(sum_left>1000) sum_left=1000;
	if(sum_left<-1000) sum_left=-1000;
	
	float pwm=kp*Err_left+ki*sum_left+kd*bias;
        last_err_left=Err_left;	
	
	return pwm;

}

//速度环
float PID_Normal_Right(float kp, float ki, float kd, float speed1, float Tar1)
{
	float bias;
	Err_right=Tar1-speed1;
        sum_right+=Err_right;
	bias=Err_right-last_err_right;
	if(sum_right>1000) sum_right=1000;
	if(sum_right<-1000) sum_right=-1000;
	
	float pwm=kp*Err_right+ki*sum_right+kd*bias;
        last_err_right=Err_right;	
	
	return pwm;

}


void PID_Control(void)
{
	float Turn_Control = PID_Turn( turn_kp, turn_kd , New_angle, blance_yaw,imu660ra_gyro_z);
        float Speed_Left = PID_Normal_Right(Speed_kp_Left, Speed_ki_Left, Speed_kd_Left, encoder_data_dir[1] , 150);
        float Speed_Righ = PID_Normal_Right(Speed_kp_Right,Speed_ki_Right,Speed_kd_Right, encoder_data_dir[2], 150);
	PWM_A = (int)Speed_Left;
	PWM_B = 0;
	Speed_Limit();
	Motor_SetSpeed( PWM_A, PWM_B );
}



void Speed_Limit(void) //  若小车开机而不跑导致积分太大，则限制小一些
{
	if(PWM_A>80)PWM_A=80;
	else if(PWM_A<0)PWM_A=0;
	if(PWM_B>80)PWM_B=80;
	else if(PWM_B<0)PWM_B=0;

}
