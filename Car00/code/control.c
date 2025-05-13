#include "control.h"
#include "zf_common_headfile.h"


//
void Speed_Limit(void);
void PID_Control(void);


///////////////////////////////////////////////////////////////////////////////////////////////
PID  PID_Angle,PID_Left,PID_right;

int16_t  right_out,left_out;

int PWM_A;
int PWM_B;

int Speed_left= 50;
int Speed_right= 50;

//×a?ò?·
float turn_kp = 0; //-8
float turn_kd = 0; //-0.04
float blance_turn = 30.0;
float Now_turn = 0;

void PID_init(PID* PID,float kp,float ki,float kd)
{
	PID->kp=kp;
	PID->ki=ki;
	PID->kd=kd;
}

float Angle_PID (PID*PID,float Now_Angle, float target_Angle)  //Now_Angle是当前角度，target_Angle是目标角度
{
        PID->err = target_Angle-Now_Angle;
        PID->kp_out=PID->kp*PID->err;
        PID->kd_out=PID->kd*(PID->err-PID->last_err);
        PID->last_err=PID->err;
        if ( PID->err > 300 )
        {
          PID->err =  -100;
          PID->last_err = 0;
        }
        if ( PID->err < -300 )
        {
          PID->err =  100;
          PID->last_err = 0;
        }
        PID->output=(int16_t)(PID->kp_out + PID->kd_out);
        if( PID->output>=10000)
         PID->output=10000;
        else if(PID->output<=-10000)
           PID->output=-10000;
        return  PID->output; 
}
int16 position_PID(PID* PID,float Now_speed,int16_t target_speed)
{
      PID->err = target_speed - Now_speed;
      PID->err_sum+=PID->err;
      if( PID->err_sum >=10000)
         PID->err_sum=10000;
      else if( PID->err_sum<-10000)
         PID->err_sum=-10000;
      
      PID->kp_out=PID->kp*PID->err;
      PID->ki_out=PID->ki*PID->err_sum;
      PID->kd_out=PID->kd*(PID->err-PID->last_err);
      PID->last_err=PID->err;
      PID->output=(int16_t)(PID->kp_out + PID->ki_out+ PID->kd_out)  ;
        if( PID->output>=10000)
         PID->output=10000;
        else if(PID->output<=-10000)
           PID->output=-10000;
        return  PID->output; 
}
void Position_Velocity_Controller(int16 velocity_right,int16_t velocity_left)
{
      left_out= position_PID(&PID_Left,encoder_data_dir[1],velocity_left);
      right_out= position_PID(&PID_right,encoder_data_dir[2],velocity_right);
        Motor_SetSpeed(left_out, right_out);
}

float PID_Turn(float kp , float kd, float Now_yaw, float blance_yaw)
{
	float bias_yaw;
	bias_yaw = Now_yaw - blance_yaw;
//	if(bias_yaw > 100) 
//	{
//		bias_yaw = -10;
//	}
//	if(bias_yaw < -100)
//	{
//		bias_yaw = 10;
//	}
	return kp*bias_yaw + kd*imu660ra_gyro_z;
}


void Speed_Limit(void) //  若小车开机而不跑导致积分太大，则限制小一些
{
	if(PWM_A>400)PWM_A=400;
	if(PWM_B>400)PWM_B=400;
}
