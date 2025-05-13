#include "Follow_Line.h"
#include "control.h"

extern struct YUAN_SU road_type ; 

int Follow_Err ;
int Follow_LastErr ;
int Follow_Err_Sum ;
float Follow_Kp = 5 ; //-100  -10
float Follow_Ki = -0 ; //-1   -0.5
int Now_Follow= 0 ;

//////////
int zhijiao_right = 0;
int zhijiao_left = 0;
int flag_Starturn_right = 0;
int flag_Starturn_left = 0;
int flag_angle = 0;


void Get_Follow_Err(void);
int Make_Car_Follow_Line_PWM(void);

void Get_Follow_Err(void)
{       
  if( flag_Starturn_right != 1 && flag_Starturn_left != 1)
  {
        Follow_Err= 46 - longest_White_Column_site;
  }
        if( Follow_Err > 40 )
        {
          Follow_Err = 0;
        }
        if( Follow_Err < -42 )
        {
          Follow_Err = 0;
        }
	Follow_LastErr = Follow_Err ;
	Follow_Err_Sum += Follow_Err ;
}

int Make_Car_Follow_Line_PWM(void)
{
	Get_Follow_Err();
	return (int)(Follow_Kp * Follow_Err + Follow_Ki * Follow_Err_Sum );
}

void turn_zhijiao(void)
{
        if( road_type.right_right_angle_bend == 1 )
        {
          zhijiao_right = 1;
          zhijiao_left = 0;
        }
        
        if( zhijiao_right == 1 && longest_White_Column <= 10 )
        {
          flag_Starturn_right = 1;
          flag_angle = 0;
        }
        
        if( flag_Starturn_right == 1 )
        {
          if ( flag_angle == 0 )
          { 
             turn_kp = -8; //8
             turn_kd = -0.04; //0.04
             Now_turn = New_angle;
             blance_turn = 50;
             flag_angle = 1;
             Speed_left = 0;
             Speed_right = 0;
          }
          
         if ( imu660ra_gyro_z < 50 && imu660ra_gyro_z > -50 )
          {         
            turn_kp = 0; //-8
            turn_kd = 0; //-0.04
            Now_turn = 0;
            blance_turn = 0;
            flag_Starturn_right = 0;
            zhijiao_right = 0;
            Speed_left = 50;
            Speed_right = 50;
          }
        }
        
        /////////////////////////////////////////////////////////////////////
        
        if( road_type.left_right_angle_bend == 1 )
        {
          zhijiao_left = 1;
          zhijiao_right = 0;
        }
        
        
        if( zhijiao_right == 1 && longest_White_Column <= 10 )
        {
          flag_Starturn_left = 1;
          flag_angle = 0;
        }
        
        if( flag_Starturn_left == 1 )
        {
          if ( flag_angle == 0 )
          { 
             turn_kp = -8; //8
             turn_kd = -0.04; //0.04
             Now_turn = New_angle;
             blance_turn = -50;
             flag_angle = 1;
             Speed_left = 0;
             Speed_right = 0;
          }
          
         if ( imu660ra_gyro_z < 50 && imu660ra_gyro_z > -50 )
          {         
            turn_kp = 0; //-8
            turn_kd = 0; //-0.04
            Now_turn = 0;
            blance_turn = 0;
            flag_Starturn_left = 0;
            zhijiao_left = 0;
             Speed_left = 50;
             Speed_right = 50;
          }
        }
}


