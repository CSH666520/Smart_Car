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
        Follow_Err= 46 - longest_White_Column_site;
        if( Follow_Err > 30 )
        {
          Follow_Err = 0;
        }
        if( Follow_Err < -30 )
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
        
        if( road_type.left_right_angle_bend == 1 )
        {
          zhijiao_left = 1;
          zhijiao_right = 0;
        }
        
        if( zhijiao_right == 1 && longest_White_Column <= 20 )
        {
          flag_Starturn_right = 1;
        }
        
        if( flag_Starturn_right == 1 )
        {
          if ( flag_angle == 0 )
          {
             Now_turn = New_angle;
             blance_turn = 60;
             turn_kd = -8.0;
             turn_kp = -0.04;
             flag_angle = 1;
          }
          
          if ( ( New_angle - Now_turn ) > 55 )
          {
             Now_turn = 0;
             blance_turn = 0;
             turn_kd = 0;
             turn_kp = 0;
             flag_Starturn_right = 0;
             zhijiao_right = 0;
             flag_angle = 0;
          }
        }
        
        if( zhijiao_left == 1 && longest_White_Column <= 20 )
        {
        }
}


