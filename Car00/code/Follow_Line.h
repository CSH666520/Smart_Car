#ifndef __FOLLOW_LINE_
#define __FOLLOW_LINE_

#include "stdint.h"
#include "camera.h"
extern int Follow_Err ;
extern int Follow_Err_Sum ;
extern float Follow_Kp ;
extern float Follow_Ki ;

extern int zhijiao_right ;
extern int zhijiao_left ;
extern int flag_Starturn_right ;
extern int flag_Starturn_left ;
extern int flag_angle ;

void Get_Follow_Err(void);
int Make_Car_Follow_Line_PWM(void);
void turn_zhijiao(void);


#endif
