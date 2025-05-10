#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"

extern int16 encoder_data_dir[4];

void Motor_SetSpeed(int PWM_1, int PWM_2);
void Get_encoder(void);


#define DIR1                (P05_0)
#define PWM1                (TCPWM_CH10_P05_1)
#define DIR2                (P05_2)
#define PWM2                (TCPWM_CH12_P05_3)

#define ENCODER_DIR1                     (TC_CH58_ENCODER)                      // 带方向编码器对应使用的编码器接口       
#define ENCODER_DIR_PULSE1               (TC_CH58_ENCODER_CH1_P17_3)            // PULSE 对应的引脚                      
#define ENCODER_DIR_DIR1                 (TC_CH58_ENCODER_CH2_P17_4)            // DIR 对应的引脚                        
                                                                                
#define ENCODER_DIR2                     (TC_CH27_ENCODER)                      // 带方向编码器对应使用的编码器接口   
#define ENCODER_DIR_PULSE2               (TC_CH27_ENCODER_CH1_P19_2)            // PULSE 对应的引脚                  
#define ENCODER_DIR_DIR2                 (TC_CH27_ENCODER_CH2_P19_3)            // DIR 对应的引脚                    
                                                                                
#define ENCODER_DIR3                     (TC_CH07_ENCODER)                      // 带方向编码器对应使用的编码器接口  
#define ENCODER_DIR_PULSE3               (TC_CH07_ENCODER_CH1_P07_6)            // PULSE 对应的引脚                 
#define ENCODER_DIR_DIR3                 (TC_CH07_ENCODER_CH2_P07_7)            // DIR 对应的引脚                   
                                                                                
#define ENCODER_DIR4                     (TC_CH20_ENCODER)                      // 带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE4               (TC_CH20_ENCODER_CH1_P08_1)            // PULSE 对应的引脚
#define ENCODER_DIR_DIR4                 (TC_CH20_ENCODER_CH2_P08_2)            // DIR 对应的引脚


#endif
