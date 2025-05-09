#include "motor.h"
#include "zf_common_headfile.h"
int16 encoder_data_dir[4] = {0};

void Motor_SetSpeed(int PWM_1, int PWM_2)
{
        if(PWM_1 >= 0)                                                           // 正转
        {
            gpio_set_level(DIR1, GPIO_HIGH);                                    // DIR输出高电平
            pwm_set_duty(PWM1, PWM_1 * (PWM_DUTY_MAX / 100));                    // 计算占空比
        }
        else                                                                    // 反转
        {
            gpio_set_level(DIR1, GPIO_LOW);                                     // DIR输出低电平
            pwm_set_duty(PWM1, (-PWM_1) * (PWM_DUTY_MAX / 100));                 // 计算占空比
        }   
        
        if(PWM_2 >= 0)                                                           // 正转
        {
            gpio_set_level(DIR2, GPIO_HIGH);                                    // DIR输出高电平
            pwm_set_duty(PWM2, PWM_2 * (PWM_DUTY_MAX / 100));                    // 计算占空比
        }
        else                                                                    // 反转
        {
            gpio_set_level(DIR2, GPIO_LOW);                                     // DIR输出低电平
            pwm_set_duty(PWM2, (-PWM_2) * (PWM_DUTY_MAX / 100));                 // 计算占空比
        }  

}

void Get_encoder(void)
{
      encoder_data_dir[0] = encoder_get_count(ENCODER_DIR1);
      encoder_clear_count(ENCODER_DIR1);
      encoder_data_dir[1] = encoder_get_count(ENCODER_DIR2);
      encoder_clear_count(ENCODER_DIR2);
      encoder_data_dir[2] = encoder_get_count(ENCODER_DIR3);
      encoder_clear_count(ENCODER_DIR3);
      encoder_data_dir[3] = encoder_get_count(ENCODER_DIR4);
      encoder_clear_count(ENCODER_DIR4);
}
