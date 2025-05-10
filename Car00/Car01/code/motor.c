#include "motor.h"
#include "zf_common_headfile.h"
int16 encoder_data_dir[4] = {0};

void Motor_SetSpeed(int PWM_1, int PWM_2)
{
        if(PWM_1 >= 0)                                                           // ��ת
        {
            gpio_set_level(DIR1, GPIO_HIGH);                                    // DIR����ߵ�ƽ
            pwm_set_duty(PWM1, PWM_1 * (PWM_DUTY_MAX / 100));                    // ����ռ�ձ�
        }
        else                                                                    // ��ת
        {
            gpio_set_level(DIR1, GPIO_LOW);                                     // DIR����͵�ƽ
            pwm_set_duty(PWM1, (-PWM_1) * (PWM_DUTY_MAX / 100));                 // ����ռ�ձ�
        }   
        
        if(PWM_2 >= 0)                                                           // ��ת
        {
            gpio_set_level(DIR2, GPIO_HIGH);                                    // DIR����ߵ�ƽ
            pwm_set_duty(PWM2, PWM_2 * (PWM_DUTY_MAX / 100));                    // ����ռ�ձ�
        }
        else                                                                    // ��ת
        {
            gpio_set_level(DIR2, GPIO_LOW);                                     // DIR����͵�ƽ
            pwm_set_duty(PWM2, (-PWM_2) * (PWM_DUTY_MAX / 100));                 // ����ռ�ձ�
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
