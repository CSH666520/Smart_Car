/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Get_Angle.h"
#include "motor.h"
#include "Camera.h"
#include "control.h"
#include "headfile.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// *************************** ����Ӳ������˵�� ***********************
// ���İ��������缴�� �����������


// *************************** ���̲���˵�� **************************
// 1.���İ���¼��ɱ����̣�����ϵ�
// 2.���Կ������İ������� LED ÿ�����ڽ�����˸
// �������������˵�����ز��� ����ձ��ļ����·� ���̳�������˵�� �����Ų�

// **************************** �������� ****************************

#define PIT_NUM                 (PIT_CH0 )                                     // ʹ�õ������жϱ��
#define PIT1                             (PIT_CH1 )                             // ʹ�õ������жϱ��

extern int16 l_line_x[LCDH], r_line_x[LCDH]; 
extern int16 l_line_x_l[LCDH], r_line_x_l[LCDH];


int i;

/**********Ԫ�ش���ṹ��**********/
extern struct YUAN_SU road_type ; 

 int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                   // ���Դ�����Ϣ��ʼ��
    // �˴���д�û����� ���������ʼ�������
    
 
    
    gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // GPIO ��ʼ��Ϊ��� Ĭ�����������
    gpio_init(DIR2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(PWM1, 17000, 0);                                                   // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    pwm_init(PWM2, 17000, 0);                                                   // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    
    encoder_dir_init(ENCODER_DIR1, ENCODER_DIR_PULSE1, ENCODER_DIR_DIR1);       // ��ʼ��������ģ�������� ����������������ģʽ
    encoder_dir_init(ENCODER_DIR2, ENCODER_DIR_PULSE2, ENCODER_DIR_DIR2);       // ��ʼ��������ģ�������� ����������������ģʽ
    encoder_dir_init(ENCODER_DIR3, ENCODER_DIR_PULSE3, ENCODER_DIR_DIR3);       // ��ʼ��������ģ�������� ����������������ģʽ
    encoder_dir_init(ENCODER_DIR4, ENCODER_DIR_PULSE4, ENCODER_DIR_DIR4);       // ��ʼ��������ģ�������� ����������������ģʽ
    
    
    tft180_set_dir(TFT180_CROSSWISE);                                           // ��Ҫ�Ⱥ��� ��Ȼ��ʾ����
    tft180_init();
    mt9v03x_init ();
       
                                   
    imu660ra_init();
    IMU_Offset_Init(IMU660RA);
    
    pit_ms_init(PIT_NUM, 10);      // ��ʼ�� CCU6_0_CH0 Ϊ�����ж� 10ms ���� 
    pit_ms_init(PIT1, 10);      // ��ʼ�� CCU6_0_CH0 Ϊ�����ж� 10ms ����
    
    //PID��ʼ��
    
    PID_init(&PID_Angle,-0.1,0,0);
    
    PID_init(&PID_Left,40,0.2,0.5);
    PID_init(&PID_right,40,0.2,0.5);
    
    
    
    

    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

        tft180_show_float(20, 16*5, New_angle, 4, 3);
        tft180_show_int(20, 16*6, Error_Angle.time , 5);
        
        
        
       if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            Get_Use_Image();
            Get_Bin_Image(0);
            Bin_Image_Filter();
            dou_Longest_White_Column();
            tft180_displayimage03x((const uint8 *)image_01, 94, 60);    
            Element_Judge(); 

        }

            for( i=0;i<58;i++)
            {  
              tft180_draw_point(r_line_x_l[i],i,RGB565_BLUE);
              tft180_draw_point(r_line_x_l[i-1],i,RGB565_BLUE);
               tft180_draw_point(l_line_x_l[i],i,RGB565_BLUE);
               tft180_draw_point(l_line_x_l[i+1],i,RGB565_BLUE);
               tft180_draw_point((l_line_x_l[i] + r_line_x_l[i])/2,i,RGB565_PINK); 
               
                if(i>=58)
                {
                   i=0;
                   break;
                 }
             }         
 
        tft180_show_uint(100, 32,  road_type.right_right_angle_bend, 6);
         tft180_show_int(100, 48,  longest_White_Column, 6);
         
         tft180_show_int(100, 64, encoder_data_dir[1], 6);
         tft180_show_int(100, 80,  encoder_data_dir[2], 6);
//        tft180_show_int(100, 16*5, left_junp_change_sign_num , 5);
//        tft180_show_int(100, 16*6, right_junp_change_sign_num , 5);
//        tft180_show_int(100, 16*2, (l_line_x_l[20] + r_line_x_l[20])/2 , 5);            
      //  tft180_show_uint(100, 48,  r_lose_value, 2);
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}


void pit0_ch0_isr()                     // ��ʱ��ͨ�� 0 �����жϷ�����     
{
   pit_isr_flag_clear(PIT_CH0);	  
   
   Error_Angle.time++;
   imu660ra_get_acc();                                                     // ��ȡ imu660ra �ļ��ٶȲ�����ֵ
   imu660ra_get_gyro();                                                    // ��ȡ imu660ra �Ľ��ٶȲ�����ֵ
   Get_New_Angle();
   Madgwick_AHRS_6_DOF_Get_Angle(BETA, 0, IMU660RA, 10);      
   New_angle = IMU.angle.yaw_a - Error_Angle.k*(Error_Angle.time)- Error_Angle.add;  //���Իع�	
   

}

void pit0_ch1_isr()                     // ��ʱ��ͨ�� 1 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH1);
    Get_encoder();        
   
   //PID_Control()
//    Position_Velocity_Controller(30 -  Make_Car_Follow_Line_PWM(),30 +  Make_Car_Follow_Line_PWM());
    turn_zhijiao(); 
  PWM_A = Speed_left -  Make_Car_Follow_Line_PWM() +  (int)(PID_Turn(turn_kp , turn_kd, (int)(New_angle - Now_turn), blance_turn ));
  PWM_B = Speed_right +  Make_Car_Follow_Line_PWM() -  (int)(PID_Turn(turn_kp , turn_kd, (int)(New_angle - Now_turn), blance_turn)); 
    
    
    
//     turn_kp = -8; //8
//     turn_kd = -0.04; //0.04

    Speed_Limit();
    Position_Velocity_Controller(PWM_A,PWM_B); 
}

// **************************** �������� ****************************

// *************************** ���̳�������˵�� ***************************
// ��������ʱ�밴�������������б���
// ����1��LED ����˸
//      �鿴�����Ƿ�������¼���Ƿ����ر���ȷ���������¸�λ����
//      ���ñ������Ӧ LED ���ŵ�ѹ�Ƿ�仯��������仯֤������δ���У�����仯֤�� LED ������
