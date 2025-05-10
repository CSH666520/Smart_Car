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
#include "Camera.h"
#include "zf_common_headfile.h"
#include "get_angle.h"
#include "motor.h"
#include "Camera.h"
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// *************************** ����Ӳ������˵�� ***************************
//      ģ��ܽ�            ��Ƭ���ܽ�
//      SCL                 �鿴 zf_device_ips114.h �� IPS114_SCL_PIN �궨��
//      SDA                 �鿴 zf_device_ips114.h �� IPS114_SDA_PIN �궨��
//      RES                 �鿴 zf_device_ips114.h �� IPS114_RST_PIN �궨��
//      DC                  �鿴 zf_device_ips114.h �� IPS114_DC_PIN  �궨��
//      CS                  �鿴 zf_device_ips114.h �� IPS114_CS_PIN  �궨��
//      BL                  �鿴 zf_device_ips114.h �� IPS114_BLK_PIN �궨��
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ



// *************************** ���̲���˵�� ***************************
// 1.���İ���¼������ ���������� 1.14��IPS ��ʾģ������������Ļ�ӿ������� ��ע�����Ŷ�Ӧ ��Ҫ���
// 2.��ع��� �ϵ�� 1.14��IPS ��Ļ���� ��ʾ�ַ����ָ������Ͳ���ͼ
// �������������˵�����ز��� ����ձ��ļ����·� ���̳�������˵�� �����Ų�


// **************************** �������� ****************************

#define PIT_NUM                          (PIT_CH0 )                             // ʹ�õ������жϱ��
#define PIT1                             (PIT_CH1 )                             // ʹ�õ������жϱ��

extern int16 top_junp_change_sign_num, bottom_junp_change_sign_num, left_junp_change_sign_num, right_junp_change_sign_num;
extern int16 l_line_x[LCDH], r_line_x[LCDH]; 
extern int16 l_line_x_l[LCDH], r_line_x_l[LCDH];
int i;
extern struct YUAN_SU road_type;

int main()
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                          // ���Դ�����Ϣ��ʼ��
    
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
    mt9v03x_init();
    
    imu660ra_init();
    IMU_Offset_Init(IMU660RA);

    pit_ms_init(PIT_NUM, 10);      // ��ʼ�� CCU6_0_CH0 Ϊ�����ж� 10ms ���� 
    pit_ms_init(PIT1, 10);      // ��ʼ�� CCU6_0_CH0 Ϊ�����ж� 10ms ����
    
    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

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
//         tft180_show_uint(100, 80,  longest_White_Column_site, 6);
         tft180_show_uint(100, 16,  top_junp_change_sign_num, 2);
         tft180_show_uint(100, 32,  bottom_junp_change_sign_num, 2);
         tft180_show_uint(100, 48,  left_junp_change_sign_num, 2);
         tft180_show_uint(100, 64,  right_junp_change_sign_num, 2);
         tft180_show_uint(130, 16,  road_type.straight, 1);
         tft180_show_uint(130, 32,  road_type.ten, 1);
         tft180_show_uint(130, 48,  road_type.left_right_angle_bend, 1);
         tft180_show_uint(130, 64,  road_type.right_right_angle_bend, 1);
         tft180_show_uint(130, 80,  road_type.ben_ring, 1);
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
//    Get_encoder();
//    PID_Control();
    
}

// **************************** �������� ****************************
// *************************** ���̳�������˵�� ***************************
// ��������ʱ�밴�������������б���
// ����1����Ļ����ʾ
//      ���ʹ��������ԣ��������Ҫ�õ�ع��� �����Ļ�������ŵ�ѹ
//      �����Ļ�ǲ��ǲ��λ���� ������Ŷ�Ӧ��ϵ
//      �����Ӧ���Ŷ���ȷ ���һ���Ƿ������Ų��β��� ��Ҫ��ʾ����
//      �޷���ɲ��β�������һ��GPIO���̽���Ļ����IO��ʼ��ΪGPIO��ת��ƽ �����Ƿ��ܿ�