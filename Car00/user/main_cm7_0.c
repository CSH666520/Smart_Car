/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Get_Angle.h"
#include "motor.h"
#include "Camera.h"
#include "control.h"
#include "headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***********************
// 核心板正常供电即可 无需额外连接


// *************************** 例程测试说明 **************************
// 1.核心板烧录完成本例程，完成上电
// 2.可以看到核心板上两个 LED 每秒周期交替闪烁
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

#define PIT_NUM                 (PIT_CH0 )                                     // 使用的周期中断编号
#define PIT1                             (PIT_CH1 )                             // 使用的周期中断编号

extern int16 l_line_x[LCDH], r_line_x[LCDH]; 
extern int16 l_line_x_l[LCDH], r_line_x_l[LCDH];


int i;

/**********元素处理结构体**********/
extern struct YUAN_SU road_type ; 

 int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                   // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    
 
    
    gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // GPIO 初始化为输出 默认上拉输出高
    gpio_init(DIR2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM1, 17000, 0);                                                   // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM2, 17000, 0);                                                   // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    encoder_dir_init(ENCODER_DIR1, ENCODER_DIR_PULSE1, ENCODER_DIR_DIR1);       // 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_dir_init(ENCODER_DIR2, ENCODER_DIR_PULSE2, ENCODER_DIR_DIR2);       // 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_dir_init(ENCODER_DIR3, ENCODER_DIR_PULSE3, ENCODER_DIR_DIR3);       // 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_dir_init(ENCODER_DIR4, ENCODER_DIR_PULSE4, ENCODER_DIR_DIR4);       // 初始化编码器模块与引脚 带方向增量编码器模式
    
    
    tft180_set_dir(TFT180_CROSSWISE);                                           // 需要先横屏 不然显示不下
    tft180_init();
    mt9v03x_init ();
       
                                   
    imu660ra_init();
    IMU_Offset_Init(IMU660RA);
    
    pit_ms_init(PIT_NUM, 10);      // 初始化 CCU6_0_CH0 为周期中断 10ms 周期 
    pit_ms_init(PIT1, 10);      // 初始化 CCU6_0_CH0 为周期中断 10ms 周期
    
    //PID初始化
    
    PID_init(&PID_Angle,-0.1,0,0);
    
    PID_init(&PID_Left,40,0.2,0.5);
    PID_init(&PID_right,40,0.2,0.5);
    
    
    
    

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

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
        // 此处编写需要循环执行的代码
    }
}


void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数     
{
   pit_isr_flag_clear(PIT_CH0);	  
   
   Error_Angle.time++;
   imu660ra_get_acc();                                                     // 获取 imu660ra 的加速度测量数值
   imu660ra_get_gyro();                                                    // 获取 imu660ra 的角速度测量数值
   Get_New_Angle();
   Madgwick_AHRS_6_DOF_Get_Angle(BETA, 0, IMU660RA, 10);      
   New_angle = IMU.angle.yaw_a - Error_Angle.k*(Error_Angle.time)- Error_Angle.add;  //线性回归	
   

}

void pit0_ch1_isr()                     // 定时器通道 1 周期中断服务函数      
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

// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：LED 不闪烁
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
