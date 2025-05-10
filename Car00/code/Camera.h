#ifndef __CAMERA_H
#define __CAMERA_H

#include "zf_common_typedef.h"

#define IMAGEH  MT9V03X_H   /*!< 摄像头采集高度 */
#define IMAGEW  MT9V03X_W   /*!< 摄像头采集宽度 */
#define LCDH    60  /*!< TFT显示高度（用户使用）高度 */
#define LCDW    94  /*!< TFT显示宽度（用户使用）宽度 */
#define MT9V034_IMAGEH  120  /*!< 行 HEIGHT 待采集摄像头图像高度行数 */
#define MT9V034_IMAGEW  188  /*!< 列 WIDTH  待采集摄像头图像宽度列数 */

//赛道类型结构体
struct YUAN_SU{
     int16 barrier;                            //横断
     int16 straight;                           //直道
     int16 right_right_angle_bend;             //右直角弯道
     int16 ten;                                //十字 
     int16 left_right_angle_bend;             //左直角弯道

};



extern uint8    mt9v03x_image[MT9V034_IMAGEH][MT9V034_IMAGEW];
extern unsigned char Image_Use[LCDH][LCDW];
extern unsigned char Bin_Image[LCDH][LCDW];
extern uint8 image_01[LCDH][LCDW];

extern int16 Longest_White_Column_Left_site,Longest_White_Column_Right_site,longest_White_Column_site;//搜线变量
extern int Longest_White_Column_Left,Longest_White_Column_Right, longest_White_Column;
extern uint8 l_lose_value, r_lose_value ;                                   //左右丢线数
extern uint8 l_search_flag[LCDH], r_search_flag[LCDH];                   //是否搜到线的标志
extern int Boundry_Start_Right,Boundry_Start_Left;
extern int Both_Lost_Time;
extern int16 Right_Lost_Flag[LCDH],Left_Lost_Flag[LCDH];
extern int16 l_line_x[LCDH], r_line_x[LCDH], m_line_x[LCDH];        //储存原始图像的左右边界的列数，用于元素判断
extern int16 l_line_x_l[LCDH], r_line_x_l[LCDH], m_line_x_l[LCDH];  //储存原始图像的左右边界的列数，用于偏差计算
//extern int search_line_end = 5;//不能为零，原因暂未查

extern int16 top_junp_change_sign_num, bottom_junp_change_sign_num, left_junp_change_sign_num, right_junp_change_sign_num;//用于记录上下左右黑白跳变数量

short GetOSTU (unsigned char tmImage[LCDH][LCDW]);
void Get_Bin_Image (unsigned char mode);
void Bin_Image_Filter (void);
void Get_Use_Image(void);
void Camera_All_Deal(void);
void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], int Row, int Col, int Bottonline);
void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline);
void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline);
void dou_Longest_White_Column(void);//最长白列巡线
void  Element_Judge(void);


#endif
