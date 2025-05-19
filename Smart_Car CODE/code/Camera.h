#ifndef __CAMERA_H
#define __CAMERA_H

#include "zf_common_typedef.h"

#define IMAGEH  MT9V03X_H   /*!< 摄像头采集高度 */
#define IMAGEW  MT9V03X_W   /*!< 摄像头采集宽度 */
#define LCDH    60  /*!< TFT显示高度（用户使用）高度 */
#define LCDW    94  /*!< TFT显示宽度（用户使用）宽度 */
#define MT9V034_IMAGEH  120  /*!< 行 HEIGHT 待采集摄像头图像高度行数 */
#define MT9V034_IMAGEW  188  /*!< 列 WIDTH  待采集摄像头图像宽度列数 */
#define Image_X 94

//赛道类型结构体
struct YUAN_SU{
     int16 barrier;                            //横断
     int16 straight;                           //直道
     int16 right_right_angle_bend;             //右直角弯道
     int16 ten;                                //十字 
     int16 left_right_angle_bend;             //左直角弯道
     int16 ben_ring;                           //苯环
     
};

/**八向迷宫法变量**/
extern int Thres_Interfere;
extern int Thres_Num_Interfere;//25个阈值点
extern uint8 Start_Flag;
extern uint8 l_x_start, l_y_start, r_x_start, r_y_start;        //八向迷宫法起始点坐标
extern uint16_t L_Thres_Record[500];     //储存阈值数组
extern uint16_t R_Thres_Record[500];
extern uint8 Adaptive_L_Start_Point[2];          //存储左侧起始点的数组（全局变量）
extern uint8 Adaptive_R_Start_Point[2];          //存储右侧起始点的数组（全局变量）
extern uint8 L_Border[200];     //存储左侧一维边线的数组
extern uint8 R_Border[200];     //存储右侧一维边线的数组
extern uint16 Adaptive_L_Line[600][2];             //存放左侧边线的二维数组
extern uint16 Adaptive_R_Line[600][2];             //存放右侧边线的二维数组
extern uint16_t Adaptive_L_Statics;                //记录左边边线点的个数
extern uint16_t Adaptive_R_Statics;                //记录右边边线点的个数
extern int16_t Adaptive_L_Grow_Dir[500];                //存放左侧边线每个点的生长方向
extern int16_t Adaptive_R_Grow_Dir[500];                //存放右侧边线每个点的生长方向
extern uint8 Adaptive_X_Meet;              //记录左右两侧爬线相遇点的X坐标
extern uint8 Adaptive_X_Meet;              //记录左右两侧爬线相遇点的Y坐标
extern uint8 Adaptive_L_Thres_Max;     //左侧阈值最大值
extern uint8 Adaptive_R_Thres_Max;     //右侧阈值最大值
extern uint8 Adaptive_L_Thres_Min;     //左侧阈值最小值
extern uint8 Adaptive_R_Thres_Min;     //右侧阈值最小值
extern uint8 Adaptive_Thres_Average;   //阈值均值
extern uint8 Last_Adaptive_Thres_Average;  //用于阈值均值滤波

extern uint8    mt9v03x_image[MT9V034_IMAGEH][MT9V034_IMAGEW];
extern unsigned char Image_Use[LCDH][LCDW];
extern unsigned char Bin_Image[LCDH][LCDW];
extern uint8 image_01[LCDH][LCDW];

//extern uint8 Image_X[LCDH][LCDW];

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
int My_ABS(int num);

#endif
