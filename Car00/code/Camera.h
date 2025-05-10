#ifndef __CAMERA_H
#define __CAMERA_H

#include "zf_common_typedef.h"

#define IMAGEH  MT9V03X_H   /*!< ����ͷ�ɼ��߶� */
#define IMAGEW  MT9V03X_W   /*!< ����ͷ�ɼ���� */
#define LCDH    60  /*!< TFT��ʾ�߶ȣ��û�ʹ�ã��߶� */
#define LCDW    94  /*!< TFT��ʾ��ȣ��û�ʹ�ã���� */
#define MT9V034_IMAGEH  120  /*!< �� HEIGHT ���ɼ�����ͷͼ��߶����� */
#define MT9V034_IMAGEW  188  /*!< �� WIDTH  ���ɼ�����ͷͼ�������� */

//�������ͽṹ��
struct YUAN_SU{
     int16 barrier;                            //���
     int16 straight;                           //ֱ��
     int16 right_right_angle_bend;             //��ֱ�����
     int16 ten;                                //ʮ�� 
     int16 left_right_angle_bend;             //��ֱ�����

};



extern uint8    mt9v03x_image[MT9V034_IMAGEH][MT9V034_IMAGEW];
extern unsigned char Image_Use[LCDH][LCDW];
extern unsigned char Bin_Image[LCDH][LCDW];
extern uint8 image_01[LCDH][LCDW];

extern int16 Longest_White_Column_Left_site,Longest_White_Column_Right_site,longest_White_Column_site;//���߱���
extern int Longest_White_Column_Left,Longest_White_Column_Right, longest_White_Column;
extern uint8 l_lose_value, r_lose_value ;                                   //���Ҷ�����
extern uint8 l_search_flag[LCDH], r_search_flag[LCDH];                   //�Ƿ��ѵ��ߵı�־
extern int Boundry_Start_Right,Boundry_Start_Left;
extern int Both_Lost_Time;
extern int16 Right_Lost_Flag[LCDH],Left_Lost_Flag[LCDH];
extern int16 l_line_x[LCDH], r_line_x[LCDH], m_line_x[LCDH];        //����ԭʼͼ������ұ߽������������Ԫ���ж�
extern int16 l_line_x_l[LCDH], r_line_x_l[LCDH], m_line_x_l[LCDH];  //����ԭʼͼ������ұ߽������������ƫ�����
//extern int search_line_end = 5;//����Ϊ�㣬ԭ����δ��

extern int16 top_junp_change_sign_num, bottom_junp_change_sign_num, left_junp_change_sign_num, right_junp_change_sign_num;//���ڼ�¼�������Һڰ���������

short GetOSTU (unsigned char tmImage[LCDH][LCDW]);
void Get_Bin_Image (unsigned char mode);
void Bin_Image_Filter (void);
void Get_Use_Image(void);
void Camera_All_Deal(void);
void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], int Row, int Col, int Bottonline);
void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline);
void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline);
void dou_Longest_White_Column(void);//�����Ѳ��
void  Element_Judge(void);


#endif
