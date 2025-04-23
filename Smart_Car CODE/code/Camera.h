#ifndef __CAMERA_H
#define __CAMERA_H

#include "zf_common_typedef.h"

#define IMAGEH  MT9V03X_H   /*!< ����ͷ�ɼ��߶� */
#define IMAGEW  MT9V03X_W   /*!< ����ͷ�ɼ���� */
#define LCDH    60  /*!< TFT��ʾ�߶ȣ��û�ʹ�ã��߶� */
#define LCDW    94  /*!< TFT��ʾ��ȣ��û�ʹ�ã���� */
#define MT9V034_IMAGEH  120  /*!< �� HEIGHT ���ɼ�����ͷͼ��߶����� */
#define MT9V034_IMAGEW  188  /*!< �� WIDTH  ���ɼ�����ͷͼ�������� */

extern uint8    mt9v03x_image[MT9V034_IMAGEH][MT9V034_IMAGEW];
extern unsigned char Image_Use[LCDH][LCDW];
extern unsigned char Bin_Image[LCDH][LCDW];
extern uint8 image_01[LCDH][LCDW];

short GetOSTU (unsigned char tmImage[LCDH][LCDW]);
void Get_Bin_Image (unsigned char mode);
void Bin_Image_Filter (void);
void Get_Use_Image(void);
void Camera_All_Deal(void);

#endif
