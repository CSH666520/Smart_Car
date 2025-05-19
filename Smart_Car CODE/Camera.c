#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "Camera.h"


#define LCDH    60  /*!< TFT��ʾ�߶ȣ��û�ʹ�ã��߶� */
#define LCDW    94  /*!< TFT��ʾ��ȣ��û�ʹ�ã���� */
#define MT9V034_IMAGEH  120  /*!< �� HEIGHT ���ɼ�����ͷͼ��߶����� */
#define MT9V034_IMAGEW  188  /*!< �� WIDTH  ���ɼ�����ͷͼ�������� */

/** ͼ��ԭʼ���ݴ�� */
extern uint8    mt9v03x_image[MT9V034_IMAGEH][MT9V034_IMAGEW];
/** ѹ����֮�����ڴ����Ļ��ʾ����  */
unsigned char Image_Use[LCDH][LCDW];

/** ��ֵ����������Ļ��ʾ������ */
unsigned char Bin_Image[LCDH][LCDW];

//int16 top_junp_change_sign_num= 0, bottom_junp_change_sign_num = 0, left_junp_change_sign_num = 0, right_junp_change_sign_num = 0;//���ڼ�¼�������Һڰ���������


uint8 image_01[60][94];           //��ֵ��ͼ��

/**�����Թ�������**/
uint8 Black_Box_Value_FFF = 50;
uint8 Black_Box_Value_FF = 50;
uint8 Black_Box_Value_F = 50;
uint8 Black_Box_Value; 
uint8 Compare_Value = 20;
int Thres_Interfere = 0;
int Thres_Num_Interfere = 25;//25����ֵ��
uint8 Start_Flag;
uint8 l_x_start, l_y_start, r_x_start, r_y_start;        //�����Թ�����ʼ������
uint16_t L_Thres_Record[500];     //������ֵ����
uint16_t R_Thres_Record[500];
uint8 Adaptive_L_Start_Point[2];          //�洢�����ʼ������飨ȫ�ֱ�����
uint8 Adaptive_R_Start_Point[2];          //�洢�Ҳ���ʼ������飨ȫ�ֱ�����
uint8_t record_gray_value[2];             //��¼�������Ҷ�ֵ
uint8_t Black_Box_Value_1;
uint8 L_Border[200];     //�洢���һά���ߵ�����
uint8 R_Border[200];     //�洢�Ҳ�һά���ߵ�����
uint16 Adaptive_L_Line[600][2];             //��������ߵĶ�ά����
uint16 Adaptive_R_Line[600][2];             //����Ҳ���ߵĶ�ά����
uint16_t Adaptive_L_Statics;                //��¼��߱��ߵ�ĸ���
uint16_t Adaptive_R_Statics;                //��¼�ұ߱��ߵ�ĸ���
int16_t Adaptive_L_Grow_Dir[500];                //���������ÿ�������������
int16_t Adaptive_R_Grow_Dir[500];                //����Ҳ����ÿ�������������
uint8 Adaptive_X_Meet;              //��¼�������������������X����
uint8 Adaptive_Y_Meet;              //��¼�������������������Y����
uint8 Adaptive_L_Thres_Max = 0;     //�����ֵ���ֵ
uint8 Adaptive_R_Thres_Max = 0;     //�Ҳ���ֵ���ֵ
uint8 Adaptive_L_Thres_Min = 0;     //�����ֵ��Сֵ
uint8 Adaptive_R_Thres_Min = 0;     //�Ҳ���ֵ��Сֵ
uint8 Adaptive_Thres_Average = 0;   //��ֵ��ֵ
uint8 Last_Adaptive_Thres_Average = 0;  //������ֵ��ֵ�˲�
uint16_t Image_Num;  //ͼ�����

int16 Longest_White_Column_Left_site,Longest_White_Column_Right_site, longest_White_Column_site;//���߱���
int Longest_White_Column_Left,Longest_White_Column_Right;
uint8 l_lose_value = 0, r_lose_value = 0;                                   //���Ҷ�����
uint8 l_search_flag[LCDH], r_search_flag[LCDH];                   //�Ƿ��ѵ��ߵı�־
int Boundry_Start_Right,Boundry_Start_Left;
int longest_White_Column=0,Both_Lost_Time;
int16 Right_Lost_Flag[LCDH],Left_Lost_Flag[LCDH];
int16 l_line_x[LCDH], r_line_x[LCDH], m_line_x[LCDH];        //����ԭʼͼ������ұ߽������������Ԫ���ж�
int16 l_line_x_l[LCDH], r_line_x_l[LCDH], m_line_x_l[LCDH];  //����ԭʼͼ������ұ߽������������ƫ�����
int search_line_end = 5;//����Ϊ�㣬ԭ����δ��

/******************����Ӧ�����Թ�����************************/
 
const int8 L_Face_Dir[4][2] = {{0,-1},{1,0},{0,1},{-1,0}};  //����Թ�����
//  0
//3   1
//  2
 
const int8 L_Face_Dir_L[4][2] = {{-1,-1},{1,-1},{1,1},{-1,1}};  //����������ǰ��
//0   1
//
//3   2
 
const int8 R_Face_Dir[4][2] = {{0,-1},{1,0},{0,1},{-1,0}};  //�Ҳ��Թ�����
//  0
//3   1
//  2
 
const int8 R_Face_Dir_R[4][2] = {{1,-1},{1,1},{-1,1},{-1,-1}};  //�Ҳ��������ǰ��
//3   0
//
//2   1
 
const int8 Square_0[25][2] = {              //һ��5 * 5�ľ������������ĵ���Χ�ľֲ���ֵ���ֲ���ֵ��
{-2,-2},{-1,-2},{0,-2},{+1,-2},{+2,-2},
{-2,-1},{-1,-1},{0,-1},{+1,-1},{+2,-1},
{-2,-0},{-1, 0},{0, 0},{+1, 0},{+2,-0},
{-2,+1},{-1,+1},{0,+1},{+1,+1},{+2,+1},
{-2,+2},{-1,+2},{0,+2},{+1,+2},{+2,+2}
};
 
//�Թ�����ֹͣ���߱�־λ
uint8 L_Stop_Flag;
uint8 R_Stop_Flag;

/**********Ԫ�ش���ṹ��**********/
struct YUAN_SU road_type = {     
      .barrier                       = 0,         //���
      .straight                      = 0,         //ֱ��
      .right_right_angle_bend        = 0,         //��ֱ�����
      .ten                           = 0,         //ʮ��
      .left_right_angle_bend         = 0,         //��ֱ�����
      .ben_ring                      = 0,         //����
      
};

/******************************����ֵ����****************************/
int My_ABS(int num)      //�����Թ�������
{
    if (num < 0) 
    {
        return -num;
    } 
    else 
    {
        return num;
    } 
}

/*************************************************************************
 *  �������ƣ�short GetOSTU (unsigned char tmImage[LCDH][LCDW])
 *  ����˵�����������ֵ��С
 *  ����˵����tmImage �� ͼ������
 *  �������أ���
 *  �޸�ʱ�䣺2011��10��28��
 *  ��    ע��  GetOSTU(Image_Use);//�����ֵ
Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ���� 1
4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��
        �ı���w0��        ��ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����
        ������)  * ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
6) i++��ת��4)��ֱ��iΪ256ʱ��������
7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
*************************************************************************/
short GetOSTU (unsigned char tmImage[LCDH][LCDW])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < LCDH; j++)
    {
        for (i = 0; i < LCDW; i++)
        {
            HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  ��������

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���
        PixelFore = Amount - PixelBack;           //�������ص���
        OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�
        OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
        PixelshortegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
        MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //���������ֵ;
}


/*************************************************************************
 *  �������ƣ�void Get_Bin_Image (unsigned char mode)
 *  ����˵����ͼ���ֵ����Bin_Image[][]
 *  ����˵����mode  ��
 *    0��ʹ�ô����ֵ
 *    1��ʹ��ƽ����ֵ
 *    2: sobel ���ӸĽ���  �ֶ���ֵ��ͬʱ�����Ϊ��ȡ���ص�ͼ��
 *    3��sobel ���ӸĽ���   ��̬��ֵ��ͬʱ�����Ϊ��ȡ���ص�ͼ��
 *  �������أ���
 *  �޸�ʱ�䣺2020��10��28��
 *  ��    ע��  Get_Bin_Image(0); //ʹ�ô�򷨶�ֵ��
 *************************************************************************/
void Get_Bin_Image (unsigned char mode)
{
    unsigned short i = 0, j = 0;
    unsigned short Threshold = 0;
    //char txt[16];

    if (mode == 0)
    {
        Threshold = GetOSTU(Image_Use);  //�����ֵ
    }
    
    
    
    /* ��ֵ�� */
    for (i = 0; i < LCDH; i++)
    {
        for (j = 0; j < LCDW; j++)
        {
            if (Image_Use[i][j] > Threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
            {
                Bin_Image[i][j] = 1;
                image_01[i][j] = 255;  //��
            }
            else
            {
                Bin_Image[i][j] = 0;
                image_01[i][j] = 0;  //��
            }
        }
    }
}

/*---------------------------------------------------------------
 ����    ����Bin_Image_Filter
 ����    �ܡ��������
 ����    ������
 ���� �� ֵ����
 ��ע�����
 ----------------------------------------------------------------*/
void Bin_Image_Filter (void)
{
    int nr; //��
    int nc; //��

    for (nr = 1; nr < LCDH - 1; nr++)
    {
        for (nc = 1; nc < LCDW - 1; nc = nc + 1)
        {
            if ((Bin_Image[nr][nc] == 0)
                    && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 2))
            {
                Bin_Image[nr][nc] = 1;
            }
            else if ((Bin_Image[nr][nc] == 1)
                    && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] < 2))
            {
                Bin_Image[nr][nc] = 0;
            }
        }
    }
}


/*************************************************************************
 *  �������ƣ�void Get_Use_Image (void)
 *  ����˵����������ͷ�ɼ���ԭʼͼ�����ŵ�����ʶ�������С
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��10��28��
 *  ��    ע��  IMAGEWΪԭʼͼ��Ŀ�ȣ�����Ϊ188��OV7725Ϊ320
 *       IMAGEHΪԭʼͼ��ĸ߶ȣ�����Ϊ120��OV7725Ϊ240
 *************************************************************************/
void Get_Use_Image(void)
{
    short i = 0, j = 0, row = 0, line = 0;

    for (i = 0; i < MT9V034_IMAGEH; i += 2)          //���۸� 120 / 2  = 60��
    {
        for (j = 0; j <= MT9V034_IMAGEW; j += 2)     //���ۿ�188 / 2  = 94��
        {
            Image_Use[row][line] = mt9v03x_image[i][j];
            line++;
        }
        line = 0;
        row++;
    }
}

/************************************************************************
�������ҵײ�����
*************************************************************************/
//void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
//{
//
//    //Ѱ����߽߱�
//    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
//    {
//        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
//        {
//            ImageDeal[Bottonline].LeftBoundary = Xsite;//��ȡ�ױ������
//            break;
//        }
//    }
//    for (int Xsite = Col / 2+2; Xsite < LCDW-1; Xsite++)
//    {
//        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
//        {
//            ImageDeal[Bottonline].RightBoundary = Xsite;//��ȡ�ױ��ұ���
//            break;
//        }
//    }
//
//}
/************************************************************************
�������ȡ���ұ���
*************************************************************************/
//void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], int Row, int Col, int Bottonline)
//{
//    //����С�˵ĵ�ǰ����״̬λ��Ϊ �� �� �� �� һ��Ҫ�� �ϣ����Ϊ��ɫ ���ϱ�Ϊ��ɫ �£��ұ�Ϊɫ  �ң������к�ɫ
///*  ǰ�������壺
//                *   0
//                * 3   1
//                *   2
//*/
///*Ѱ�����������*/
//    int Left_Rule[2][8] = {
//                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
//                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
//    };
//    /*Ѱ�����������*/
//    int Right_Rule[2][8] = {
//                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
//                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
//    };
//      int num=0;
//    uint8 Left_Ysite = Bottonline;
//    uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
//    uint8 Left_Rirection = 0;//��߷���
//    uint8 Pixel_Left_Ysite = Bottonline;
//    uint8 Pixel_Left_Xsite = 0;
//
//    uint8 Right_Ysite = Bottonline;
//    uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
//    uint8 Right_Rirection = 0;//�ұ߷���
//    uint8 Pixel_Right_Ysite = Bottonline;
//    uint8 Pixel_Right_Xsite = 0;
//    uint8 Ysite = Bottonline;
//    ImageStatus.OFFLineBoundary = 5;
//    while (1)
//    {
//            num++;
//            if(num>400)
//            {
//                 ImageStatus.OFFLineBoundary = Ysite;
//                break;
//            }
//        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
//        {
//            if (Ysite < ImageStatus.OFFLineBoundary)
//            {
//                ImageStatus.OFFLineBoundary = Ysite;
//                break;
//            }
//            else
//            {
//                Ysite--;
//            }
//        }
//        /*********���Ѳ��*******/
//        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//�ұ�ɨ��
//        {
//            /*����ǰ������*/
//            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
//            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
//
//            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 255)//ǰ���Ǻ�ɫ
//            {
//                //˳ʱ����ת90
//                if (Left_Rirection == 3)
//                    Left_Rirection = 0;
//                else
//                    Left_Rirection++;
//              
//            }
//            else//ǰ���ǰ�ɫ
//            {
//                /*������ǰ������*/
//                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
//                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
//
//                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 255)//��ǰ��Ϊ��ɫ
//                {
//                    //���򲻱�  Left_Rirection
//                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
//                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
//                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0){
//                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
//                        ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
//                    }
//                }
//                else//��ǰ��Ϊ��ɫ
//                {
//                    // �������ı� Left_Rirection  ��ʱ��90��
//                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
//                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
//                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0 )
//                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
//                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
//                    if (Left_Rirection == 0)
//                        Left_Rirection = 3;
//                    else
//                        Left_Rirection--;
//                }
////               for( i=0;i<58;i++)
////            {
////                if(i>=58)
////                {
////                   i=0;
////                 }
////                   
////                   tft180_draw_point(47,i,RGB565_RED);
////             }
////                Left_Xsite
//            }
//        }
//        /*********�ұ�Ѳ��*******/
//        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//�ұ�ɨ��
//        {
//            /*����ǰ������*/
//            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
//            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
//
//            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 255)//ǰ���Ǻ�ɫ
//            {
//                //��ʱ����ת90
//                if (Right_Rirection == 0)
//                    Right_Rirection = 3;
//                else
//                    Right_Rirection--;
//            }
//            else//ǰ���ǰ�ɫ
//            {
//                /*������ǰ������*/
//                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
//                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
//
//                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 255)//��ǰ��Ϊ��ɫ
//                {
//                    //���򲻱�  Right_Rirection
//                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
//                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
//                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79 )
//                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
//                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
//                }
//                else//��ǰ��Ϊ��ɫ
//                {
//                    // �������ı� Right_Rirection  ��ʱ��90��
//                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
//                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
//                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
//                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
//                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
//                    if (Right_Rirection == 3)
//                        Right_Rirection = 0;
//                    else
//                        Right_Rirection++;
//                }
//
//            }
//        }
//
//        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80��Ϊ�˷��ڵײ��ǰ�����ɨ�����  3 && Ysite < 30
//        {
//            ImageStatus.OFFLineBoundary = Ysite;
//            break;
//        }
//
//    }
//}
//
///************************************************************************
//������
//*************************************************************************/
//void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
//{
//    ImageStatus.WhiteLine_L = 0;
//    ImageStatus.WhiteLine_R = 0;
//    //ImageStatus.OFFLine = 1;
//    /*�����±߽紦��*/
//    for (int Xsite = 0; Xsite < LCDW; Xsite++)
//    {
//        imageInput[0][Xsite] = 0;
//        imageInput[Bottonline + 1][Xsite] = 0;
//    }
//    /*�����ұ߽紦��*/
//    for (int Ysite = 0; Ysite < LCDH; Ysite++)
//    {
//            ImageDeal[Ysite].LeftBoundary_First = 0;
//            ImageDeal[Ysite].RightBoundary_First = 79;
//
//            imageInput[Ysite][0] = 0;
//            imageInput[Ysite][LCDW - 1] = 0;
//    }
//    /********��ȡ�ײ�����*********/
//    Search_Bottom_Line_OTSU(image_01, Row, Col, Bottonline);
//    /********��ȡ���ұ���*********/
//    Search_Left_and_Right_Lines(image_01, Row, Col, Bottonline);
//
//
//    for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
//    {
//        if (ImageDeal[Ysite].LeftBoundary < 3)
//        {
//            ImageStatus.WhiteLine_L++;
//            
//        }
//        if (ImageDeal[Ysite].RightBoundary > LCDW - 3)
//        {
//            ImageStatus.WhiteLine_R++;
//        }
//        // tft180_draw_point(ImageStatus.WhiteLine_L,Ysite,RGB565_RED); 
//    }
//}

/************************************************************************
�����Ѳ��
*************************************************************************/
volatile int White_Column[LCDW];
void dou_Longest_White_Column(void)//�����Ѳ��
{
    int16 i, j;
    int16 start_column=0;//����е���������
    int16 end_column=LCDW;
    //int16 left_border = 0, right_border = 0;//��ʱ�洢����λ��
    Longest_White_Column_Left = 0;//�����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Left_site = 0;//�����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Right = 0;//�����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Right_site = 0;//�����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    r_lose_value = 0;    //�߽綪����
    l_lose_value  = 0;
    Boundry_Start_Left  = 0;//��һ���Ƕ��ߵ�,����߽���ʼ��
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//����ͬʱ������
    longest_White_Column=0;
    longest_White_Column_site = 0;

    for (i = 0; i <=LCDH-1; i++)//��������
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        l_line_x[i] = 0;
        r_line_x[i] = LCDW-1;//
        l_line_x_l[i]=0;
        r_line_x_l[i]=LCDW-1;
    }
    
    
    
     //��������
    for(i=LCDW-1;i>=0;i--)
    {
        White_Column[i] = 0;
    }
    for(i = start_column;i<=LCDW-1;i++)//image_01[MT9V03X_H-1][start_column]==0x00
    {
        if(image_01[LCDH-1][i] != 0x00)
        {
            start_column++;
        }

        else
        {
            break;
        }
    }
    for(i = start_column;i<=end_column-1;i++)
    {
        for(j=LCDH-1;j>=0;j--)
        {
            if(image_01[j][i] == 0x00&&image_01[j-1][i] == 0x00)
            {
                break;
            }
            else
            {
                White_Column[i]++;
            }
        }
        if(White_Column[i]>=longest_White_Column)
        {
            longest_White_Column=White_Column[i];
            longest_White_Column_site=i;
        }
    }
    
    
    search_line_end = longest_White_Column;//������ֹ��ѡȡ����������𲻴�����������������һ����
    if(search_line_end>=55){search_line_end=55;longest_White_Column=55;}
    for (i = LCDH - 1; i >=LCDH-search_line_end; i--)//����Ѳ��
    {
        for (j = longest_White_Column_site; j <= LCDW - 1 - 2; j++)
        {
            if (image_01[i][j] != 0x00 && image_01[i][j + 1] == 0x00 && image_01[i][j + 2] == 0x00)//�׺ںڣ��ҵ��ұ߽�
            {
                r_line_x[i] = j;
                r_line_x_l[i] = j;
                Right_Lost_Flag[i] = 0; //�Ҷ������飬������1����������0
                break;
            }
            else if(j>=LCDW-1-3)//û�ҵ��ұ߽磬����Ļ���Ҹ�ֵ���ұ߽�
            {
                r_line_x[i] = j;
                r_line_x_l[i] = j;
                r_lose_value++;
                Right_Lost_Flag[i] = 1;//�Ҷ������飬������1����������0
                break;
            }
        }
        for (j = longest_White_Column_site; j >= 0 + 2; j--)//�����ɨ��
        {
            if (image_01[i][j] != 0x00 && image_01[i][j - 1] == 0x00 && image_01[i][j - 2] == 0x00)//�׺ں���Ϊ������߽�
            {
                l_line_x [i] = j;
                l_line_x_l[i] = j;
                Left_Lost_Flag[i]=0;//l_lose_value = 0; //�������飬������1����������0
                break;
            }
            else if(j<=0+3)
            {
                l_line_x [i] = j;//�ҵ�ͷ��û�ҵ��ߣ��Ͱ���Ļ�����ҵ����߽�
                l_line_x_l[i] = j;
                l_lose_value++;
                Left_Lost_Flag[i]=1;//�������飬������1����������0
//                if(Right_Lost_Flag[i] == 1)Both_Lost_Time++;
                break;
            }
        }

    }

//
//    for (i = LCDH - 1; i >= 0; i--)//�������ݳ�������
//    {
////        if (Left_Lost_Flag[i]  == 1)//���߶�����
////            l_lose_value++;
////        if (Right_Lost_Flag[i] == 1)
////            r_lose_value++;
////        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//˫�߶�����
////            Both_Lost_Time++;
//        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)//��¼��һ���Ƕ��ߵ㣬�߽���ʼ��
//            Boundry_Start_Left = i;
//        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
//            Boundry_Start_Right = i;
//
//     }  //Road_Wide[i]=r_line_x[i]-l_line_x[i];
}

//���ڿ򣨱���Ϊһ�����ؿ�ȣ��߽���ؿճ�һ��
/**
* �������ܣ�      ͼ�񲹺ڿ�
* ����˵����      ע��ڿ���߽���һ�����ؿ��
* ��  �Σ�        uint8 black_box_value            �ڿ�ĻҶ�ֵ
*                uint8(*image)[Image_X]            Ҫ���ڿ��ͼ��
*
* ʾ����          Draw_Black_Box(Black_Box_Value, Find_Line_Image);��
* ����ֵ��        ��
*/
void Draw_Black_Box(uint8 black_box_value, uint8(*image)[Image_X])          
{
    uint8 i,j;
 
    Black_Box_Value_FFF = Black_Box_Value_FF;
    Black_Box_Value_FF = Black_Box_Value_F;
    Black_Box_Value_F = black_box_value;
    black_box_value = (uint8_t)(0.5 * Black_Box_Value_F + 0.3 * Black_Box_Value_FF + 0.2 * Black_Box_Value_FFF);        //�˲�
    Black_Box_Value = black_box_value;
    for(i = 1; i < 60; i++)
    {
        image[i][LCDW - 2] = black_box_value;
        image[i][1] = black_box_value;
    }
    for(j = 1; j < LCDW - 2; j++)
    {
        image[1][j] = black_box_value;
    }
}

/***********************************************************************************
* �������ܣ�      ��Ⱥ�
* ����˵����      ���������㷨�����
* ��  �Σ�        int16 a                  ��ֵ�ϴ�ĻҶ�ֵ
*                int16 b                   ��ֵ��С�ĻҶ�ֵ
*                uint8 compare_value       ��Ⱥ���ֵ
*
* ʾ����          Compare_Num(image[start_row][i + 5], image[start_row][i], Compare_Value)��
* ����ֵ��        ������ֵ����1�����򷵻�0.
*/
int16 Compare_Num(int16 a, int16 b, uint8 compare_value)               //****
{
    if((((a - b) << 7) / (a + b)) > compare_value)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*******************************************************************************************
* �������ܣ�      �����㷨�����
* ����˵����      ��
* ��  �Σ�        uint8 start_row              ������ͼ����Y����
*                uint8(*image)[Image_X]        Ҫ�����ͼ��
*                uint8 *l_start_point          �洢�����ʼ������飨ȫ�ֱ�����
*                uint8 *r_start_point          �洢�Ҳ���ʼ������飨ȫ�ֱ�����
*                uint8 l_border_x              ���������Ľ�ֹ�㣬��Զ�ҵ������ֹͣ
*                uint8 r_border_x              ���������Ľ�ֹ�㣬��Զ�ҵ������ֹͣ
*
* ʾ����          Get_Start_Point(Image_Y - 3, Find_Line_Image, Adaptive_L_Start_Point, Adaptive_R_Start_Point, 1, 78)
* ����ֵ��        ���߶��ҵ�����1�����򷵻�0.
*/
uint8 Get_Start_Point(uint8 start_row, uint8(*image)[Image_X], uint8 *l_start_point, uint8 *r_start_point, uint8 r_border_x)          //*****
{
    uint8 i = 0, j = 0;
    uint8 L_Is_Found = 0, R_Is_Found = 0;   //�ҵ����ʱ�ҳ���Ӧ��־λ
//    uint8 Start_X  = 0;                     //��ʼX���꣬��һ��ͼ��ȡͼ������е㣬����ͼ������һ��ͼ������������ʼ����м�ֵ
//    uint8 Start_Row_0 = 0;                  //��ʼY����
// 
//    Start_Row_0 = start_row;
//    Start_X = LCDW / 2;
    //���м�����ߣ��������
    for(j = 0; j < 10; j ++)        //ָ������û�ҵ����ʱ��������һ�м����ң������ʮ��
    {
        l_start_point[1] = start_row;//y
        r_start_point[1] = start_row;//y
// 
//        if(Start_Flag == 0)       //��һ��ͼ�������������ʱ����ʼX����ѡ��ͼ������е�
//        {
//            Start_X = LCDW / 2;
//        }
//        else
//        {//            Start_X = (l_start_point[0] + r_start_point[0]) / 2;    //������ʼX��������һ��ͼ������������ʼ����м�ֵ

//        }
            for (i = 3; i < r_border_x ; i++)      //����������ʼ��
            {
                if (Compare_Num(image[start_row][i + 1], image[start_row][i], Compare_Value))//��Ⱥ�Ϊ��
                {
                    {
                        l_start_point[0] = i;   //�ҵ����¼X����
                        record_gray_value[0]=image[start_row][i];
                        L_Is_Found = 1;         //�ҳ��Ҽ���־λ
                        break;
                    }
                }
            }
 
            for (i = l_start_point[0] + 2; i < r_border_x ; i++)      //����������ʼ��
            {
                if (Compare_Num(image[start_row][i], image[start_row][i + 1], Compare_Value))//��Ⱥ�Ϊ��
                {
                    {
                        r_start_point[0] = i + 1;
                        record_gray_value[1]=image[start_row][i+1];
                        R_Is_Found = 1;
                        break;
                    }
                }
            }
            if(L_Is_Found && R_Is_Found)
            {
                Start_Flag = 1;    //�Ƿ�Ϊ��һ��ͼ���־λ
                return 1;
            }
            else
            {
                start_row = start_row - 1;      //��������һ��û�ҵ����������ƶ�һ��������
            }
    }
}

/*************************************************************************
 *  �������ƣ�void labyrinth_findline_OSTU(void)
 *  ����˵��������Ӧ�������Թ���
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2025��5��13��
 *************************************************************************/                                 

/******
* �������ܣ�      ��ȡ������ά�������
* ����˵����      �����Ͻ����������Ӧ�Թ��Ż��������Ӧ�����Թ�
* ��  �Σ�        uint16 Break_Flag         ���ѭ����������ֹ��������һ��Ϊ3~4��ͼ����
*                uint8(*image)[Image_X]     ��ȡ���ߵ�ͼ��1
*                uint8(*l_line)[2]          ��������ߵĶ�ά����1
*                uint8(*r_line)[2]          ����Ҳ���ߵĶ�ά����1
*                int8 *l_dir                ���������ÿ�������������1
*                int8 *r_dir                ����Ҳ����ÿ�������������1
*                uint16 *l_stastic          ��¼�����ߵ�ĸ���1
*                uint16 *r_stastic          ��¼�Ҳ���ߵ�ĸ���1
*                uint8 *x_meet              ��¼�������������������X����1
*                uint8 *y_meet              ��¼�������������������Y����1
*                uint8 l_start_x            ���������ʼ���X����
*                uint8 l_start_y            ���������ʼ���Y����
*                uint8 r_start_x            �Ҳ�������ʼ���X����
*                uint8 r_start_y            �Ҳ�������ʼ���Y����
*                uint8 clip_value           ����ÿ����ֵʱ��ӵľ���ֵ��һ��Ϊ-5 ~ 5������ǿ�зָ��ֱ����Ϊ0
*
* ʾ����         Dir_Labyrinth_5((uint16)Use_Num, Find_Line_Image, Adaptive_L_Line, Adaptive_R_Line, Adaptive_L_Grow_Dir, Adaptive_R_Grow_Dir, &Adaptive_L_Statics, &Adaptive_R_Statics, &Adaptive_X_Meet, &Adaptive_Y_Meet,
                   Adaptive_L_Start_Point[0], Adaptive_L_Start_Point[1], Adaptive_R_Start_Point[0], Adaptive_R_Start_Point[1], 0);
* ����ֵ��        ��
*/

void Dir_Labyrinth_5(uint16 Break_Flag, uint8(*image)[Image_X], uint16(*l_line)[2], uint16(*r_line)[2], int16 *l_dir, int16 *r_dir, uint16 *l_stastic, uint16 *r_stastic, uint8 *x_meet, uint8 *y_meet,
                     uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 clip_value)
{
    uint8 j = 0;
 
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
//��߱���
    uint8  L_Center_Point[2] = {0};     //���ÿ���ҵ���XY����
    uint16 L_Data_Statics = 0;          //ͳ������ҵ��ı��ߵ�ĸ���
 
    uint8  L_Front_Value = 0;           //��� �����ǰ����ĻҶ�ֵ
    uint8  L_Front_L_Value = 0;         //��� �������ǰ����ĻҶ�ֵ
 
    uint8  L_Dir = 0;                   //�˲�������ת��
    uint8  L_Turn_Num = 0;              //��¼ת������������ĵ�ǰ�����Ҷ��Ǻ�ɫ���أ��ͻ���һ����ת���ĴΣ���¼���Ĵ�ʱ�˳�ѭ����ֹ��������
    uint16 L_Pixel_Value_Sum = 0;       //���ĵ�����Χ24���������ֵ��
    float L_Thres = 0;                 //�ֲ���ֵ,��L_Pixel_Value_Sum / 25
 
//�ұ߱���
    uint8  R_Center_Point[2] = {0};     //���ÿ���ҵ���XY����
    uint16 R_Data_Statics = 0;          //ͳ���ұ��ҵ��ı��ߵ�ĸ���
 
    uint8  R_Front_Value = 0;           //�Ҳ� �����ǰ����ĻҶ�ֵ
    uint8  R_Front_R_Value = 0;         //�Ҳ� �������ǰ����ĻҶ�ֵ
 
    uint8  R_Dir = 0;                   //�˲�������ת��
    uint8  R_Turn_Num = 0;              //��¼ת������������ĵ�ǰ�����Ҷ��Ǻ�ɫ���أ��ͻ���һ����ת���ĴΣ���¼���Ĵ�ʱ�˳�ѭ����ֹ��������
    uint16 R_Pixel_Value_Sum = 0;       //���ĵ�����Χ24���������ֵ��
    float R_Thres = 0;                 //�ֲ���ֵ
 
//��һ�θ��������  ���ҵ������ֵ������
    L_Center_Point[0] = l_start_x + 1;//x
    L_Center_Point[1] = l_start_y;//y
    R_Center_Point[0] = r_start_x - 1;//x
    R_Center_Point[1] = r_start_y;//y
 
    //���������Թ�ѭ��
    while (Break_Flag--)
    {
         //���
        //�ж��������󣬹ҳ�ֹͣ��־λ����������ֹͣ��
        if(L_Stop_Flag == 0)
        {
            l_line[L_Data_Statics][0] = L_Center_Point[0];  //�ҵ������ĵ�X����������������
            l_line[L_Data_Statics][1] = L_Center_Point[1];  //�ҵ������ĵ�Y����������������
 
            if(L_Data_Statics != 0)
            {
                switch(l_dir[L_Data_Statics - 1])  //������һ����Ը�����һ����������������Ż�����ʱ��
                {
                    //�ӵڶ����㿪ʼ����һ�������ֵҪ25����ȫ����һ��
                    //���������������ʱ����ԭ�ȵ�25�μӷ������Ϊʮ�μ���
                    //��б������ʱ����ԭ�ȵ�25�μӷ������Ϊʮ�˴�����
                    //���Ի���ͼ�������Ŵ����߼��飬�Ϳ��Ժܿ��ٵ����
                    case 1:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 3][L_Center_Point[0] + 2] - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] + 0] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2];
                        break;
                    }
                    case -2:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 0][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 2][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 3][L_Center_Point[0] + 2]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 0]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] + 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2];
                        break;
                    }
                    case -3:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 2][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 0][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 2][L_Center_Point[0] + 3]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] - 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2];
                        break;
                    }
                    case -4:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 2]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 2][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 0][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] + 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2];
                        break;
                    }
                    case -1:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 3][L_Center_Point[0] - 2] - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2];
                        break;
                    }
                    case 2:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 0][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 2][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 3][L_Center_Point[0] - 2]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] - 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2];
                        break;
                    }
                    case 3:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 2][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 0][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 2][L_Center_Point[0] - 3]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] + 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2];
                        break;
                    }
                    case 4:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 0]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 2]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 2][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 0][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] - 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2];
                        break;
                    }
                }
            }
            else
            {
                for (j = 0; j < 25; j++)    //��һ����ֵ��25����ȫ����һ�飬���������ֵ���������������
                {
                    L_Pixel_Value_Sum += image[L_Center_Point[1] + Square_0[j][1]][L_Center_Point[0] + Square_0[j][0]];
                }
            }
 
            L_Thres = (L_Pixel_Value_Sum + Thres_Interfere) / Thres_Num_Interfere;   //��ֵΪ25����Ҷ�ֵ��ƽ��ֵ
//            L_Thres -= clip_value;              //���õ��ĻҶ���ֵ��ȥһ������ֵ�������Ż��ж�
 
            //����Ϊ����ƽ���˲�����֪�����Ƿ��������㷨���˴�Ϊ����ԭ������������Ӧ�ɽϰ������ֲ����������򣬻�֮�Ĺ��������������ϸ����ԭ��
//            if(Thres_Filiter_Flag_1 == 1 || Thres_Filiter_Flag_2 == 1)
//            {
                if(L_Data_Statics > 3)
                {
                    L_Thres = L_Thres * 1.3f - L_Thres_Record[L_Data_Statics - 1] * 0.2f - L_Thres_Record[L_Data_Statics - 2] * 0.1f;
                }
//            }
            L_Thres_Record[L_Data_Statics] = (uint8_t)L_Thres;
            L_Data_Statics++;                   //ÿ�ҵ�һ����ͳ�Ƹ���+1
 
            L_Judge_Again:    //L_Judge_Again �� goto ���ʹ��
            if(L_Stop_Flag == 0)
            {
                L_Front_Value = image[L_Center_Point[1] + L_Face_Dir[L_Dir][1]][L_Center_Point[0] + L_Face_Dir[L_Dir][0]];          //��¼�����ǰ����ĻҶ�ֵ
                L_Front_L_Value = image[L_Center_Point[1] + L_Face_Dir_L[L_Dir][1]][L_Center_Point[0] + L_Face_Dir_L[L_Dir][0]];    //��¼�������ǰ����ĻҶ�ֵ
                if((float)L_Front_Value < L_Thres)     //�����ǰ�����Ǻ�ɫ
                {
                    L_Dir = (L_Dir + 1) % 4;    //����תһ��
                    L_Turn_Num ++;
                    if(L_Turn_Num == 4)        //��������
                    {
                        L_Stop_Flag = 1;       //��ǰ�����Ҷ��Ǻ�ɫʱ������������ֹͣ�������
                    }
                    goto L_Judge_Again;
                }
                else if((float)L_Front_L_Value < L_Thres)   //��ǰ�����Ǻ�ɫ��ǰ�����ǰ�ɫ
                {
                    L_Center_Point[0] += L_Face_Dir[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir[L_Dir][1];      //��ǰ��һ��
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir[L_Dir][0] * 3) - L_Face_Dir[L_Dir][1];
                    L_Turn_Num = 0;
                }
                else        //��ǰ����ǰ�����ǰ�ɫ��
                {
                    L_Center_Point[0] += L_Face_Dir_L[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir_L[L_Dir][1];        //����ǰ����һ��
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir_L[L_Dir][0] * 3) - L_Face_Dir_L[L_Dir][1];
                    L_Dir = (L_Dir + 3) % 4;        //��תһ��
                    L_Turn_Num = 0;
                }
                if(L_Data_Statics >= 5)     //O��������ת��һȦ��ص�ԭ����Ҳ��һ��������������ֹͣ����
                {
                    if(l_line[L_Data_Statics - 1][0] == l_line[L_Data_Statics - 5][0]&&
                       l_line[L_Data_Statics - 1][1] == l_line[L_Data_Statics - 5][1])
                    {
                        L_Stop_Flag = 1;
                    }
                }
            }
            if(L_Center_Point[0]<1) L_Center_Point[0]=1;
            if(L_Center_Point[0]>92) L_Center_Point[0]=92;
            if(L_Center_Point[1]<1) L_Center_Point[1]=1;
            if(L_Center_Point[1]>58) L_Center_Point[1]=58;
        }
 
        //�Ҳ������ͬ������Ҳ���ƣ���������Ҳ�ͺܼ�
        if(R_Stop_Flag == 0)
        {
            r_line[R_Data_Statics][0] = R_Center_Point[0];
            r_line[R_Data_Statics][1] = R_Center_Point[1];
 
            if(R_Data_Statics != 0)
            {
                switch(r_dir[R_Data_Statics - 1])
                {
                    case 1:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 3][R_Center_Point[0] + 2] - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] + 0] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2];
                        break;
                    }
                    case -2:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 0][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 2][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 3][R_Center_Point[0] + 2]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 0]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] + 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2];
                        break;
                    }
                    case -3:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 2][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 0][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 2][R_Center_Point[0] + 3]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] - 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2];
                        break;
                    }
                    case -4:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 2]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 2][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 0][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] + 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2];
                        break;
                    }
                    case -1:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 3][R_Center_Point[0] - 2] - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2];
                        break;
                    }
                    case 2:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 0][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 2][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 3][R_Center_Point[0] - 2]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] - 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2];
                        break;
                    }
                    case 3:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 2][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 0][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 2][R_Center_Point[0] - 3]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] + 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2];
                        break;
                    }
                    case 4:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 0]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 2]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 2][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 0][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] - 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2];
                        break;
                    }
                }
            }
            else
            {
                for (j = 0; j < 25; j++)
                {
                    R_Pixel_Value_Sum += image[R_Center_Point[1] + Square_0[j][1]][R_Center_Point[0] + Square_0[j][0]];
                }
            }
 
            R_Thres = (R_Pixel_Value_Sum + Thres_Interfere) / Thres_Num_Interfere;
//            R_Thres -= clip_value;
 
//            if(Thres_Filiter_Flag_1 == 1 || Thres_Filiter_Flag_2 == 1)
//            {
                if(R_Data_Statics > 3)
                {
                    R_Thres = R_Thres * 1.3f - R_Thres_Record[R_Data_Statics - 1] * 0.2f - R_Thres_Record[R_Data_Statics - 2] * 0.1f;
                }
//            }
 
            R_Thres_Record[R_Data_Statics] = (uint8_t)R_Thres;
 
            R_Data_Statics++;
 
            R_Judgme_Again:
            if(R_Stop_Flag == 0)
            {
                R_Front_Value = image[R_Center_Point[1] + R_Face_Dir[R_Dir][1]][R_Center_Point[0] + R_Face_Dir[R_Dir][0]];
                R_Front_R_Value = image[R_Center_Point[1] + R_Face_Dir_R[R_Dir][1]][R_Center_Point[0] + R_Face_Dir_R[R_Dir][0]];
                if((float)R_Front_Value < R_Thres)
                {
                    R_Dir = (R_Dir + 3) % 4;
                    R_Turn_Num ++;
                    if(R_Turn_Num == 4)
                    {
                        R_Stop_Flag = 1;
                    }
                    goto R_Judgme_Again;
                }
                else if((float)R_Front_R_Value < R_Thres)
                {
                    R_Center_Point[0] += R_Face_Dir[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir[R_Dir][1];
                    r_dir[R_Data_Statics - 1] = R_Face_Dir[R_Dir][0] * 3 - R_Face_Dir[R_Dir][1];
                    R_Turn_Num = 0;
                }
                else
                {
                    R_Center_Point[0] += R_Face_Dir_R[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir_R[R_Dir][1];
                    r_dir[R_Data_Statics - 1] = R_Face_Dir_R[R_Dir][0] * 3 - R_Face_Dir_R[R_Dir][1];
                    R_Dir = (R_Dir + 1) % 4;
                    R_Turn_Num = 0;
                }
                if(R_Data_Statics >= 5)
                {
                    if(r_line[R_Data_Statics - 1][0] == r_line[R_Data_Statics - 5][0]&&
                       r_line[R_Data_Statics - 1][1] == r_line[R_Data_Statics - 5][1])
                    {
                        R_Stop_Flag = 1;
                    }
                }
            }
            if(R_Center_Point[0]<1) R_Center_Point[0]=1;
            if(R_Center_Point[0]>92) R_Center_Point[0]=92;
            if(R_Center_Point[1]<1) R_Center_Point[1]=1;
            if(R_Center_Point[1]>58) R_Center_Point[1]=58;
        }
 
        if(L_Stop_Flag == 0 && R_Stop_Flag == 0)
        {
            if ((My_ABS(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 1)
                && (My_ABS(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 1))        //���������������˳�ѭ����һ��ͼ�����߽���
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //��¼������Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //��¼������X
                break;
            }
        }
        //��һ���������ʱ������������ж��ſ���һЩ����ֹʵ��������û���ж����������������ҵ����
        else
        {
            if ((My_ABS(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 3)
                && (My_ABS(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 3))        //���������������˳�ѭ����һ��ͼ�����߽���
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //��¼������Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //��¼������X
                break;
            }
        }
    }
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
    *l_stastic = L_Data_Statics;    //��¼�����ߵ����
    *r_stastic = R_Data_Statics;    //��¼�Ҳ���ߵ����
}

/**
* �������ܣ�      �ɶ�ά����������ȡһά����
* ����˵����      ��
* ��  �Σ�        uint16 l_total       //����ά���ߵ�ĸ���
*                 uint16 r_total      //�Ҳ��ά���ߵ�ĸ���
*                 uint8 start         //��ʼ�У�ͼ��ײ���
*                 uint8 end           //��ֹ�У�ͼ�񶥲���
*                 uint8 *l_border     //�洢���һά���ߵ�����
*                 uint8 *r_border     //�洢�Ҳ�һά���ߵ�����
*                 uint8(*l_line)[2]   //�洢����ά���ߵ�����
*                 uint8(*r_line)[2]   //�洢�Ҳ��ά���ߵ�����
*
* ʾ����          Get_Border(L_Statics, R_Statics, Image_Y - 3, 2, L_Border, R_Border, L_Line, R_Line);
* ����ֵ��        ��
*/
void Get_Border(uint16 l_total, uint16 r_total, uint8 start, uint8 end, uint8 *l_border, uint8 *r_border, uint8(*l_line)[2], uint8(*r_line)[2])
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < LCDH - 1; i++)
    {
        l_border[i] = 1;
        r_border[i] = LCDW - 2;     //�ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ������պ�����������߾ͻ����м䣬������ŵõ�������
    }
    h = start;
    //�ұ�
    for (j = 0; j < r_total; j++)
    {
        if (r_line[j][1] == h)
        {
            r_border[h] = r_line[j][0];
        }
        else
        {
            continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        }
        h--;
        if (h == end)
        {
            break;//�����һ���˳�
        }
    }
    h = start;
    for (j = 0; j < l_total; j++)
    {
        if (l_line[j][1] == h)
        {
            l_border[h] = l_line[j][0];
        }
        else
        {
            continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        }
        h--;
        if (h == end)
        {
            break;//�����һ���˳�
        }
    }
}

/**
* �������ܣ�      ��ȡ��ֵ�е������Сֵ,�������ֵ��ֵ
* ����˵����      ����ʱ��С��5us
* ��  �Σ�        ��
*
* ʾ����          Thres_Record_Process������
* ����ֵ��        ��
*/
void Thres_Record_Process(void)
{
    uint16_t i = 0;
    uint8 Left_Temp_Value_1 = 0;
    uint32 Left_Temp_Value_2 = 0;
    uint8 Right_Temp_Value_1 = 0;
    uint32 Right_Temp_Value_2 = 0;
    uint8 L_Average_Thres = 0;
    uint8 R_Average_Thres = 0;
 
    Adaptive_L_Thres_Max = 0;
    Adaptive_R_Thres_Max = 0;
    Adaptive_L_Thres_Min = 0;
    Adaptive_R_Thres_Min = 0;
 
    Adaptive_L_Thres_Max = L_Thres_Record[0];
    Adaptive_L_Thres_Min = L_Thres_Record[0];
    Adaptive_R_Thres_Max = R_Thres_Record[0];
    Adaptive_R_Thres_Min = R_Thres_Record[0];
 
    for(i = 0; i < Adaptive_L_Statics; i += 2)      //���ȡֵ���ɣ����ټ�����
    {
        if(Adaptive_L_Line[i][0] != 2 && Adaptive_L_Line[i][1] != 2)      //��ȥλ�ںڿ��ϵı��ߵ���ֵ����2���������λ�ںڿ���ʱ��X����
        {
            if(L_Thres_Record[i] < Adaptive_L_Thres_Min)
            {
                Adaptive_L_Thres_Min = L_Thres_Record[i];
            }
            if(L_Thres_Record[i] > Adaptive_L_Thres_Max)
            {
                Adaptive_L_Thres_Max = L_Thres_Record[i];
            }
            Left_Temp_Value_1 ++;
            Left_Temp_Value_2 += L_Thres_Record[i];
        }
    }
    for(i = 0; i < Adaptive_R_Statics; i += 2)
    {
        if(Adaptive_R_Line[i][0] != 91 && Adaptive_R_Line[i][1] != 2)       //�����ͬ��
        {
            if(R_Thres_Record[i] < Adaptive_R_Thres_Min)
            {
                Adaptive_R_Thres_Min = R_Thres_Record[i];
            }
            if(R_Thres_Record[i] > Adaptive_R_Thres_Max)
            {
                Adaptive_R_Thres_Max = R_Thres_Record[i];
            }
            Right_Temp_Value_1 ++;
            Right_Temp_Value_2 += R_Thres_Record[i];
        }
    }
 
    if(Left_Temp_Value_1 == 0)      //��������ȫ��λ�ڱ߽���ʱ��ֱ�ӽ���ֵ��ֵȡ0
    {
        L_Average_Thres = 0;
    }
    else        //��������ֵ��ֵ
    {
        L_Average_Thres = (uint8)(Left_Temp_Value_2 / Left_Temp_Value_1);
    }
 
    if(Right_Temp_Value_1 == 0)     //�����ͬ��
    {
        R_Average_Thres = 0;
    }
    else
    {
        R_Average_Thres = (uint8)(Right_Temp_Value_2 / Right_Temp_Value_1);
    }
 
    if(Image_Num <= 1)      //ǰ����ͼ��ֱ�����ֵ
    {
        Last_Adaptive_Thres_Average = (uint8)((L_Average_Thres + R_Average_Thres) / 2);
    }
    else
    {
        if(My_ABS(L_Average_Thres - R_Average_Thres) >= 40)       //��������ߵ���ֵ��ֵ������ʱ����һ������
        {
            if(My_ABS(L_Average_Thres - Last_Adaptive_Thres_Average) <= My_ABS(R_Average_Thres - Last_Adaptive_Thres_Average))      //ѡȡ������ֵ��ֵ��ӽ��ϴ�ͼ����ֵ��ֵ��ֵ��Ϊ�˴ε�ֵ
            {
                Adaptive_Thres_Average = (uint8)((Last_Adaptive_Thres_Average + L_Average_Thres) / 2);
                Last_Adaptive_Thres_Average = Adaptive_Thres_Average;
            }
            else
            {
                Adaptive_Thres_Average = (uint8)((Last_Adaptive_Thres_Average + R_Average_Thres) / 2);
                Last_Adaptive_Thres_Average = Adaptive_Thres_Average;
            }
        }
        else    //��������ֵ��ֵ����ʱ��ֱ�������߾�ֵ
        {
            Adaptive_Thres_Average = (uint8)((Last_Adaptive_Thres_Average + L_Average_Thres + R_Average_Thres) / 3);
            Last_Adaptive_Thres_Average = Adaptive_Thres_Average;
        }
    }
 
    //����жϰ����ߵ���ֵ
    //Zbra_Thres = Adaptive_Thres_Average - 10;
 
    //�����ʼ���Ⱥ͵���ֵ
    if(Adaptive_Thres_Average >= 100 && Adaptive_Thres_Average <= 140)      //��ֵ��ֵ����100 - 140֮�䣬˵��ͼ�����Ⱥܺ��ʣ���ʱ����Ⱥ���ֵ��Ϊ20����
    {
        Compare_Value = 20;
    }
    else if(Adaptive_Thres_Average < 100)
    {
        Compare_Value = 20 - (uint8)(((float)(100 - Adaptive_Thres_Average) / 60.0f) * 10.0f);      //��С��100ʱ���ʵ����Ͳ�Ⱥ���ֵ��ʹ����ͼ��ϰ�������¸������ҳ���������
    }
    else if(Adaptive_Thres_Average > 140)
    {
        Compare_Value = 20 - (uint8)(((float)(Adaptive_Thres_Average - 140) / 60.0f) * 10.0f);      //������140ʱ���ʵ����Ͳ�Ⱥ���ֵ��ʹ����ͼ�����������¸������ҳ���������
    }
     Black_Box_Value_1=(record_gray_value[0]+record_gray_value[1])/2;
    //��ڿ�Ҷ�ֵ
    Black_Box_Value = (uint8)(0.45f * (float)sqrt(Adaptive_L_Thres_Min * Adaptive_L_Thres_Min + Adaptive_R_Thres_Min * Adaptive_R_Thres_Min)) + (uint8)(0.1f * (float)Black_Box_Value_1);      
    //0.45f��0.9 * 0.5��0.9ΪȨ�أ�0.5���Լ����ڡ�Black_Box_Value_1Ϊ�����������ĻҶ�ֵ��ֵ�����ֵ�Ҷ�ֵΪͼ�����������ߵĻҶ�ֵ������Ϊ�ڿ�Ҷ�ֵ�������Լ�д�㷨�����£�
    //����Black_Box_Value_1ֱ�Ӷ�20 ~ 50֮���ֵ����
    
    
    if(Image_Num<2)
    {
      Image_Num++;    
    }
}

void  Element_Judge(void)
{
    if(Adaptive_X_Meet > 65 && Adaptive_Y_Meet < 35 && Adaptive_L_Statics > 35 && Adaptive_R_Statics > 25 && (Adaptive_L_Statics - Adaptive_R_Statics) > 0)
    {
         road_type.right_right_angle_bend = 1;
    }
    else
    {
         road_type.right_right_angle_bend = 0;
    }
    if(Adaptive_X_Meet < 23 && Adaptive_Y_Meet < 40 && Adaptive_L_Statics > 25 && Adaptive_R_Statics > 35 && (Adaptive_R_Statics - Adaptive_L_Statics) > 0)
    {
         road_type.left_right_angle_bend = 1;
    }
    else
    {
         road_type.left_right_angle_bend = 0;
    }
//    if()
//    {
//       
//    }
}

/*************************************************************************
ͼ������ܴ���
*************************************************************************/
void Camera_All_Deal(void)
{
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        Get_Use_Image();
        Draw_Black_Box(Black_Box_Value, Image_Use);
        Get_Start_Point(LCDH - 3, Image_Use, Adaptive_L_Start_Point, Adaptive_R_Start_Point, 91);
        Dir_Labyrinth_5(3.5 * LCDW, Image_Use, Adaptive_L_Line, Adaptive_R_Line, Adaptive_L_Grow_Dir, Adaptive_L_Grow_Dir, &Adaptive_L_Statics, &Adaptive_R_Statics, &Adaptive_X_Meet, &Adaptive_Y_Meet, Adaptive_L_Start_Point[0], Adaptive_L_Start_Point[1], Adaptive_R_Start_Point[0], Adaptive_R_Start_Point[1], 0);
//        Get_Border(Adaptive_L_Statics, Adaptive_R_Statics, LCDH - 3, 2, L_Border, R_Border, Adaptive_L_Line, Adaptive_R_Line);
        Thres_Record_Process();
        tft180_displayimage03x((const uint8 *)Image_Use, 94, 60);
        Element_Judge();
        
    }
}