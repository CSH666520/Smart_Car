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

uint8 image_01[60][94];           //��ֵ��ͼ��



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
                image_01[i][j] = 0;  //��
            }
            else
            {
                Bin_Image[i][j] = 0;
                image_01[i][j] = 255;  //��
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

/*************************************************************************
ͼ������ܴ���
*************************************************************************/
void Camera_All_Deal(void)
{
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        Get_Use_Image();
        Get_Bin_Image(0);
        Bin_Image_Filter();
        ips114_displayimage03x((const uint8 *)image_01, LCDH, LCDW);
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
            if(image_01[j][i] != 0x00&&image_01[j-1][i] != 0x00)
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
            if (image_01[i][j] ==0x00 && image_01[i][j + 1] != 0x00 && image_01[i][j + 2] != 0x00)//�׺ںڣ��ҵ��ұ߽�
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
            if (image_01[i][j] ==0x00 && image_01[i][j - 1] != 0x00 && image_01[i][j - 2] != 0x00)//�ںڰ���Ϊ������߽�
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
/************************************************************************
Ԫ���ж�
*************************************************************************/
int16 top_junp_change_sign_num, bottom_junp_change_sign_num, left_junp_change_sign_num, right_junp_change_sign_num;//���ڼ�¼�������Һڰ���������
int right_right_angle_flag, left_right_angle_flag; //�ж�����ֱ�Ǳ���
int ten_sign;//�ж�ʮ�ֱ���
void  Element_Judge(void)
{
//     int16 right_angle_white_column, left_angle_white_column;//ֱ�ǰ����жϱ���
     int16 top, bottom, left, right;
     int16 row, column;
     
    for (top = 10; top < 20 ; top++)//������ɨ����10-20��
    {
        for (column = 0; column < LCDW - 1; column++)
        {
             if (image_01[top][column-1] != 0x00 && image_01[top][column] == 0x00&& image_01[top][column+1] == 0x00)//�ڰװ�
             {
                 top_junp_change_sign_num++;
             }
             if (image_01[top][column-1] == 0x00 && image_01[top][column] != 0x00&& image_01[top][column+1] != 0x00)//�׺ں�
             {
                 top_junp_change_sign_num++;
             }
        }
    }
   for (bottom = 54; bottom > 44 ; bottom--)//������ɨ�ײ�45-55��
    {
        for (column = 0; column < LCDW - 1; column++)
        {
             if (image_01[bottom][column-1] != 0x00 && image_01[bottom][column] == 0x00 && image_01[bottom][column+1] == 0x00)//�ڰװ�
             {
                 bottom_junp_change_sign_num++;
             }
             if (image_01[bottom][column-1] == 0x00 && image_01[bottom][column] != 0x00 && image_01[bottom][column+1] != 0x00)//�׺ں�
             {
                 bottom_junp_change_sign_num++;
             }
        }
    }
    for (left = 10; left < 20; left++)//���µ���ɨ��ߵ�10-20��
    {
        for (row = LCDH - 1; column > 0; column--)
        {
             if (image_01[row-1][left] != 0x00 && image_01[row][left] == 0x00 && image_01[row+1][left] == 0x00)//�ڰװ�
             {
               left_junp_change_sign_num++;
             }
             if (image_01[row-1][left] == 0x00 && image_01[row][left] != 0x00 && image_01[row+1][left] != 0x00)//�׺ں�
             {
               left_junp_change_sign_num++;
             }
        }
    }   
    for (right = 83; right > 73; right--)//���µ���ɨ�ұߵ�74-84��
    {
        for (row = LCDH - 1; column > 0; column--)
        {
             if (image_01[row-1][right] != 0x00 && image_01[row][right] == 0x00 && image_01[row+1][right] == 0x00)//�ڰװ�
             {
                 right_junp_change_sign_num++;
             }
             if (image_01[row-1][right] == 0x00 && image_01[row][right] != 0x00 && image_01[row+1][right] != 0x00)//�׺ں�
             {
                 right_junp_change_sign_num++;
             }
        }
    }
    if(top_junp_change_sign_num <= 0 && bottom_junp_change_sign_num >= 2 && left_junp_change_sign_num <= 0 && right_junp_change_sign_num >= 2)//��ֱ��
    {
        right_right_angle_flag=1;
    }
    if(top_junp_change_sign_num <= 0 && bottom_junp_change_sign_num >= 2 && left_junp_change_sign_num >= 2 && right_junp_change_sign_num <= 0)//��ֱ��
    {
        left_right_angle_flag=1;
    }
    if(top_junp_change_sign_num >= 2 && bottom_junp_change_sign_num >= 2 && left_junp_change_sign_num >= 2 && right_junp_change_sign_num >= 2)//ʮ��
    {
        ten_sign=1;
    }
}