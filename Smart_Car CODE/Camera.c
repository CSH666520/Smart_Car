#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "Camera.h"


#define LCDH    60  /*!< TFT显示高度（用户使用）高度 */
#define LCDW    94  /*!< TFT显示宽度（用户使用）宽度 */
#define MT9V034_IMAGEH  120  /*!< 行 HEIGHT 待采集摄像头图像高度行数 */
#define MT9V034_IMAGEW  188  /*!< 列 WIDTH  待采集摄像头图像宽度列数 */

/** 图像原始数据存放 */
extern uint8    mt9v03x_image[MT9V034_IMAGEH][MT9V034_IMAGEW];
/** 压缩后之后用于存放屏幕显示数据  */
unsigned char Image_Use[LCDH][LCDW];

/** 二值化后用于屏幕显示的数据 */
unsigned char Bin_Image[LCDH][LCDW];

uint8 image_01[60][94];           //二值化图像



int16 Longest_White_Column_Left_site,Longest_White_Column_Right_site, longest_White_Column_site;//搜线变量
int Longest_White_Column_Left,Longest_White_Column_Right;
uint8 l_lose_value = 0, r_lose_value = 0;                                   //左右丢线数
uint8 l_search_flag[LCDH], r_search_flag[LCDH];                   //是否搜到线的标志
int Boundry_Start_Right,Boundry_Start_Left;
int longest_White_Column=0,Both_Lost_Time;
int16 Right_Lost_Flag[LCDH],Left_Lost_Flag[LCDH];
int16 l_line_x[LCDH], r_line_x[LCDH], m_line_x[LCDH];        //储存原始图像的左右边界的列数，用于元素判断
int16 l_line_x_l[LCDH], r_line_x_l[LCDH], m_line_x_l[LCDH];  //储存原始图像的左右边界的列数，用于偏差计算
int search_line_end = 5;//不能为零，原因暂未查

/*************************************************************************
 *  函数名称：short GetOSTU (unsigned char tmImage[LCDH][LCDW])
 *  功能说明：大津法求阈值大小
 *  参数说明：tmImage ： 图像数据
 *  函数返回：无
 *  修改时间：2011年10月28日
 *  备    注：  GetOSTU(Image_Use);//大津法阈值
Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
3) i表示分类的阈值，也即一个灰度级，从0开始迭代 1
4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像
        的比例w0，        并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背
        景像素)  * 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
6) i++；转到4)，直到i为256时结束迭代
7) 将最大g相应的i值作为图像的全局阈值
缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
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
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < LCDH; j++)
    {
        for (i = 0; i < LCDW; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}


/*************************************************************************
 *  函数名称：void Get_Bin_Image (unsigned char mode)
 *  功能说明：图像二值化到Bin_Image[][]
 *  参数说明：mode  ：
 *    0：使用大津法阈值
 *    1：使用平均阈值
 *    2: sobel 算子改进型  手动阈值，同时输出改为提取边沿的图像
 *    3：sobel 算子改进型   动态阈值，同时输出改为提取边沿的图像
 *  函数返回：无
 *  修改时间：2020年10月28日
 *  备    注：  Get_Bin_Image(0); //使用大津法二值化
 *************************************************************************/
void Get_Bin_Image (unsigned char mode)
{
    unsigned short i = 0, j = 0;
    unsigned short Threshold = 0;
    //char txt[16];

    if (mode == 0)
    {
        Threshold = GetOSTU(Image_Use);  //大津法阈值
    }
    
    
    
    /* 二值化 */
    for (i = 0; i < LCDH; i++)
    {
        for (j = 0; j < LCDW; j++)
        {
            if (Image_Use[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
            {
                Bin_Image[i][j] = 1;
                image_01[i][j] = 0;  //白
            }
            else
            {
                Bin_Image[i][j] = 0;
                image_01[i][j] = 255;  //黑
            }
        }
    }
}

/*---------------------------------------------------------------
 【函    数】Bin_Image_Filter
 【功    能】过滤噪点
 【参    数】无
 【返 回 值】无
 【注意事项】
 ----------------------------------------------------------------*/
void Bin_Image_Filter (void)
{
    int nr; //行
    int nc; //列

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
 *  函数名称：void Get_Use_Image (void)
 *  功能说明：把摄像头采集到原始图像，缩放到赛道识别所需大小
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年10月28日
 *  备    注：  IMAGEW为原始图像的宽度，神眼为188，OV7725为320
 *       IMAGEH为原始图像的高度，神眼为120，OV7725为240
 *************************************************************************/
void Get_Use_Image(void)
{
    short i = 0, j = 0, row = 0, line = 0;

    for (i = 0; i < MT9V034_IMAGEH; i += 2)          //神眼高 120 / 2  = 60，
    {
        for (j = 0; j <= MT9V034_IMAGEW; j += 2)     //神眼宽188 / 2  = 94，
        {
            Image_Use[row][line] = mt9v03x_image[i][j];
            line++;
        }
        line = 0;
        row++;
    }
}

/*************************************************************************
图像最后总处理
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
八邻域找底部边线
*************************************************************************/
//void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
//{
//
//    //寻找左边边界
//    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
//    {
//        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
//        {
//            ImageDeal[Bottonline].LeftBoundary = Xsite;//获取底边左边线
//            break;
//        }
//    }
//    for (int Xsite = Col / 2+2; Xsite < LCDW-1; Xsite++)
//    {
//        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
//        {
//            ImageDeal[Bottonline].RightBoundary = Xsite;//获取底边右边线
//            break;
//        }
//    }
//
//}
/************************************************************************
八邻域获取左右边线
*************************************************************************/
//void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], int Row, int Col, int Bottonline)
//{
//    //定义小人的当前行走状态位置为 上 左 下 右 一次要求 上：左边为黑色 左：上边为褐色 下：右边为色  右：下面有黑色
///*  前进方向定义：
//                *   0
//                * 3   1
//                *   2
//*/
///*寻左线坐标规则*/
//    int Left_Rule[2][8] = {
//                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
//                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
//    };
//    /*寻右线坐标规则*/
//    int Right_Rule[2][8] = {
//                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
//                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
//    };
//      int num=0;
//    uint8 Left_Ysite = Bottonline;
//    uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
//    uint8 Left_Rirection = 0;//左边方向
//    uint8 Pixel_Left_Ysite = Bottonline;
//    uint8 Pixel_Left_Xsite = 0;
//
//    uint8 Right_Ysite = Bottonline;
//    uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
//    uint8 Right_Rirection = 0;//右边方向
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
//        /*********左边巡线*******/
//        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
//        {
//            /*计算前方坐标*/
//            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
//            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
//
//            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 255)//前方是黑色
//            {
//                //顺时针旋转90
//                if (Left_Rirection == 3)
//                    Left_Rirection = 0;
//                else
//                    Left_Rirection++;
//              
//            }
//            else//前方是白色
//            {
//                /*计算左前方坐标*/
//                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
//                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
//
//                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 255)//左前方为黑色
//                {
//                    //方向不变  Left_Rirection
//                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
//                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
//                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0){
//                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
//                        ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
//                    }
//                }
//                else//左前方为白色
//                {
//                    // 方向发生改变 Left_Rirection  逆时针90度
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
//        /*********右边巡线*******/
//        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
//        {
//            /*计算前方坐标*/
//            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
//            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
//
//            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 255)//前方是黑色
//            {
//                //逆时针旋转90
//                if (Right_Rirection == 0)
//                    Right_Rirection = 3;
//                else
//                    Right_Rirection--;
//            }
//            else//前方是白色
//            {
//                /*计算右前方坐标*/
//                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
//                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
//
//                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 255)//左前方为黑色
//                {
//                    //方向不变  Right_Rirection
//                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
//                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
//                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79 )
//                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
//                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
//                }
//                else//左前方为白色
//                {
//                    // 方向发生改变 Right_Rirection  逆时针90度
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
//        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80是为了放在底部是斑马线扫描结束  3 && Ysite < 30
//        {
//            ImageStatus.OFFLineBoundary = Ysite;
//            break;
//        }
//
//    }
//}
//
///************************************************************************
//八邻域
//*************************************************************************/
//void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
//{
//    ImageStatus.WhiteLine_L = 0;
//    ImageStatus.WhiteLine_R = 0;
//    //ImageStatus.OFFLine = 1;
//    /*封上下边界处理*/
//    for (int Xsite = 0; Xsite < LCDW; Xsite++)
//    {
//        imageInput[0][Xsite] = 0;
//        imageInput[Bottonline + 1][Xsite] = 0;
//    }
//    /*封左右边界处理*/
//    for (int Ysite = 0; Ysite < LCDH; Ysite++)
//    {
//            ImageDeal[Ysite].LeftBoundary_First = 0;
//            ImageDeal[Ysite].RightBoundary_First = 79;
//
//            imageInput[Ysite][0] = 0;
//            imageInput[Ysite][LCDW - 1] = 0;
//    }
//    /********获取底部边线*********/
//    Search_Bottom_Line_OTSU(image_01, Row, Col, Bottonline);
//    /********获取左右边线*********/
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
最长白列巡线
*************************************************************************/
volatile int White_Column[LCDW];
void dou_Longest_White_Column(void)//最长白列巡线
{
    int16 i, j;
    int16 start_column=0;//最长白列的搜索区间
    int16 end_column=LCDW;
    //int16 left_border = 0, right_border = 0;//临时存储赛道位置
    Longest_White_Column_Left = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left_site = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right_site = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    r_lose_value = 0;    //边界丢线数
    l_lose_value  = 0;
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//两边同时丢线数
    longest_White_Column=0;
    longest_White_Column_site = 0;

    for (i = 0; i <=LCDH-1; i++)//数据清零
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        l_line_x[i] = 0;
        r_line_x[i] = LCDW-1;//
        l_line_x_l[i]=0;
        r_line_x_l[i]=LCDW-1;
    }
    
    
    
     //清零数组
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
    
    
    search_line_end = longest_White_Column;//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
    if(search_line_end>=55){search_line_end=55;longest_White_Column=55;}
    for (i = LCDH - 1; i >=LCDH-search_line_end; i--)//常规巡线
    {
        for (j = longest_White_Column_site; j <= LCDW - 1 - 2; j++)
        {
            if (image_01[i][j] ==0x00 && image_01[i][j + 1] != 0x00 && image_01[i][j + 2] != 0x00)//白黑黑，找到右边界
            {
                r_line_x[i] = j;
                r_line_x_l[i] = j;
                Right_Lost_Flag[i] = 0; //右丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j>=LCDW-1-3)//没找到右边界，把屏幕最右赋值给右边界
            {
                r_line_x[i] = j;
                r_line_x_l[i] = j;
                r_lose_value++;
                Right_Lost_Flag[i] = 1;//右丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        for (j = longest_White_Column_site; j >= 0 + 2; j--)//往左边扫描
        {
            if (image_01[i][j] ==0x00 && image_01[i][j - 1] != 0x00 && image_01[i][j - 2] != 0x00)//黑黑白认为到达左边界
            {
                l_line_x [i] = j;
                l_line_x_l[i] = j;
                Left_Lost_Flag[i]=0;//l_lose_value = 0; //左丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j<=0+3)
            {
                l_line_x [i] = j;//找到头都没找到边，就把屏幕最左右当做边界
                l_line_x_l[i] = j;
                l_lose_value++;
                Left_Lost_Flag[i]=1;//左丢线数组，丢线置1，不丢线置0
//                if(Right_Lost_Flag[i] == 1)Both_Lost_Time++;
                break;
            }
        }

    }

//
//    for (i = LCDH - 1; i >= 0; i--)//赛道数据初步分析
//    {
////        if (Left_Lost_Flag[i]  == 1)//单边丢线数
////            l_lose_value++;
////        if (Right_Lost_Flag[i] == 1)
////            r_lose_value++;
////        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//双边丢线数
////            Both_Lost_Time++;
//        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)//记录第一个非丢线点，边界起始点
//            Boundry_Start_Left = i;
//        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
//            Boundry_Start_Right = i;
//
//     }  //Road_Wide[i]=r_line_x[i]-l_line_x[i];
}
/************************************************************************
元素判断
*************************************************************************/
int16 top_junp_change_sign_num, bottom_junp_change_sign_num, left_junp_change_sign_num, right_junp_change_sign_num;//用于记录上下左右黑白跳变数量
int right_right_angle_flag, left_right_angle_flag; //判断左右直角变量
int ten_sign;//判断十字变量
void  Element_Judge(void)
{
//     int16 right_angle_white_column, left_angle_white_column;//直角白列判断变量
     int16 top, bottom, left, right;
     int16 row, column;
     
    for (top = 10; top < 20 ; top++)//从左到右扫顶部10-20行
    {
        for (column = 0; column < LCDW - 1; column++)
        {
             if (image_01[top][column-1] != 0x00 && image_01[top][column] == 0x00&& image_01[top][column+1] == 0x00)//黑白白
             {
                 top_junp_change_sign_num++;
             }
             if (image_01[top][column-1] == 0x00 && image_01[top][column] != 0x00&& image_01[top][column+1] != 0x00)//白黑黑
             {
                 top_junp_change_sign_num++;
             }
        }
    }
   for (bottom = 54; bottom > 44 ; bottom--)//从左到右扫底部45-55行
    {
        for (column = 0; column < LCDW - 1; column++)
        {
             if (image_01[bottom][column-1] != 0x00 && image_01[bottom][column] == 0x00 && image_01[bottom][column+1] == 0x00)//黑白白
             {
                 bottom_junp_change_sign_num++;
             }
             if (image_01[bottom][column-1] == 0x00 && image_01[bottom][column] != 0x00 && image_01[bottom][column+1] != 0x00)//白黑黑
             {
                 bottom_junp_change_sign_num++;
             }
        }
    }
    for (left = 10; left < 20; left++)//从下到上扫左边第10-20行
    {
        for (row = LCDH - 1; column > 0; column--)
        {
             if (image_01[row-1][left] != 0x00 && image_01[row][left] == 0x00 && image_01[row+1][left] == 0x00)//黑白白
             {
               left_junp_change_sign_num++;
             }
             if (image_01[row-1][left] == 0x00 && image_01[row][left] != 0x00 && image_01[row+1][left] != 0x00)//白黑黑
             {
               left_junp_change_sign_num++;
             }
        }
    }   
    for (right = 83; right > 73; right--)//从下到上扫右边第74-84行
    {
        for (row = LCDH - 1; column > 0; column--)
        {
             if (image_01[row-1][right] != 0x00 && image_01[row][right] == 0x00 && image_01[row+1][right] == 0x00)//黑白白
             {
                 right_junp_change_sign_num++;
             }
             if (image_01[row-1][right] == 0x00 && image_01[row][right] != 0x00 && image_01[row+1][right] != 0x00)//白黑黑
             {
                 right_junp_change_sign_num++;
             }
        }
    }
    if(top_junp_change_sign_num <= 0 && bottom_junp_change_sign_num >= 2 && left_junp_change_sign_num <= 0 && right_junp_change_sign_num >= 2)//右直角
    {
        right_right_angle_flag=1;
    }
    if(top_junp_change_sign_num <= 0 && bottom_junp_change_sign_num >= 2 && left_junp_change_sign_num >= 2 && right_junp_change_sign_num <= 0)//右直角
    {
        left_right_angle_flag=1;
    }
    if(top_junp_change_sign_num >= 2 && bottom_junp_change_sign_num >= 2 && left_junp_change_sign_num >= 2 && right_junp_change_sign_num >= 2)//十字
    {
        ten_sign=1;
    }
}