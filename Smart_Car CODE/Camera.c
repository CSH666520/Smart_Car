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

//int16 top_junp_change_sign_num= 0, bottom_junp_change_sign_num = 0, left_junp_change_sign_num = 0, right_junp_change_sign_num = 0;//用于记录上下左右黑白跳变数量


uint8 image_01[60][94];           //二值化图像

/**八向迷宫法变量**/
uint8 Black_Box_Value_FFF = 50;
uint8 Black_Box_Value_FF = 50;
uint8 Black_Box_Value_F = 50;
uint8 Black_Box_Value; 
uint8 Compare_Value = 20;
int Thres_Interfere = 0;
int Thres_Num_Interfere = 25;//25个阈值点
uint8 Start_Flag;
uint8 l_x_start, l_y_start, r_x_start, r_y_start;        //八向迷宫法起始点坐标
uint16_t L_Thres_Record[500];     //储存阈值数组
uint16_t R_Thres_Record[500];
uint8 Adaptive_L_Start_Point[2];          //存储左侧起始点的数组（全局变量）
uint8 Adaptive_R_Start_Point[2];          //存储右侧起始点的数组（全局变量）
uint8_t record_gray_value[2];             //记录左右起点灰度值
uint8_t Black_Box_Value_1;
uint8 L_Border[200];     //存储左侧一维边线的数组
uint8 R_Border[200];     //存储右侧一维边线的数组
uint16 Adaptive_L_Line[600][2];             //存放左侧边线的二维数组
uint16 Adaptive_R_Line[600][2];             //存放右侧边线的二维数组
uint16_t Adaptive_L_Statics;                //记录左边边线点的个数
uint16_t Adaptive_R_Statics;                //记录右边边线点的个数
int16_t Adaptive_L_Grow_Dir[500];                //存放左侧边线每个点的生长方向
int16_t Adaptive_R_Grow_Dir[500];                //存放右侧边线每个点的生长方向
uint8 Adaptive_X_Meet;              //记录左右两侧爬线相遇点的X坐标
uint8 Adaptive_Y_Meet;              //记录左右两侧爬线相遇点的Y坐标
uint8 Adaptive_L_Thres_Max = 0;     //左侧阈值最大值
uint8 Adaptive_R_Thres_Max = 0;     //右侧阈值最大值
uint8 Adaptive_L_Thres_Min = 0;     //左侧阈值最小值
uint8 Adaptive_R_Thres_Min = 0;     //右侧阈值最小值
uint8 Adaptive_Thres_Average = 0;   //阈值均值
uint8 Last_Adaptive_Thres_Average = 0;  //用于阈值均值滤波
uint16_t Image_Num;  //图像个数

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

/******************自适应方向迷宫参数************************/
 
const int8 L_Face_Dir[4][2] = {{0,-1},{1,0},{0,1},{-1,0}};  //左侧迷宫面向
//  0
//3   1
//  2
 
const int8 L_Face_Dir_L[4][2] = {{-1,-1},{1,-1},{1,1},{-1,1}};  //左侧面向的左前方
//0   1
//
//3   2
 
const int8 R_Face_Dir[4][2] = {{0,-1},{1,0},{0,1},{-1,0}};  //右侧迷宫面向
//  0
//3   1
//  2
 
const int8 R_Face_Dir_R[4][2] = {{1,-1},{1,1},{-1,1},{-1,-1}};  //右侧面向的右前方
//3   0
//
//2   1
 
const int8 Square_0[25][2] = {              //一个5 * 5的矩阵，用来求中心点周围的局部阈值（局部阈值）
{-2,-2},{-1,-2},{0,-2},{+1,-2},{+2,-2},
{-2,-1},{-1,-1},{0,-1},{+1,-1},{+2,-1},
{-2,-0},{-1, 0},{0, 0},{+1, 0},{+2,-0},
{-2,+1},{-1,+1},{0,+1},{+1,+1},{+2,+1},
{-2,+2},{-1,+2},{0,+2},{+1,+2},{+2,+2}
};
 
//迷宫单侧停止爬线标志位
uint8 L_Stop_Flag;
uint8 R_Stop_Flag;

/**********元素处理结构体**********/
struct YUAN_SU road_type = {     
      .barrier                       = 0,         //横断
      .straight                      = 0,         //直道
      .right_right_angle_bend        = 0,         //右直角弯道
      .ten                           = 0,         //十字
      .left_right_angle_bend         = 0,         //左直角弯道
      .ben_ring                      = 0,         //苯环
      
};

/******************************绝对值函数****************************/
int My_ABS(int num)      //八向迷宫法所用
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
                image_01[i][j] = 255;  //白
            }
            else
            {
                Bin_Image[i][j] = 0;
                image_01[i][j] = 0;  //黑
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
    
    
    search_line_end = longest_White_Column;//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
    if(search_line_end>=55){search_line_end=55;longest_White_Column=55;}
    for (i = LCDH - 1; i >=LCDH-search_line_end; i--)//常规巡线
    {
        for (j = longest_White_Column_site; j <= LCDW - 1 - 2; j++)
        {
            if (image_01[i][j] != 0x00 && image_01[i][j + 1] == 0x00 && image_01[i][j + 2] == 0x00)//白黑黑，找到右边界
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
            if (image_01[i][j] != 0x00 && image_01[i][j - 1] == 0x00 && image_01[i][j - 2] == 0x00)//白黑黑认为到达左边界
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

//画黑框（必须为一个像素宽度，边界务必空出一格）
/**
* 函数功能：      图像补黑框
* 特殊说明：      注意黑框与边界间隔一格像素宽度
* 形  参：        uint8 black_box_value            黑框的灰度值
*                uint8(*image)[Image_X]            要补黑框的图像
*
* 示例：          Draw_Black_Box(Black_Box_Value, Find_Line_Image);；
* 返回值：        无
*/
void Draw_Black_Box(uint8 black_box_value, uint8(*image)[Image_X])          
{
    uint8 i,j;
 
    Black_Box_Value_FFF = Black_Box_Value_FF;
    Black_Box_Value_FF = Black_Box_Value_F;
    Black_Box_Value_F = black_box_value;
    black_box_value = (uint8_t)(0.5 * Black_Box_Value_F + 0.3 * Black_Box_Value_FF + 0.2 * Black_Box_Value_FFF);        //滤波
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
* 函数功能：      差比和
* 特殊说明：      用于爬线算法找起点
* 形  参：        int16 a                  数值较大的灰度值
*                int16 b                   数值较小的灰度值
*                uint8 compare_value       差比和阈值
*
* 示例：          Compare_Num(image[start_row][i + 5], image[start_row][i], Compare_Value)；
* 返回值：        大于阈值返回1，否则返回0.
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
* 函数功能：      爬线算法找起点
* 特殊说明：      无
* 形  参：        uint8 start_row              找起点的图像行Y坐标
*                uint8(*image)[Image_X]        要处理的图像
*                uint8 *l_start_point          存储左侧起始点的数组（全局变量）
*                uint8 *r_start_point          存储右侧起始点的数组（全局变量）
*                uint8 l_border_x              向左找起点的截止点，最远找到这里就停止
*                uint8 r_border_x              向右找起点的截止点，最远找到这里就停止
*
* 示例：          Get_Start_Point(Image_Y - 3, Find_Line_Image, Adaptive_L_Start_Point, Adaptive_R_Start_Point, 1, 78)
* 返回值：        两边都找到返回1，否则返回0.
*/
uint8 Get_Start_Point(uint8 start_row, uint8(*image)[Image_X], uint8 *l_start_point, uint8 *r_start_point, uint8 r_border_x)          //*****
{
    uint8 i = 0, j = 0;
    uint8 L_Is_Found = 0, R_Is_Found = 0;   //找到起点时挂出对应标志位
//    uint8 Start_X  = 0;                     //起始X坐标，第一张图像取图像的行中点，后续图像用上一次图像左右两侧起始点的中间值
//    uint8 Start_Row_0 = 0;                  //起始Y坐标
// 
//    Start_Row_0 = start_row;
//    Start_X = LCDW / 2;
    //从中间往左边，先找起点
    for(j = 0; j < 10; j ++)        //指定的行没找到起点时，向上走一行继续找，最多找十行
    {
        l_start_point[1] = start_row;//y
        r_start_point[1] = start_row;//y
// 
//        if(Start_Flag == 0)       //第一张图像和遇到斑马线时，起始X坐标选用图像的行中点
//        {
//            Start_X = LCDW / 2;
//        }
//        else
//        {//            Start_X = (l_start_point[0] + r_start_point[0]) / 2;    //否则起始X坐标用上一次图像左右两侧起始点的中间值

//        }
            for (i = 3; i < r_border_x ; i++)      //向右找左起始点
            {
                if (Compare_Num(image[start_row][i + 1], image[start_row][i], Compare_Value))//差比和为真
                {
                    {
                        l_start_point[0] = i;   //找到后记录X坐标
                        record_gray_value[0]=image[start_row][i];
                        L_Is_Found = 1;         //挂出找见标志位
                        break;
                    }
                }
            }
 
            for (i = l_start_point[0] + 2; i < r_border_x ; i++)      //向左找右起始点
            {
                if (Compare_Num(image[start_row][i], image[start_row][i + 1], Compare_Value))//差比和为真
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
                Start_Flag = 1;    //是否为第一张图像标志位
                return 1;
            }
            else
            {
                start_row = start_row - 1;      //当此行有一侧没找到，就向上移动一行重新找
            }
    }
}

/*************************************************************************
 *  函数名称：void labyrinth_findline_OSTU(void)
 *  功能说明：自适应八项向迷宫法
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2025年5月13日
 *************************************************************************/                                 

/******
* 函数功能：      求取赛道二维数组边线
* 特殊说明：      基于上交代码的自适应迷宫优化后的自适应八向迷宫
* 形  参：        uint16 Break_Flag         最大循环次数，防止卡死程序，一般为3~4倍图像宽度
*                uint8(*image)[Image_X]     提取边线的图像1
*                uint8(*l_line)[2]          存放左侧边线的二维数组1
*                uint8(*r_line)[2]          存放右侧边线的二维数组1
*                int8 *l_dir                存放左侧边线每个点的生长方向1
*                int8 *r_dir                存放右侧边线每个点的生长方向1
*                uint16 *l_stastic          记录左侧边线点的个数1
*                uint16 *r_stastic          记录右侧边线点的个数1
*                uint8 *x_meet              记录左右两侧爬线相遇点的X坐标1
*                uint8 *y_meet              记录左右两侧爬线相遇点的Y坐标1
*                uint8 l_start_x            左侧爬线起始点的X坐标
*                uint8 l_start_y            左侧爬线起始点的Y坐标
*                uint8 r_start_x            右侧爬线起始点的X坐标
*                uint8 r_start_y            右侧爬线起始点的Y坐标
*                uint8 clip_value           计算每个阈值时相加的经验值，一般为-5 ~ 5，避免强行分割，可直接设为0
*
* 示例：         Dir_Labyrinth_5((uint16)Use_Num, Find_Line_Image, Adaptive_L_Line, Adaptive_R_Line, Adaptive_L_Grow_Dir, Adaptive_R_Grow_Dir, &Adaptive_L_Statics, &Adaptive_R_Statics, &Adaptive_X_Meet, &Adaptive_Y_Meet,
                   Adaptive_L_Start_Point[0], Adaptive_L_Start_Point[1], Adaptive_R_Start_Point[0], Adaptive_R_Start_Point[1], 0);
* 返回值：        无
*/

void Dir_Labyrinth_5(uint16 Break_Flag, uint8(*image)[Image_X], uint16(*l_line)[2], uint16(*r_line)[2], int16 *l_dir, int16 *r_dir, uint16 *l_stastic, uint16 *r_stastic, uint8 *x_meet, uint8 *y_meet,
                     uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 clip_value)
{
    uint8 j = 0;
 
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
//左边变量
    uint8  L_Center_Point[2] = {0};     //存放每次找到的XY坐标
    uint16 L_Data_Statics = 0;          //统计左边找到的边线点的个数
 
    uint8  L_Front_Value = 0;           //左侧 面向的前方点的灰度值
    uint8  L_Front_L_Value = 0;         //左侧 面向的左前方点的灰度值
 
    uint8  L_Dir = 0;                   //此参数用于转向
    uint8  L_Turn_Num = 0;              //记录转向次数，若中心点前后左右都是黑色像素，就会在一个点转向四次，记录到四次时退出循环防止卡死程序
    uint16 L_Pixel_Value_Sum = 0;       //中心点与周围24个点的像素值和
    float L_Thres = 0;                 //局部阈值,即L_Pixel_Value_Sum / 25
 
//右边变量
    uint8  R_Center_Point[2] = {0};     //存放每次找到的XY坐标
    uint16 R_Data_Statics = 0;          //统计右边找到的边线点的个数
 
    uint8  R_Front_Value = 0;           //右侧 面向的前方点的灰度值
    uint8  R_Front_R_Value = 0;         //右侧 面向的左前方点的灰度值
 
    uint8  R_Dir = 0;                   //此参数用于转向
    uint8  R_Turn_Num = 0;              //记录转向次数，若中心点前后左右都是黑色像素，就会在一个点转向四次，记录到四次时退出循环防止卡死程序
    uint16 R_Pixel_Value_Sum = 0;       //中心点与周围24个点的像素值和
    float R_Thres = 0;                 //局部阈值
 
//第一次更新坐标点  将找到的起点值传进来
    L_Center_Point[0] = l_start_x + 1;//x
    L_Center_Point[1] = l_start_y;//y
    R_Center_Point[0] = r_start_x - 1;//x
    R_Center_Point[1] = r_start_y;//y
 
    //开启方向迷宫循环
    while (Break_Flag--)
    {
         //左边
        //判定出死区后，挂出停止标志位，单侧爬线停止。
        if(L_Stop_Flag == 0)
        {
            l_line[L_Data_Statics][0] = L_Center_Point[0];  //找到的中心点X坐标计入左边线数组
            l_line[L_Data_Statics][1] = L_Center_Point[1];  //找到的中心点Y坐标计入左边线数组
 
            if(L_Data_Statics != 0)
            {
                switch(l_dir[L_Data_Statics - 1])  //下面这一坨可以根据上一个点的生长方向大幅优化爬线时间
                {
                    //从第二个点开始，第一个点的阈值要25个点全部加一遍
                    //当横向或纵向生长时，将原先的25次加法运算简化为十次计算
                    //当斜向生长时，将原先的25次加法计算简化为十八次运算
                    //可以画出图来，跟着代码走几遍，就可以很快速的理解
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
                for (j = 0; j < 25; j++)    //第一个阈值将25个点全部加一遍，后续点的阈值根据生长方向计算
                {
                    L_Pixel_Value_Sum += image[L_Center_Point[1] + Square_0[j][1]][L_Center_Point[0] + Square_0[j][0]];
                }
            }
 
            L_Thres = (L_Pixel_Value_Sum + Thres_Interfere) / Thres_Num_Interfere;   //阈值为25个点灰度值的平均值
//            L_Thres -= clip_value;              //将得到的灰度阈值减去一个经验值，用来优化判定
 
            //这里为反向平滑滤波（不知网上是否有这种算法，此处为本人原创），用于适应由较暗到（局部）高亮区域，或反之的光线情况。后续详细讲解原理
//            if(Thres_Filiter_Flag_1 == 1 || Thres_Filiter_Flag_2 == 1)
//            {
                if(L_Data_Statics > 3)
                {
                    L_Thres = L_Thres * 1.3f - L_Thres_Record[L_Data_Statics - 1] * 0.2f - L_Thres_Record[L_Data_Statics - 2] * 0.1f;
                }
//            }
            L_Thres_Record[L_Data_Statics] = (uint8_t)L_Thres;
            L_Data_Statics++;                   //每找到一个点统计个数+1
 
            L_Judge_Again:    //L_Judge_Again 与 goto 配合使用
            if(L_Stop_Flag == 0)
            {
                L_Front_Value = image[L_Center_Point[1] + L_Face_Dir[L_Dir][1]][L_Center_Point[0] + L_Face_Dir[L_Dir][0]];          //记录面向的前方点的灰度值
                L_Front_L_Value = image[L_Center_Point[1] + L_Face_Dir_L[L_Dir][1]][L_Center_Point[0] + L_Face_Dir_L[L_Dir][0]];    //记录面向的左前方点的灰度值
                if((float)L_Front_Value < L_Thres)     //面向的前方点是黑色
                {
                    L_Dir = (L_Dir + 1) % 4;    //需右转一次
                    L_Turn_Num ++;
                    if(L_Turn_Num == 4)        //死区处理
                    {
                        L_Stop_Flag = 1;       //当前后左右都是黑色时，进入死区，停止左侧爬线
                    }
                    goto L_Judge_Again;
                }
                else if((float)L_Front_L_Value < L_Thres)   //左前方点是黑色，前方点是白色
                {
                    L_Center_Point[0] += L_Face_Dir[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir[L_Dir][1];      //向前走一步
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir[L_Dir][0] * 3) - L_Face_Dir[L_Dir][1];
                    L_Turn_Num = 0;
                }
                else        //左前方和前方都是白色点
                {
                    L_Center_Point[0] += L_Face_Dir_L[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir_L[L_Dir][1];        //向左前方走一步
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir_L[L_Dir][0] * 3) - L_Face_Dir_L[L_Dir][1];
                    L_Dir = (L_Dir + 3) % 4;        //左转一次
                    L_Turn_Num = 0;
                }
                if(L_Data_Statics >= 5)     //O环处理，即转了一圈后回到原处，也是一种死区，当立即停止爬线
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
 
        //右侧与左侧同理，代码也类似，理解左侧后右侧就很简单
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
                && (My_ABS(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 1))        //两侧爬线相遇，退出循环，一张图像爬线结束
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //记录相遇点Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //记录相遇点X
                break;
            }
        }
        //有一侧存在死区时，对相遇点的判定放宽松一些，防止实际相遇但没有判定出，导致爬线紊乱的情况
        else
        {
            if ((My_ABS(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 3)
                && (My_ABS(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 3))        //两侧爬线相遇，退出循环，一张图像爬线结束
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //记录相遇点Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //记录相遇点X
                break;
            }
        }
    }
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
    *l_stastic = L_Data_Statics;    //记录左侧边线点个数
    *r_stastic = R_Data_Statics;    //记录右侧边线点个数
}

/**
* 函数功能：      由二维边线数组提取一维边线
* 特殊说明：      无
* 形  参：        uint16 l_total       //左侧二维边线点的个数
*                 uint16 r_total      //右侧二维边线点的个数
*                 uint8 start         //起始行（图像底部）
*                 uint8 end           //截止行（图像顶部）
*                 uint8 *l_border     //存储左侧一维边线的数组
*                 uint8 *r_border     //存储右侧一维边线的数组
*                 uint8(*l_line)[2]   //存储左侧二维边线的数组
*                 uint8(*r_line)[2]   //存储右侧二维边线的数组
*
* 示例：          Get_Border(L_Statics, R_Statics, Image_Y - 3, 2, L_Border, R_Border, L_Line, R_Line);
* 返回值：        无
*/
void Get_Border(uint16 l_total, uint16 r_total, uint8 start, uint8 end, uint8 *l_border, uint8 *r_border, uint8(*l_line)[2], uint8(*r_line)[2])
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < LCDH - 1; i++)
    {
        l_border[i] = 1;
        r_border[i] = LCDW - 2;     //右边线初始化放到最右边，左边线放到最左边，这样闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = start;
    //右边
    for (j = 0; j < r_total; j++)
    {
        if (r_line[j][1] == h)
        {
            r_border[h] = r_line[j][0];
        }
        else
        {
            continue;//每行只取一个点，没到下一行就不记录
        }
        h--;
        if (h == end)
        {
            break;//到最后一行退出
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
            continue;//每行只取一个点，没到下一行就不记录
        }
        h--;
        if (h == end)
        {
            break;//到最后一行退出
        }
    }
}

/**
* 函数功能：      提取阈值中的最大最小值,并求出阈值均值
* 特殊说明：      计算时间小于5us
* 形  参：        无
*
* 示例：          Thres_Record_Process（）；
* 返回值：        无
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
 
    for(i = 0; i < Adaptive_L_Statics; i += 2)      //间隔取值即可，减少计算量
    {
        if(Adaptive_L_Line[i][0] != 2 && Adaptive_L_Line[i][1] != 2)      //舍去位于黑框上的边线点阈值，“2”即左边线位于黑框上时的X坐标
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
        if(Adaptive_R_Line[i][0] != 91 && Adaptive_R_Line[i][1] != 2)       //与左侧同理
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
 
    if(Left_Temp_Value_1 == 0)      //当左侧边线全部位于边界上时，直接将阈值均值取0
    {
        L_Average_Thres = 0;
    }
    else        //求出左侧阈值均值
    {
        L_Average_Thres = (uint8)(Left_Temp_Value_2 / Left_Temp_Value_1);
    }
 
    if(Right_Temp_Value_1 == 0)     //与左侧同理
    {
        R_Average_Thres = 0;
    }
    else
    {
        R_Average_Thres = (uint8)(Right_Temp_Value_2 / Right_Temp_Value_1);
    }
 
    if(Image_Num <= 1)      //前两张图像直接求均值
    {
        Last_Adaptive_Thres_Average = (uint8)((L_Average_Thres + R_Average_Thres) / 2);
    }
    else
    {
        if(My_ABS(L_Average_Thres - R_Average_Thres) >= 40)       //当两侧边线的阈值均值相差过大时，进一步处理
        {
            if(My_ABS(L_Average_Thres - Last_Adaptive_Thres_Average) <= My_ABS(R_Average_Thres - Last_Adaptive_Thres_Average))      //选取两侧阈值均值最接近上次图像阈值均值的值作为此次的值
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
        else    //当两侧阈值均值相差不大时，直接求三者均值
        {
            Adaptive_Thres_Average = (uint8)((Last_Adaptive_Thres_Average + L_Average_Thres + R_Average_Thres) / 3);
            Last_Adaptive_Thres_Average = Adaptive_Thres_Average;
        }
    }
 
    //获得判断斑马线的阈值
    //Zbra_Thres = Adaptive_Thres_Average - 10;
 
    //获得起始点差比和的阈值
    if(Adaptive_Thres_Average >= 100 && Adaptive_Thres_Average <= 140)      //阈值均值落在100 - 140之间，说明图像亮度很合适，此时将差比和阈值设为20即可
    {
        Compare_Value = 20;
    }
    else if(Adaptive_Thres_Average < 100)
    {
        Compare_Value = 20 - (uint8)(((float)(100 - Adaptive_Thres_Average) / 60.0f) * 10.0f);      //当小于100时，适当拉低差比和阈值，使其在图像较暗的情况下更容易找出赛道边线
    }
    else if(Adaptive_Thres_Average > 140)
    {
        Compare_Value = 20 - (uint8)(((float)(Adaptive_Thres_Average - 140) / 60.0f) * 10.0f);      //当大于140时，适当拉低差比和阈值，使其在图像较亮的情况下更容易找出赛道边线
    }
     Black_Box_Value_1=(record_gray_value[0]+record_gray_value[1])/2;
    //求黑框灰度值
    Black_Box_Value = (uint8)(0.45f * (float)sqrt(Adaptive_L_Thres_Min * Adaptive_L_Thres_Min + Adaptive_R_Thres_Min * Adaptive_R_Thres_Min)) + (uint8)(0.1f * (float)Black_Box_Value_1);      
    //0.45f即0.9 * 0.5，0.9为权重，0.5可自己调节。Black_Box_Value_1为两侧爬线起点的灰度值均值（起点值灰度值为图像中赛道黑线的灰度值，不可为黑框灰度值，可以自己写算法处理下）
    //或者Black_Box_Value_1直接丢20 ~ 50之间的值即可
    
    
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
图像最后总处理
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