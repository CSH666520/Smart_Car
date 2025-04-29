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