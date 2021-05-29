#include "headfile.h"
#include "jf_data.h"

uint8   HistGram[255];

void GetHistGram1(uint8_t *image, uint32 len)//统计灰度直方图
{
    int y;
    for (y = 0; y < 256; y++)
    {
        HistGram[y] = 0; //初始化灰度直方图
    }
    while(len--)
    HistGram[*image++]++; //统计每个灰度值的个数信息
}
uint8_t OSTUThreshold1()//二值化求阈值
{
    int16_t Y;
    uint32_t Amount = 0;
    uint32_t PixelBack = 0;
    uint32_t PixelIntegralBack = 0;
    uint32_t PixelIntegral = 0;
    int32_t PixelIntegralFore = 0;
    int32_t PixelFore = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int16_t MinValue, MaxValue;
    uint8_t Threshold = 0;
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值
    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }
    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];    //前景像素点数
        PixelFore = Amount - PixelBack;         //背景像素点数
        OmegaBack = (double)PixelBack / Amount;//前景像素百分比
        OmegaFore = (double)PixelFore / Amount;//背景像素百分比
        PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
        MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
        MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
        if (Sigma > SigmaB)//遍历最大的类间方差g
        {
            SigmaB = Sigma;
            Threshold = Y;
        }
    }
    return Threshold;
}
void uart_putbuff2(UARTN_enum uartn, uint8 *buff, uint32 len, uint8 tu)//图像二值化发送
{
    uint8 a = 0xff, b = 0x00;

    while(len--)
    {
        if (*buff++ > tu)
        {
            uart_putchar(uartn, a);
        }
        else
        {
            uart_putchar(uartn, b);
        }
    }
}
int regression(uint8 c, uint8 datw[])//最小二值化
{

  int i=0,SumX=0,SumY=0;
  float SumUp=0,SumDown=0,avrX=0,avrY=0,B;

  for(i=0;i<c;i++)
  {
    SumX+=i;
    SumY+=datw[i];    //这里为存放中线的数组
  }
  avrX=SumX/c;     //X的平均值
  avrY=SumY/c;     //Y的平均值
  SumUp=0;
  SumDown=0;
  for(i=0;i<c;i++)
  {
    SumUp+=(datw[i]-avrY)*(i-avrX);
    SumDown+=(i-avrX)*(i-avrX);
  }
  if(SumDown==0)
    B=0;
  else
    B=(int)(SumUp/SumDown);
    //A=(SumY-B*SumX)/c;  //截距
    return B;  //返回斜率
}
