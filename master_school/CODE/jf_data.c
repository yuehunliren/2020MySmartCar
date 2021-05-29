#include "headfile.h"
#include "jf_data.h"

uint8   HistGram[255];

void GetHistGram1(uint8_t *image, uint32 len)//ͳ�ƻҶ�ֱ��ͼ
{
    int y;
    for (y = 0; y < 256; y++)
    {
        HistGram[y] = 0; //��ʼ���Ҷ�ֱ��ͼ
    }
    while(len--)
    HistGram[*image++]++; //ͳ��ÿ���Ҷ�ֵ�ĸ�����Ϣ
}
uint8_t OSTUThreshold1()//��ֵ������ֵ
{
    int16_t Y;
    uint32_t Amount = 0;
    uint32_t PixelBack = 0;
    uint32_t PixelIntegralBack = 0;
    uint32_t PixelIntegral = 0;
    int32_t PixelIntegralFore = 0;
    int32_t PixelFore = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    int16_t MinValue, MaxValue;
    uint8_t Threshold = 0;
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ
    if (MaxValue == MinValue)
    {
        return MaxValue;          // ͼ����ֻ��һ����ɫ
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // ͼ����ֻ�ж�����ɫ
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  ��������
    }
    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];    //ǰ�����ص���
        PixelFore = Amount - PixelBack;         //�������ص���
        OmegaBack = (double)PixelBack / Amount;//ǰ�����ذٷֱ�
        OmegaFore = (double)PixelFore / Amount;//�������ذٷֱ�
        PixelIntegralBack += HistGram[Y] * Y;  //ǰ���Ҷ�ֵ
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
        MicroBack = (double)PixelIntegralBack / PixelBack;//ǰ���ҶȰٷֱ�
        MicroFore = (double)PixelIntegralFore / PixelFore;//�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
        if (Sigma > SigmaB)//����������䷽��g
        {
            SigmaB = Sigma;
            Threshold = Y;
        }
    }
    return Threshold;
}
void uart_putbuff2(UARTN_enum uartn, uint8 *buff, uint32 len, uint8 tu)//ͼ���ֵ������
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
int regression(uint8 c, uint8 datw[])//��С��ֵ��
{

  int i=0,SumX=0,SumY=0;
  float SumUp=0,SumDown=0,avrX=0,avrY=0,B;

  for(i=0;i<c;i++)
  {
    SumX+=i;
    SumY+=datw[i];    //����Ϊ������ߵ�����
  }
  avrX=SumX/c;     //X��ƽ��ֵ
  avrY=SumY/c;     //Y��ƽ��ֵ
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
    //A=(SumY-B*SumX)/c;  //�ؾ�
    return B;  //����б��
}
