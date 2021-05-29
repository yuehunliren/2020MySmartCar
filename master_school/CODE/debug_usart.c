#include "debug_usart.h"
__IO uint8_t RxBuf[FRAME_LENTH]; // 接收缓存区
__IO MSG_TypeDef Msg;
extern uint8_t startflag;
/**
  * 函数功能: 计算检验和
  * 输入参数: Ptr:待计算校验和的数据起始地址 , Num:待计算的字节数
  * 返 回 值: 计算得到的校验和
  * 说    明: 计算检验和
  */
uint8_t CheckSum(uint8_t *Ptr,uint8_t Num )
{
  uint8_t Sum = 0;
  while(Num--)
  {
    Sum += *Ptr;
    Ptr++;
  }
  return Sum;
}
/**
  * 函数功能: 解析数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 将接收到的数据帧进行解析提取.
  */
void Analyse_Data_Callback(PID_Typedef *PID,int16 *Target)
{
    __IO uint8_t i = 0, j;
    char bemp;
    __IO uint8_t *Ptr = &RxBuf[FRAME_CHECK_BEGIN + 1];
    __IO char *Str = &Msg.data[0].Ch[0];
    Msg.Code = RxBuf[FRAME_CHECK_BEGIN]; // 第二位是指令码
    /* 利用结构体和数组的内存连续性,使用指针提取数据 */
    for (i = 0; i < (FRAME_CHECK_NUM - 1); i++)
    {
        *Str++ = *Ptr++;
    }
    switch (Msg.Code)
    {
    /* 设置PID参数 */
    case CODE_SETPID:
        //数据转换
        for (j = 0; j < 3; j++)
        {
            bemp = Msg.data[j].Ch[0];
            Msg.data[j].Ch[0] = Msg.data[j].Ch[3];
            Msg.data[j].Ch[3] = bemp;
            bemp = Msg.data[j].Ch[1];
            Msg.data[j].Ch[1] = Msg.data[j].Ch[2];
            Msg.data[j].Ch[2] = bemp;
        }
        PID->Kp = Msg.data[0].Float;
        PID->Ki = Msg.data[1].Float;
        PID->Kd = Msg.data[2].Float;
        break;
    /* 设置目标值 */
    case CODE_SETTGT:
        *Target = Msg.data[0].Int;
        break;
    case CODE_STARTMOTOR:
    {
        startflag = !startflag;
        break;
    }
    default:
        break;
    }
}
/**
  * 函数功能: 发送反馈值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 将反馈值发送到串口
  */
void Transmit_FB( int32_t Feedback)
{
  uint8_t TxBuf[FRAME_LENTH] ; // 发送缓存区
  uint8_t i = 0;
  for(i=0;i<FRAME_LENTH;i++)
  {
//    TxBuf[i] = FILL_VALUE;  // 参数填充 0x55
    TxBuf[i] = 0;
  }

  Msg.data[0].Int = Feedback;//反馈值 速度

  TxBuf[0] = FRAME_START;   // 帧头
//  TxBuf[1] = 0x80|CODE_SETTGT; // 指令码
  TxBuf[1] = 0x88;
  TxBuf[2] = Msg.data[0].Ch[0];
  TxBuf[3] = Msg.data[0].Ch[1];
  TxBuf[4] = Msg.data[0].Ch[2];
  TxBuf[5] = Msg.data[0].Ch[3];

  TxBuf[FRAME_CHECKSUM] = CheckSum((uint8_t*)&TxBuf[FRAME_CHECK_BEGIN], FRAME_CHECK_NUM);  // 计算校验和
  TxBuf[FRAME_LENTH-1] = FRAME_END;   // 加入帧尾

  uart_putbuff(UART_3, (uint8_t *)TxBuf, FRAME_LENTH);
}
