#include "debug_usart.h"
__IO uint8_t RxBuf[FRAME_LENTH]; // ���ջ�����
__IO MSG_TypeDef Msg;
extern uint8_t startflag;
/**
  * ��������: ��������
  * �������: Ptr:������У��͵�������ʼ��ַ , Num:��������ֽ���
  * �� �� ֵ: ����õ���У���
  * ˵    ��: ��������
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
  * ��������: ��������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: �����յ�������֡���н�����ȡ.
  */
void Analyse_Data_Callback(PID_Typedef *PID,int16 *Target)
{
    __IO uint8_t i = 0, j;
    char bemp;
    __IO uint8_t *Ptr = &RxBuf[FRAME_CHECK_BEGIN + 1];
    __IO char *Str = &Msg.data[0].Ch[0];
    Msg.Code = RxBuf[FRAME_CHECK_BEGIN]; // �ڶ�λ��ָ����
    /* ���ýṹ���������ڴ�������,ʹ��ָ����ȡ���� */
    for (i = 0; i < (FRAME_CHECK_NUM - 1); i++)
    {
        *Str++ = *Ptr++;
    }
    switch (Msg.Code)
    {
    /* ����PID���� */
    case CODE_SETPID:
        //����ת��
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
    /* ����Ŀ��ֵ */
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
  * ��������: ���ͷ���ֵ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ������ֵ���͵�����
  */
void Transmit_FB( int32_t Feedback)
{
  uint8_t TxBuf[FRAME_LENTH] ; // ���ͻ�����
  uint8_t i = 0;
  for(i=0;i<FRAME_LENTH;i++)
  {
//    TxBuf[i] = FILL_VALUE;  // ������� 0x55
    TxBuf[i] = 0;
  }

  Msg.data[0].Int = Feedback;//����ֵ �ٶ�

  TxBuf[0] = FRAME_START;   // ֡ͷ
//  TxBuf[1] = 0x80|CODE_SETTGT; // ָ����
  TxBuf[1] = 0x88;
  TxBuf[2] = Msg.data[0].Ch[0];
  TxBuf[3] = Msg.data[0].Ch[1];
  TxBuf[4] = Msg.data[0].Ch[2];
  TxBuf[5] = Msg.data[0].Ch[3];

  TxBuf[FRAME_CHECKSUM] = CheckSum((uint8_t*)&TxBuf[FRAME_CHECK_BEGIN], FRAME_CHECK_NUM);  // ����У���
  TxBuf[FRAME_LENTH-1] = FRAME_END;   // ����֡β

  uart_putbuff(UART_3, (uint8_t *)TxBuf, FRAME_LENTH);
}
