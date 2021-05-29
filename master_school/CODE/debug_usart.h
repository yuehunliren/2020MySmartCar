#ifndef __DEBUG_USART_H__
#define __DEBUG_USART_H__
#include "headfile.h"
#include "Motor.h"
/* ���Ͷ��� ----------------------------------------------------------------- */

/* ���ڽ���״̬���Ͷ��� */
typedef enum{
  Rx_IDLE,
  Rx_ING,
  Rx_END
}UartRxState_TypeDef;

/* ���ݸ�ʽ��ת�� */
typedef union {
  char Ch[4];
  float Float;
  int32_t Int;
}Format_UnionTypedef;

/* ͨ��Э����Ч���� */
typedef struct {
  uint8_t  Code ;  	
  Format_UnionTypedef data[3];//����֡��3������
}MSG_TypeDef;
// Э����ض���
#define FRAME_LENTH               16     // ָ���
#define FRAME_START               (uint8_t)0xAA   // Э��֡��ʼ
#define FRAME_END                 '/'    // Э��֡����
#define FRAME_CHECK_BEGIN          (uint8_t)1     // У���뿪ʼ��λ�� RxBuf[1]
#define FRAME_CHECKSUM            (uint8_t)14     // У�����λ��   RxBuf[14]
#define FRAME_CHECK_NUM           (uint8_t)13     // ��ҪУ����ֽ���
#define FILL_VALUE                (uint8_t)0x55   // ���ֵ
#define CODE_SETPID               (uint8_t)0x07   // ����PID����
#define CODE_SETTGT               (uint8_t)0x08   // ����Ŀ��ֵ
#define CODE_RESET                (uint8_t)0x09   // ��λ����
#define CODE_STARTMOTOR           (uint8_t)0x0A   // �������

/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint8_t RxBuf[FRAME_LENTH] ; // ���ջ�����
extern __IO MSG_TypeDef Msg;



uint8_t CheckSum(uint8_t *Ptr,uint8_t Num );
void Analyse_Data_Callback(PID_Typedef *PID,int16 *Target);
void Transmit_FB( int32_t Feedback);
#endif // __DEBUG_USART_H__
