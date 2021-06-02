#ifndef __DEBUG_USART_H__
#define __DEBUG_USART_H__
#include "headfile.h"
#include "Motor.h"
/* 类型定义 ----------------------------------------------------------------- */

/* 串口接收状态类型定义 */
typedef enum{
  Rx_IDLE,
  Rx_ING,
  Rx_END
}UartRxState_TypeDef;

/* 数据格式化转换 */
typedef union {
  char Ch[4];
  float Float;
  int32_t Int;
}Format_UnionTypedef;

/* 通信协议有效数据 */
typedef struct {
  uint8_t  Code ;  	
  Format_UnionTypedef data[3];//数据帧有3个参数
}MSG_TypeDef;
// 协议相关定义
#define FRAME_LENTH               16     // 指令长度
#define FRAME_START               (uint8_t)0xAA   // 协议帧开始
#define FRAME_END                 '/'    // 协议帧结束
#define FRAME_CHECK_BEGIN          (uint8_t)1     // 校验码开始的位置 RxBuf[1]
#define FRAME_CHECKSUM            (uint8_t)14     // 校验码的位置   RxBuf[14]
#define FRAME_CHECK_NUM           (uint8_t)13     // 需要校验的字节数
#define FILL_VALUE                (uint8_t)0x55   // 填充值
#define CODE_SETPID               (uint8_t)0x07   // 设置PID参数
#define CODE_SETTGT               (uint8_t)0x08   // 设置目标值
#define CODE_RESET                (uint8_t)0x09   // 复位重启
#define CODE_STARTMOTOR           (uint8_t)0x0A   // 启动电机

/* 扩展变量 ------------------------------------------------------------------*/
extern __IO uint8_t RxBuf[FRAME_LENTH] ; // 接收缓存区
extern __IO MSG_TypeDef Msg;



uint8_t CheckSum(uint8_t *Ptr,uint8_t Num );
void Analyse_Data_Callback(PID_Typedef *PID,int16 *Target);
void Transmit_FB( int32_t Feedback);
#endif // __DEBUG_USART_H__
