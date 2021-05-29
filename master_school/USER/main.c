#include "headfile.h"
#include "Motor.h"
#include "encoder.h"
#include "debug_usart.h"
#include "jf_data.h"

#define E_START 0              //准备状态
#define E_OK 1                 //成功
#define E_FRAME_HEADER_ERROR 2 //帧头错误
#define E_FRAME_RTAIL_ERROR 3  //帧尾错误

#define LINE_LEN 12 //数据长度
#define SLAMIDLINE 97     //NORMAL状态下得中线数值
#define MASMIDLINE 94      //HORIZONTAL状态下的中线数值
typedef enum
{
    NORMAL = 0,
    DRIVEIN,
    HORIZONTAL,
    DRIVEOUT,
    STOPBUFER,
    STOP,
} CarMode;                 //小车运行模式
uint8 temp_buff[LINE_LEN]; //主机用于接收数据的BUFF
vuint8 uart_flag;          //接收数据标志位

int16 slave_encoder_LB,slave_encoder_RB,master_encoder_RF,master_encoder_LF;  //从机左后编码器值,从机右后编码器值，主机右前编码器值，主机左前编码器值

int16 positionerr,slave_position;    //转角值偏差 ，从机转角值

uint8 anaflag, startflag = 0; //数据处理标志位，小车启动标志位
PID_Typedef PIDLF, PIDLB, PIDRB, PIDRF, PIDPos;
int16 LFTar, RFTar, LBTar, RBTar;
uint8 tag1[50], tag2[50], tag3[50], tag5 = 0, tag6 = 0, tag7 = 0, tag8 = 1, tag9 = 0;//tag1左右边界，tag3中值，tag4差值(暂时移除)，tag5赛道状态标志位，tag67超限位标志
int16 i, w, mov = 0;
int16 basespeed;
uint32_t nortime = 500,hortime;
uint8_t norsta,horsta;
CarMode Mode = NORMAL;
void clearPID()
{
    PIDLF.output = 0;
    PIDRF.output = 0;
    PIDLB.output = 0;
    PIDRB.output = 0;
    PIDPos.output = 0;
    PIDPos.Kp = 3.0;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取从机数据
//  @param      data            串口数据
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void get_slave_data(uint8 data)
{
    static uint8 uart_num = 0;
    temp_buff[uart_num++] = data;

    if (1 == uart_num)
    {
        //接收到的第一个字符不为0xD8，帧头错误
        if (0xD8 != temp_buff[0])
        {
            uart_num = 0;
            uart_flag = E_FRAME_HEADER_ERROR;
        }
    }
    if (LINE_LEN == uart_num)
    {
        uart_flag = E_OK;
        //接收到最后一个字节为0xEE
        if (0xEE == temp_buff[LINE_LEN - 1])
        {
            uart_flag = E_OK;
        }
        else //接收到最后一个字节不是0xEE，帧尾错误
        {
            uart_flag = E_FRAME_RTAIL_ERROR;
        }
        uart_num = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      根据协议对从机发送过来的数据，进行数据解析
//  @param      *line                           串口接收到的数据BUFF
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void data_analysis(uint8 *line)
{
    static uint8_t i,j = 0;
    if (line[1] == 0xB0)
    slave_encoder_LB = ((int16)line[2] << 8) | line[3];
    if (line[4] == 0xB1)
    slave_encoder_RB = ((int16)line[5] << 8) | line[6];
    slave_encoder_RB = -slave_encoder_RB;
    if(Mode == NORMAL)
    {
        if (line[7] == 0xB2)
        slave_position = line[8];
        if (line[9] == 0xf1 && nortime >= 200)
        {
            j++;
        }
        else
        {
            j = 0;
        }
        if (j == 3)
        {
           j = 0;
           horsta = 1;
           Mode = DRIVEIN;
        }
        if (line[9] == 0xf4)
        {
            if (mov == 1)
            {
            Mode = STOPBUFER;
            clearPID();
            }
        }
    }
    if (Mode == HORIZONTAL)
    {
     slave_position = tag3[30];
     if (tag5 == 0xf2 && hortime >= 300)
     {
         Mode = DRIVEOUT;
         horsta = 0;
         hortime = 0;
         tag8 = 1;
         tag5 = 0;
     }
    }
    if(line[10] == 0xA2 && i == 0)
    {
        startflag = 1;
        i = 1;
    }
    else if (line[10] == 0xA1 && i == 1)
    {
        startflag = 0;
        i = 0;
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      外部中断0中断服务函数
//  @param      void
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void EXTI0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI0_IRQHandler(void)
{
    if (SET == EXTI_GetITStatus(EXTI_Line0))
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
        while (uart_flag != E_OK);                     //等待接收数据
        uart_flag = E_START;      //清空标志位
        data_analysis(temp_buff); //数据解析
        Encoder_master_get(&master_encoder_LF, &master_encoder_RF);
        if (slave_position > 0)
        {
            if(Mode == NORMAL)
            positionerr = slave_position - SLAMIDLINE;
            else if(Mode == HORIZONTAL)
            positionerr = slave_position - MASMIDLINE;
        }
        if (horsta)
        hortime++;
        if(norsta)
        nortime++;
        anaflag = 1;
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      串口1中断服务函数
//  @param      void
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)

{
    uint8 dat;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); //清除串口接收中断标志位
        dat = USART_ReceiveData(USART1);                //获取串口数据
        get_slave_data(dat);                            //将每一个字节的串口数据存入temp_buff中。
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      串口3中断服务函数
//  @param      void
//  @return     void
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t data, suc;
        static uint8_t i = 0;
        suc = uart_query(UART_3, &data); //获取串口数据
        USART_ClearITPendingBit(USART3, USART_IT_RXNE); //清除串口接收中断标志位
        if (suc)
        {
            RxBuf[i++] = data;
        }
        if (i == FRAME_LENTH)
        {
            if (RxBuf[0] != FRAME_START)
            {
                i = 0;
                return;
            }
            if (RxBuf[FRAME_LENTH - 1] == FRAME_END)
            {
                if (CheckSum((uint8_t *)&RxBuf[FRAME_CHECK_BEGIN], FRAME_CHECK_NUM) != RxBuf[FRAME_CHECKSUM])
                {
                    i = 0;
                    Msg.Code = 0;
                    return;
                }
                else
                {
                    Analyse_Data_Callback(&PIDLB,&basespeed);
                    i = 0;
                }
            }
        }
    }
}
///摄像头处理函数
void twwo(uint8 ow)//picture work
{
    uint8 data2[MT9V03X_H][MT9V03X_W];
    uint8 max[2];

    tag8 = 1;
    max[0] = 184;
    max[1] = 4;
    for (i = 0; i < MT9V03X_H; i++)
    {
        for (w = 0; w < MT9V03X_W; w++)
        {
            if (mt9v03x_image[i][w] > ow)
            {
                data2[i][w] = 1;
            }
            else
            {
                data2[i][w] = 0;
            }
        }
    }
    tag3[30] = 94;
    tag5 = 0;
    tag1[49] = 184;
    tag2[49] = 4;
    for (i = MT9V03X_H - 2; i >= 0; i--)      //取左右边界
    {
        tag3[i] = 94;                          //取中线
        tag1[i] = 138;
        tag2[i] = 50;
        tag6 = 0;
        tag7 = 0;
        for (w = tag3[i]; w < MT9V03X_W - 1; w++)
        {
            if (data2[i][w] == 0)
            {
                tag1[i] = w;
                tag6 = 1;
                break;
            }
        }
        for (w = tag3[i]; w >= 0; w--)
        {
            if (data2[i][w] == 0)
            {
                tag2[i] = w;
                tag7 = 1;
                break;
            }
        }
        if (tag6 == 0 || tag7 == 0)
                {
            if (i < 20 && tag8 == 0 && (tag6 == 0 && tag7 == 0))
                        {
                            tag5 = 0xf2;
                            tag9 = 0;
                        }
                    if (i > 35)
                    {
                        if (tag6 == 0) tag1[i] = 184;
                        if (tag7 == 0) tag2[i] = 4;
                    }
                    else
                    {
                        tag1[i] = tag1[i+1];
                        tag2[i] = tag2[i+1];
                    }
                        if (tag6 == 0 && tag7 == 0)
                        {
                            tag1[i] = 184;
                            tag2[i] = 4;
                        }
                    //tag1[i] = 98;
                    //tag2[i] = 98;
                        if (max[0] > tag1[i] && i > 30)
                        {
                            max[0] = tag1[i];
                        }
                        if (max[1] < tag2[i] && i > 30)
                        {
                            max[1] = tag2[i];
                        }
                        if (i == 30)
                        {
                            tag1[i] = max[0];
                            tag2[i] = max[1];
                        }
                }
        else tag8 = 0;
        tag3[i] = (tag1[i] + tag2[i]) / 2; //取中线，改变后面计算
        if (tag3[i] < 93) tag3[i] = tag3[i] + 2;
    }
}

int main(void)

{
    int16 basespeed1;
    uint8 yu1;
    DisableGlobalIRQ();
    basespeed =  100;
    basespeed1 = 70;
    board_init(); //务必保留
    uart_rx_irq(UART_3, ENABLE);
    nvic_init(USART3_IRQn, 1, 3, ENABLE); //无线串口中断配置
    oled_init();
    Motorinit(); //电机初始化
    Encoder_master_init();                                //主机编码器初始化
//-------------------------------------------------------------------//

        PID_Init(&PIDLF,35.00, 1.30, 1.10);                  //左前电机的PID参数初始化
        PID_Init(&PIDRF,37.00,1.50,1.35);                  //右前电机的PID参数初始化
        PID_Init(&PIDRB, 38.00, 1.50, 1.10);                  //左后电机的PID参数初始化
        PID_Init(&PIDLB, 35.00, 1.65, 1.35);                  //右后电机的PID参数初始化
        PID_Init(&PIDPos, 2.5, 0, 2.6);                       //位置PID初始化

//--------------------------------------------------------------------//
    uart_init(UART_1, 460800, UART1_TX_A9, UART1_RX_A10); //两块板子之间的通信串口初始化
    uart_rx_irq(UART_1, ENABLE);                          //默认抢占优先级0 次优先级0
    nvic_init((IRQn_Type)(53 + UART_1), 0, 0, ENABLE);    //将串口3的抢占优先级设置为最高，次优先级设置为最高
    gpio_interrupt_init(B0, RISING, GPIO_INT_CONFIG); //B0为两个板子之间的链接。上升沿触发中断
    nvic_init(EXTI0_IRQn, 1, 2, ENABLE); //EXTI0优先级配置，抢占优先级0，次优先级1
    mt9v03x_init();
    EnableGlobalIRQ(0);
    while (1)
    {
        Motor_Set(0,0,0,0);
        oled_p6x8str(0, 5, "slave_position:");
        oled_printf_int32(90, 5, slave_position, 3);
        oled_p6x8str(0, 6, "positonerr:");
        oled_printf_int32(75, 6, positionerr, 3);
        oled_p6x8str(0, 0, "Stop");
        oled_p6x8str(0, 1, "KP:");
        oled_printf_float(30, 1, PIDLB.Kp, 3, 2);
        oled_p6x8str(0, 2, "KI:");
        oled_printf_float(30, 2, PIDLB.Ki, 3, 2);
        oled_p6x8str(0, 3, "KP:");
        oled_printf_float(30, 3, PIDLB.Kd, 3, 2);
        oled_p6x8str(0, 4, "Target:");
        oled_printf_int32(60, 4,basespeed, 3);
        oled_p6x8str(0, 7, "CarMode:");
        oled_printf_int32(60, 7,Mode, 1);
        while (startflag == 1)
        {
            oled_printf_int32(60, 7,Mode, 1);
            oled_printf_int32(75, 6, positionerr, 3);
            oled_printf_int32(90, 5, slave_position, 3);
            printf("%d,%d\r\n",slave_position,positionerr);
            if (nortime < 250)
            {
                basespeed =  30;
            }
            else
            {
                basespeed = 80;
            }
            switch (Mode)
            {
            case NORMAL:
              Motor_Set(PIDLF.output, PIDRF.output, PIDLB.output, PIDRB.output);
                if (anaflag == 1)
                {
                    GetposPID(0.0, positionerr, &PIDPos);
                    LFTar = basespeed - floor(PIDPos.output);
                    RFTar = basespeed + floor(PIDPos.output);
                    LBTar = basespeed - floor(PIDPos.output);
                    RBTar = basespeed + floor(PIDPos.output);
                    GetspeedPID(LFTar, master_encoder_LF, &PIDLF);
                    GetspeedPID(RFTar, master_encoder_RF, &PIDRF);
                    GetspeedPID(LBTar, slave_encoder_LB, &PIDLB);
                    GetspeedPID(RBTar, slave_encoder_RB, &PIDRB);
                  anaflag = 0;
                }
                break;
            case DRIVEIN:
                Motor_Set(0, 0, 0, 0);
                systick_delay_ms(500);
                Motor_Set(-1700, 1700, -1700,1700);
                systick_delay_ms(380);
                Motor_Set(0,0,0,0);
                systick_delay_ms(100);
                clearPID();
                tag8 = 1;
                norsta = 0;
                Mode = HORIZONTAL;
                break;
            case HORIZONTAL:
                Motor_Set(PIDLF.output, PIDRF.output, PIDLB.output, PIDRB.output);
                if(mt9v03x_finish_flag)
                {
                    mt9v03x_finish_flag = 0;
                    //使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
                    GetHistGram1(mt9v03x_image[0], MT9V03X_W*MT9V03X_H);
                    yu1 = OSTUThreshold1();
                    twwo(yu1);
                }
                  if (anaflag == 1)
                  {
                      GetposPID(0.0, positionerr, &PIDPos);
                      LFTar = basespeed1 - PIDPos.output;
                      RFTar = -basespeed1 + PIDPos.output;
                      LBTar = -basespeed1 - PIDPos.output;
                      RBTar = basespeed1 + PIDPos.output;
                      GetspeedPID(LFTar, master_encoder_LF, &PIDLF);
                      GetspeedPID(RFTar, master_encoder_RF, &PIDRF);
                      GetspeedPID(LBTar, slave_encoder_LB, &PIDLB);
                      GetspeedPID(RBTar, slave_encoder_RB, &PIDRB);
                    anaflag = 0;
                  }
                break;
            case DRIVEOUT:
                Motor_Set(0, 0, 0, 0);
                systick_delay_ms(100);
                Motor_Set(1700, -1700, 1700, -1700);
                systick_delay_ms(380);
                Motor_Set(0,0,0,0);
                systick_delay_ms(100);
                clearPID();
                norsta = 1;
                nortime = 0;
                mov = 1;
                Mode = NORMAL;
                break;
            case STOPBUFER:
                systick_delay_ms(500);
                Mode = STOP;
                break;
            case STOP:
                Motor_Set(PIDLF.output, PIDRF.output, PIDLB.output, PIDRB.output);
                  if (anaflag == 1)
                  {
                      LFTar =0;
                      RFTar =0;
                      LBTar =0;
                      RBTar =0;
                      GetspeedPID(LFTar, master_encoder_LF, &PIDLF);
                      GetspeedPID(RFTar, master_encoder_RF, &PIDRF);
                      GetspeedPID(LBTar, slave_encoder_LB, &PIDLB);
                      GetspeedPID(RBTar, slave_encoder_RB, &PIDRB);
                    anaflag = 0;
                  }
                break;
            default:
                break;
            }
        }
    }
}
