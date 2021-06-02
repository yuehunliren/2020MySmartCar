#include "headfile.h"
#include "Motor.h"
#include "encoder.h"
#include "debug_usart.h"
#include "jf_data.h"

#define E_START 0              // 准备状态
#define E_OK 1                 // 成功
#define E_FRAME_HEADER_ERROR 2 // 帧头错误
#define E_FRAME_RTAIL_ERROR 3  // 帧尾错误

#define LINE_LEN 13   // 数据长度
#define SLAMIDLINE 97 // NORMAL状态下得中线数值
#define MASMIDLINE 94 // HORIZONTAL状态下的中线数值
typedef enum
{
    NORMAL = 0,            // 正常行驶
    INBUFER,               // 刚开始换向前的缓冲
    DRIVEIN,               //In换向动作
    HORIZONTAL,            //横着行驶
    OUTBUFER,              // 二次换向
    DRIVEOUT,              // Out换向动作
    STOPBUFER,             // 停止前的缓冲
    STOP,                  // 停止
} CarMode;                 // 小车运行模式
uint8 temp_buff[LINE_LEN]; // 主机用于接收数据的BUFF
vuint8 uart_flag;          // 接收数据标志位

int16 slave_encoder_LB, slave_encoder_RB, master_encoder_RF, master_encoder_LF; // 从机左后编码器值,从机右后编码器值，主机右前编码器值，主机左前编码器值

int16 positionerr, slave_position; // 转角值偏差 ，从机转角值

uint8 anaflag, startflag = 0;                                                         // 数据处理标志位，小车启动标志位
PID_Typedef PIDLF, PIDLB, PIDRB, PIDRF, PIDPos;                                       // 4个轮子的PID参数，和位置环的PID参数
int16 LFTar, RFTar, LBTar, RBTar;                                                     // 4个轮子的目标速度
uint8 tag1[50], tag2[50], tag3[50], tag5 = 0, tag6 = 0, tag7 = 0, tag8 = 1, tag9 = 0; //tag1左右边界，tag3中值，tag4差值(暂时移除)，tag5赛道状态标志位，tag67超限位标志
int16 i, w, mov = 0;
int16 basespeed; // 竖向基础速度
uint32_t nortime = 500, hortime;
uint8_t STA_FLAG;
uint8_t norsta, horsta;
int16 Intime, Outtime; // 换向计数器
CarMode Mode = NORMAL;
// 清空换向前轮子的参数
void clearPID()
{
    PIDLF.output = 0;
    PIDRF.output = 0;
    PIDLB.output = 0;
    PIDRB.output = 0;
    PIDPos.output = 0;
    PIDPos.Kp = 3.3;
}
void clearPID1()
{
    PIDLF.output = 0;
    PIDRF.output = 0;
    PIDLB.output = 0;
    PIDRB.output = 0;
    PIDPos.output = 0;
    PIDPos.Kp = 4.0;
}
// 清空目标值
void clearTar()
{
    LFTar = 0;
    RFTar = 0;
    LBTar = 0;
    RBTar = 0;
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
    static uint8_t i, j = 0;
    if (line[1] == 0xB0)
        slave_encoder_LB = ((int16)line[2] << 8) | line[3];
    if (line[4] == 0xB1)
        slave_encoder_RB = ((int16)line[5] << 8) | line[6];
    slave_encoder_RB = -slave_encoder_RB;
    STA_FLAG = line[11];
    if (Mode == NORMAL)
    {
        if (line[7] == 0xB2)
            slave_position = line[8];
        if (line[9] == 0xf1 && nortime >= 750)
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
            Mode = INBUFER;
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
        if (tag5 == 0xf2 && hortime >= 750)
        {
            Mode = OUTBUFER;
            horsta = 0;
            hortime = 0;
            tag8 = 1;
            tag5 = 0;
        }
    }
    if (line[10] == 0xA2 && i == 0)
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
        while (uart_flag != E_OK); // 等待接收数据
        uart_flag = E_START;      // 清空标志位
        data_analysis(temp_buff); // 数据解析
        Encoder_master_get(&master_encoder_LF, &master_encoder_RF);
        if (slave_position > 0)
        {
            if (Mode == NORMAL)
                positionerr = slave_position - SLAMIDLINE;
            else if (Mode == HORIZONTAL)
                positionerr = slave_position - MASMIDLINE;
        }
        if (horsta)
            hortime++;
        if (norsta)
            nortime++;
        if (Mode == DRIVEIN)
        {
            Intime++;
            if (Intime >= 300)
            {
                clearPID();
                clearTar();
                Mode = HORIZONTAL;
                Intime = 0;
            }
        }
        else if (Mode == DRIVEOUT)
        {
            Outtime++;
            if (Outtime >= 260)
            {
                clearPID();
                clearTar();
                Mode = NORMAL;
                Outtime = 0;
            }
        }
        anaflag = 1;
        EXTI_ClearITPendingBit(EXTI_Line0);
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
        suc = uart_query(UART_3, &data);                //获取串口数据
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
                    Analyse_Data_Callback(&PIDLF, &basespeed);
                    i = 0;
                }
            }
        }
    }
}
///摄像头处理函数
void twwo(uint8 ow) //picture work
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
    for (i = MT9V03X_H - 2; i >= 0; i--) //取左右边界
    {
        tag3[i] = 94; //取中线
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
                if (tag6 == 0)
                    tag1[i] = 184;
                if (tag7 == 0)
                    tag2[i] = 4;
            }
            else
            {
                tag1[i] = tag1[i + 1];
                tag2[i] = tag2[i + 1];
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
        else
            tag8 = 0;
        tag3[i] = (tag1[i] + tag2[i]) / 2; //取中线，改变后面计算
        if (tag3[i] < 93)
            tag3[i] = tag3[i] + 2;
    }
}

int main(void)
{
    int16 basespeed1;
    uint8 yu1;
    int16 x;
    DisableGlobalIRQ();
    basespeed = 50;
    basespeed1 = 35;
    board_init();                                         //务必保留
//--------------初始化两块板子之间的串口连接和中断连接-----------------------//
    uart_init(UART_1, 460800, UART1_TX_A9, UART1_RX_A10); //两块板子之间的通信串口初始化
    uart_rx_irq(UART_1, ENABLE);                          //默认抢占优先级0 次优先级0
    nvic_init((IRQn_Type)(53 + UART_1), 0, 0, ENABLE);    //将串口3的抢占优先级设置为最高，次优先级设置为最高
    gpio_interrupt_init(B0, RISING, GPIO_INT_CONFIG);     //B0为两个板子之间的链接。上升沿触发中断
    nvic_init(EXTI0_IRQn, 1, 1, ENABLE);                  //EXTI0优先级配置，抢占优先级0，次优先级1
//-------------------无线串口的中断配置------------------------------------//
    uart_rx_irq(UART_3, ENABLE);
    nvic_init(USART3_IRQn, 1, 2, ENABLE); //无线串口中断配置
//------------------基本外设的初始化配置-----------------------------//
    oled_init();
    Motorinit();                         //电机初始化
    Encoder_master_init();               //主机编码器初始化
//---------------PID参数的初始化----------------------------------//
    PID_Init(&PIDLF,35.00, 1.30, 1.10);                  //左前电机的PID参数初始化
    PID_Init(&PIDRF,37.00,1.50,1.35);                  //右前电机的PID参数初始化
    PID_Init(&PIDRB, 38.00, 1.50, 1.10);                  //左后电机的PID参数初始化
    PID_Init(&PIDLB, 35.00, 1.65, 1.35);                  //右后电机的PID参数初始化
    PID_Init(&PIDPos,2.3, 0, 4.0);      //位置PID初始化
//------------摄像头的初始化------------------------------//
    mt9v03x_init();
    EnableGlobalIRQ(0);
    while (1)
    {
        Motor_Set(0, 0,0,0);
        oled_p6x8str(0, 0, "STA_FLAG:");
        oled_printf_int32(60,0,STA_FLAG,3);
        oled_p6x8str(0, 1, "KP:");
        oled_printf_float(30, 1, PIDLF.Kp, 3, 2);
        oled_p6x8str(0, 2, "KI:");
        oled_printf_float(30, 2, PIDLF.Ki, 3, 2);
        oled_p6x8str(0, 3, "KD:");
        oled_printf_float(30, 3, PIDLF.Kd, 3, 2);
        oled_p6x8str(0, 4, "Target:");
        oled_printf_int32(60, 4, basespeed, 3);
        oled_p6x8str(0, 5, "slave_position:");
        oled_printf_int32(90, 5, slave_position, 3);
        oled_p6x8str(0, 6, "positonerr:");
        oled_printf_int32(75, 6, positionerr, 3);
        oled_p6x8str(0, 7, "CarMode:");
        oled_printf_int32(60, 7, Mode, 1);
        while (startflag == 1)
        {

            oled_printf_int32(60, 7, Mode, 1);
            oled_printf_int32(75, 6, positionerr, 3);
            oled_printf_int32(90, 5, slave_position, 3);
            printf("%d,%d\r\n",slave_position,positionerr);
            if (nortime < 280)
            {
                basespeed = 15;
            }
            else
            {
                if (STA_FLAG)
                {
                basespeed = 30;
                PIDPos.Kp = 2.0;
                }
                else
                {
                basespeed = 20;
                PIDPos.Kp = 2.5;
                }
            }
            switch (Mode)
            {
            case NORMAL:
                Motor_Set(ceil(PIDLF.output),ceil(PIDRF.output),ceil(PIDLB.output),ceil(PIDRB.output));
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
            case INBUFER:
             for(x = 0;x<100;x++)
              {
                Motor_Set(0,0,0,0);
              }
              clearPID();
              tag8 = 1;
              norsta = 0;
              LFTar = 0;
              RFTar = 0;
              LBTar = -40;
              RBTar = 40;
              Mode = DRIVEIN;
              break;
            case DRIVEIN:
                Motor_Set(PIDLF.output, PIDRF.output, PIDLB.output, PIDRB.output);
                if (anaflag == 1)
                {
                    GetspeedPID(LFTar, master_encoder_LF, &PIDLF);
                    GetspeedPID(RFTar, master_encoder_RF, &PIDRF);
                    GetspeedPID(LBTar, slave_encoder_LB, &PIDLB);
                    GetspeedPID(RBTar, slave_encoder_RB, &PIDRB);
                  anaflag = 0;
                }
                break;
            case HORIZONTAL:
                Motor_Set(PIDLF.output, PIDRF.output, PIDLB.output, PIDRB.output);
                if (mt9v03x_finish_flag)
                {
                    mt9v03x_finish_flag = 0;
                    //使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
                    GetHistGram1(mt9v03x_image[0], MT9V03X_W * MT9V03X_H);
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
            case OUTBUFER:
                    for(x = 0;x<100;x++)
                    {
                        Motor_Set(0,0,0,0);
                    }
                    clearPID1();
                    norsta = 1;
                    nortime = 0;
                    LFTar = 30;
                    RFTar = -30;
                    LBTar = 30;
                    RBTar = -30;
                    Mode  = DRIVEOUT;
                break;
            case DRIVEOUT:
                Motor_Set(PIDLF.output, PIDRF.output, PIDLB.output, PIDRB.output);
                if (anaflag == 1)
                {
                    GetspeedPID(LFTar, master_encoder_LF, &PIDLF);
                    GetspeedPID(RFTar, master_encoder_RF, &PIDRF);
                    GetspeedPID(LBTar, slave_encoder_LB, &PIDLB);
                    GetspeedPID(RBTar, slave_encoder_RB, &PIDRB);
                  anaflag = 0;
                }
                mov = 1;
                break;
            case STOPBUFER:
                systick_delay_ms(200);
                Mode = STOP;
                break;
            case STOP:
                Motor_Set(PIDLF.output, PIDRF.output, PIDLB.output, PIDRB.output);
                if (anaflag == 1)
                {
                    LFTar = 0;
                    RFTar = 0;
                    LBTar = 0;
                    RBTar = 0;
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
