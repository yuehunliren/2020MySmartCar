#include "headfile.h"
#include "Motor.h"
#include "encoder.h"
#include "debug_usart.h"
#include "jf_data.h"

#define E_START 0              //׼��״̬
#define E_OK 1                 //�ɹ�
#define E_FRAME_HEADER_ERROR 2 //֡ͷ����
#define E_FRAME_RTAIL_ERROR 3  //֡β����

#define LINE_LEN 12 //���ݳ���
#define SLAMIDLINE 97     //NORMAL״̬�µ�������ֵ
#define MASMIDLINE 94      //HORIZONTAL״̬�µ�������ֵ
typedef enum
{
    NORMAL = 0,
    DRIVEIN,
    HORIZONTAL,
    DRIVEOUT,
    STOPBUFER,
    STOP,
} CarMode;                 //С������ģʽ
uint8 temp_buff[LINE_LEN]; //�������ڽ������ݵ�BUFF
vuint8 uart_flag;          //�������ݱ�־λ

int16 slave_encoder_LB,slave_encoder_RB,master_encoder_RF,master_encoder_LF;  //�ӻ���������ֵ,�ӻ��Һ������ֵ��������ǰ������ֵ��������ǰ������ֵ

int16 positionerr,slave_position;    //ת��ֵƫ�� ���ӻ�ת��ֵ

uint8 anaflag, startflag = 0; //���ݴ����־λ��С��������־λ
PID_Typedef PIDLF, PIDLB, PIDRB, PIDRF, PIDPos;
int16 LFTar, RFTar, LBTar, RBTar;
uint8 tag1[50], tag2[50], tag3[50], tag5 = 0, tag6 = 0, tag7 = 0, tag8 = 1, tag9 = 0;//tag1���ұ߽磬tag3��ֵ��tag4��ֵ(��ʱ�Ƴ�)��tag5����״̬��־λ��tag67����λ��־
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
//  @brief      ��ȡ�ӻ�����
//  @param      data            ��������
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void get_slave_data(uint8 data)
{
    static uint8 uart_num = 0;
    temp_buff[uart_num++] = data;

    if (1 == uart_num)
    {
        //���յ��ĵ�һ���ַ���Ϊ0xD8��֡ͷ����
        if (0xD8 != temp_buff[0])
        {
            uart_num = 0;
            uart_flag = E_FRAME_HEADER_ERROR;
        }
    }
    if (LINE_LEN == uart_num)
    {
        uart_flag = E_OK;
        //���յ����һ���ֽ�Ϊ0xEE
        if (0xEE == temp_buff[LINE_LEN - 1])
        {
            uart_flag = E_OK;
        }
        else //���յ����һ���ֽڲ���0xEE��֡β����
        {
            uart_flag = E_FRAME_RTAIL_ERROR;
        }
        uart_num = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����Э��Դӻ����͹��������ݣ��������ݽ���
//  @param      *line                           ���ڽ��յ�������BUFF
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
//  @brief      �ⲿ�ж�0�жϷ�����
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
        while (uart_flag != E_OK);                     //�ȴ���������
        uart_flag = E_START;      //��ձ�־λ
        data_analysis(temp_buff); //���ݽ���
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
//  @brief      ����1�жϷ�����
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
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); //������ڽ����жϱ�־λ
        dat = USART_ReceiveData(USART1);                //��ȡ��������
        get_slave_data(dat);                            //��ÿһ���ֽڵĴ������ݴ���temp_buff�С�
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����3�жϷ�����
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
        suc = uart_query(UART_3, &data); //��ȡ��������
        USART_ClearITPendingBit(USART3, USART_IT_RXNE); //������ڽ����жϱ�־λ
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
///����ͷ������
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
    for (i = MT9V03X_H - 2; i >= 0; i--)      //ȡ���ұ߽�
    {
        tag3[i] = 94;                          //ȡ����
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
        tag3[i] = (tag1[i] + tag2[i]) / 2; //ȡ���ߣ��ı�������
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
    board_init(); //��ر���
    uart_rx_irq(UART_3, ENABLE);
    nvic_init(USART3_IRQn, 1, 3, ENABLE); //���ߴ����ж�����
    oled_init();
    Motorinit(); //�����ʼ��
    Encoder_master_init();                                //������������ʼ��
//-------------------------------------------------------------------//

        PID_Init(&PIDLF,35.00, 1.30, 1.10);                  //��ǰ�����PID������ʼ��
        PID_Init(&PIDRF,37.00,1.50,1.35);                  //��ǰ�����PID������ʼ��
        PID_Init(&PIDRB, 38.00, 1.50, 1.10);                  //�������PID������ʼ��
        PID_Init(&PIDLB, 35.00, 1.65, 1.35);                  //�Һ�����PID������ʼ��
        PID_Init(&PIDPos, 2.5, 0, 2.6);                       //λ��PID��ʼ��

//--------------------------------------------------------------------//
    uart_init(UART_1, 460800, UART1_TX_A9, UART1_RX_A10); //�������֮���ͨ�Ŵ��ڳ�ʼ��
    uart_rx_irq(UART_1, ENABLE);                          //Ĭ����ռ���ȼ�0 �����ȼ�0
    nvic_init((IRQn_Type)(53 + UART_1), 0, 0, ENABLE);    //������3����ռ���ȼ�����Ϊ��ߣ������ȼ�����Ϊ���
    gpio_interrupt_init(B0, RISING, GPIO_INT_CONFIG); //B0Ϊ��������֮������ӡ������ش����ж�
    nvic_init(EXTI0_IRQn, 1, 2, ENABLE); //EXTI0���ȼ����ã���ռ���ȼ�0�������ȼ�1
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
                    //ʹ��������ʾ����������ԭʼͼ���С �Լ�������Ҫ��ʾ�Ĵ�С�Զ��������Ż��߷Ŵ���ʾ
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
