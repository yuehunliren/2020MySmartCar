#include "headfile.h"
#include "jf_data.h"
#include "multi_button.h"

#define LINE_LEN                13             //���ݳ���
#define symbals 10
#define chazhi 30

#define white 100

/*��Ӷ������*/
uint8 BEN_FLAG,STA_FLAG;






uint8 temp_buff[LINE_LEN];                      //�ӻ���������������BUFF

int16 slave_encoder_RB;                       //�ӻ��Һ������ֵ
int16 slave_encoder_LB;                      //�ӻ���ǰ������ֵ
//int16 slave_position;                           //�ӻ�ת��ֵ
uint8 tag1[50], tag2[50], tag3[50], tag5 = 0, tag6 = 0, tag7 = 0, tag8 = 1, tag9 = 0;//tag1���ұ߽磬tag3��ֵ��tag4��ֵ(��ʱ�Ƴ�)��tag5����״̬��־λ��tag67����λ��־
int16 i, w;

struct Button button1;
uint8_t buflag = 0;
uint8_t read_c14gpio(void)
{
    return gpio_get(C14);
}
void btn1_press_up_Handler(void* btn)
{
   buflag = !buflag;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����������
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void get_sensor_data(void)
{
    slave_encoder_LB = timer_quad_get(TIMER_3);      //B4-B5������ȡֵ
    slave_encoder_RB = timer_quad_get(TIMER_2);       //A15-B3������ȡֵ
    timer_quad_clear(TIMER_2);                          //��ռ�����
    timer_quad_clear(TIMER_3);                          //��ռ�����

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����Э�鴦������
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void process_data(void)
{
    temp_buff[0] = 0xD8;                         //֡ͷ

    temp_buff[1] = 0xB0;                         //������
    temp_buff[2] = slave_encoder_RB>>8;        //���ݸ�8λ
    temp_buff[3] = slave_encoder_RB&0xFF;      //���ݵ�8λ

    temp_buff[4] = 0xB1;                         //������
    temp_buff[5] = slave_encoder_LB>>8;       //���ݸ�8λ
    temp_buff[6] = slave_encoder_LB&0xFF;     //���ݵ�8λ

    temp_buff[7] = 0xB2;                         //������
    temp_buff[8] = tag3[symbals];            //����1
    //temp_buff[8] = 0x05;
    temp_buff[9] = tag5;                     //����2
    if (buflag == 0)
    {
        temp_buff[10] = 0xA1;
    }
    else
    {
        temp_buff[10] = 0xA2;
    }
    temp_buff[11] = STA_FLAG;
    temp_buff[12] = 0xEE;                        //֡β
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ��4�жϷ�����
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        gpio_set(B0,1);                           //A0��������

        get_sensor_data();                          //��ȡ���������ݡ�
        process_data();                             //����Э�鴦�����ݣ�������temp_buff�С�
        uart_putbuff(UART_1, temp_buff, LINE_LEN);  //ͨ������1�����ݷ��ͳ�ȥ��
        button_ticks();
        gpio_set(B0, 0);                         //A0��������
    }
}
void twwo(uint8 ow)
{
    uint8 data2[MT9V03X_H][MT9V03X_W];

    tag8 = 1;
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
    tag5 = 0;
    tag1[i] = 184;
    tag2[i] = 10;
    for (i = MT9V03X_H - 2; i >= 0; i--)      //ȡ���ұ߽�
    {
        tag3[i] = 97;                          //ȡ����
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
            /*if (i == 35  && tag8 == 0 && (tag6 == 0 && tag7 == 0))
            {
                tag5 = 0xf1;
                tag9 = 0;
            }
                tag1[i] = tag1[i+1];
                tag2[i] = tag2[i+1];
            tag1[i] = 94;
            tag2[i] = 94;
            if (i > 35)
                    {
                        tag1[i] = 94;
                        tag2[i] = 94;
                    }*/
            if (i == 30 && tag8 == 0 && (tag6 == 0 && tag7 == 0))
                                    {
                                        tag5 = 0xf1;
                                        tag9 = 0;
                                    }
                                if (i > 35)
                                {
                                    if (tag6 == 0) tag1[i] = 184;
                                    if (tag7 == 0) tag2[i] = 10;
                                }
                                else
                                {
                                    tag1[i] = tag1[i+1];
                                    tag2[i] = tag2[i+1];
                                }
                                    if (tag6 == 0 && tag7 == 0)
                                    {
                                        tag1[i] = 184;
                                        tag2[i] = 10;
                                    }
        }
        else tag8 = 0;
        tag3[i] = (tag1[i] + tag2[i]) / 2;               //ȡ���ߣ��ı�������
    }
}
void whu(void)
{
    tag8 = 1;
        tag5 = 0;
        tag1[49] = 184;
        tag2[49] = 10;
        for (i = MT9V03X_H - 2; i >= 0; i--)      //ȡ���ұ߽�
        {
            tag3[i] = 97;                          //ȡ����
            tag1[i] = 184;
            tag2[i] = 10;
            tag6 = 0;
            tag7 = 0;
            for (w = tag3[i]; w < MT9V03X_W - 1; w++)
            {
                if ((mt9v03x_image[i][w] - mt9v03x_image[i][w-1] > chazhi && mt9v03x_image[i][w] > mt9v03x_image[i][w-1]) || (mt9v03x_image[i][w-1] - mt9v03x_image[i][w] > chazhi && mt9v03x_image[i][w] < mt9v03x_image[i][w-1]))
                {
                    tag1[i] = w;
                    tag6 = 1;
                    break;
                }
                if (mt9v03x_image[i][w] < white)
                {
                    tag6 = 1;
                }
            }
            for (w = tag3[i]; w >= 0; w--)
            {
                if ((mt9v03x_image[i][w] - mt9v03x_image[i][w+1] > chazhi && mt9v03x_image[i][w] > mt9v03x_image[i][w+1]) || (mt9v03x_image[i][w+1] - mt9v03x_image[i][w] > chazhi && mt9v03x_image[i][w] < mt9v03x_image[i][w+1]))
                {
                    tag2[i] = w;
                    tag7 = 1;
                    break;
                }
                if (mt9v03x_image[i][w] < white)
                {
                    tag7 = 1;
                }
            }
            if (tag6 == 0 || tag7 == 0)
            {
                /*if (i == 35  && tag8 == 0 && (tag6 == 0 && tag7 == 0))
                {
                    tag5 = 0xf1;
                    tag9 = 0;
                }
                    tag1[i] = tag1[i+1];
                    tag2[i] = tag2[i+1];
                tag1[i] = 94;
                tag2[i] = 94;
                if (i > 35)
                        {
                            tag1[i] = 94;
                            tag2[i] = 94;
                        }*/
                if (i == 30 && tag8 == 0 && (tag6 == 0 && tag7 == 0))
                                        {
                                            tag5 = 0xf1;
                                            tag9 = 0;
                                        }
                                    if (i > 35)
                                    {
                                        if (tag6 == 0) tag1[i] = 184;
                                        if (tag7 == 0) tag2[i] = 10;
                                    }
                                    else
                                    {
                                        tag1[i] = tag1[i+1];
                                        tag2[i] = tag2[i+1];
                                    }
                                        if (tag6 == 0 && tag7 == 0)
                                        {
                                            tag1[i] = 184;
                                            tag2[i] = 10;
                                        }
            }
            else tag8 = 0;
            tag3[i] = (tag1[i] + tag2[i]) / 2;               //ȡ���ߣ��ı�������
            if (tag1[i] < 107 || tag2[i] > 87)
            {
                tag3[i] = 97;
            }
            if (i == 30)
                    {
                    if (tag1[i] < 107 || tag2[i] > 87)
                    {
                        tag5 = 0xf4;
                    }
                    }
        }




/*��Ӵ���*/


                int32 a = 0;
//               for (int16 m = 0; m < 50; m++)
//               {
//                   if (abs(tag3[m] - (188 / 2)) < 20)
//                   {
//                       a++;
//                   }
//               }

               a = abs(tag3[1]-tag3[48]);
               if (a > 5)
               {
                   BEN_FLAG = 1;
                   STA_FLAG = 0;
               }
               else
               {
                   BEN_FLAG = 0;
                   STA_FLAG = 1;
               }


}
int main(void)
{
    DisableGlobalIRQ();
    board_init();           //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    mt9v03x_init();
//--------------------�������֮������ӳ�ʼ��----------------//
    uart_init(UART_1, 460800, UART1_TX_A9, UART1_RX_A10);
    gpio_init(B0, GPO, 0, GPIO_PIN_CONFIG);  //B0Ϊ��������֮�������
    timer_pit_interrupt_ms(TIMER_4, 5);      //��ʱ��4 5ms��ʱ���ж�
//-------------------������ʼ��--------------------//
    gpio_init(C14, GPI, 1,GPIO_INT_CONFIG);
    button_init(&button1, read_c14gpio,0);
    button_attach(&button1, PRESS_UP, btn1_press_up_Handler);
//------------------�������ĳ�ʼ��--------------//
    timer_quad_init(TIMER_2, TIMER2_CHA_A15, TIMER2_CHB_B3);
    timer_quad_init(TIMER_3, TIMER3_CHA_B4, TIMER3_CHB_B5);
    button_start(&button1);
    EnableGlobalIRQ(0);
    while(1)
    {
        if(mt9v03x_finish_flag)
                {
                    mt9v03x_finish_flag = 0;
                    //ʹ��������ʾ����������ԭʼͼ���С �Լ�������Ҫ��ʾ�Ĵ�С�Զ��������Ż��߷Ŵ���ʾ
                    /*GetHistGram1(mt9v03x_image[0], MT9V03X_W*MT9V03X_H);
                    yu1 = OSTUThreshold1();
                    twwo(yu1);*/
                    whu();
                }
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

