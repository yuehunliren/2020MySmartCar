#include "headfile.h"
#include "jf_data.h"
#include "multi_button.h"
#include <stdlib.h>

#define LINE_LEN                13             //数据长度
#define chazhi 30

#define white 100

/*添加定义变量*/
uint8 BEN_FLAG,STA_FLAG;






uint8 temp_buff[LINE_LEN];                      //从机向主机发送数据BUFF

int16 slave_encoder_RB;                       //从机右后编码器值
int16 slave_encoder_LB;                      //从机左前编码器值
//int16 slave_position;                           //从机转角值
uint8 tag1[50], tag2[50], tag3[50], tag5 = 0, tag6 = 0, tag7 = 0, tag8 = 1, tag9 = 0;//tag1左右边界，tag3中值，tag4差值(暂时移除)，tag5赛道状态标志位，tag67超限位标志
int16 i, w;
int symbals = 10, ttim = 0, tyim = 0;

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
//  @brief      获取传感器数据
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void get_sensor_data(void)
{
    slave_encoder_LB = timer_quad_get(TIMER_3);      //B4-B5编码器取值
    slave_encoder_RB = timer_quad_get(TIMER_2);       //A15-B3编码器取值
    timer_quad_clear(TIMER_2);                          //清空计数器
    timer_quad_clear(TIMER_3);                          //清空计数器

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      根据协议处理数据
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void process_data(void)
{
    temp_buff[0] = 0xD8;                         //帧头

    temp_buff[1] = 0xB0;                         //功能字
    temp_buff[2] = slave_encoder_RB>>8;        //数据高8位
    temp_buff[3] = slave_encoder_RB&0xFF;      //数据低8位

    temp_buff[4] = 0xB1;                         //功能字
    temp_buff[5] = slave_encoder_LB>>8;       //数据高8位
    temp_buff[6] = slave_encoder_LB&0xFF;     //数据低8位

    temp_buff[7] = 0xB2;                         //功能字
    temp_buff[8] = tag3[symbals];            //数据1
    //temp_buff[8] = 0x05;
    temp_buff[9] = tag5;                     //数据2
    if (buflag == 0)
    {
        temp_buff[10] = 0xA1;
    }
    else
    {
        temp_buff[10] = 0xA2;
    }
    temp_buff[11] = STA_FLAG;
    temp_buff[12] = 0xEE;                        //帧尾
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器4中断服务函数
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
        gpio_set(B0,1);                           //A0引脚拉高

        get_sensor_data();                          //获取传感器数据。
        if (tyim == 1)
        {
            ttim++;
        }
        if (ttim < 80)
        {
            //tag5 = 0;
        }
        else
        {
            tag5 = 0xf1;
            tyim = 0;
            ttim = 0;
        }
        process_data();                             //根据协议处理数据，并存入temp_buff中。
        uart_putbuff(UART_1, temp_buff, LINE_LEN);  //通过串口1将数据发送出去。
        button_ticks();
        gpio_set(B0, 0);                         //A0引脚拉低
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
    for (i = MT9V03X_H - 2; i >= 0; i--)      //取左右边界
    {
        tag3[i] = 97;                          //取中线
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
        tag3[i] = (tag1[i] + tag2[i]) / 2;               //取中线，改变后面计算
    }
}
void whu(void)
{
    tag8 = 1;
        tag5 = 0;
        tag1[49] = 184;
        tag2[49] = 10;
        for (i = MT9V03X_H - 2; i >= 0; i--)      //取左右边界
        {
            tag3[i] = 97;                          //取中线
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
                                            //tag5 = 0xf1;
                                            tag9 = 0;
                                            tyim = 1;
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
            tag3[i] = (tag1[i] + tag2[i]) / 2;               //取中线，改变后面计算
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




/*添加代码*/


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
    board_init();           //务必保留，本函数用于初始化MPU 时钟 调试串口
    mt9v03x_init();
//--------------------两块板子之间的连接初始化----------------//
    uart_init(UART_1, 460800, UART1_TX_A9, UART1_RX_A10);
    gpio_init(B0, GPO, 0, GPIO_PIN_CONFIG);  //B0为两个板子之间的连接
    timer_pit_interrupt_ms(TIMER_4, 5);      //定时器4 5ms定时器中断
//-------------------按键初始化--------------------//
    gpio_init(C14, GPI, 1,GPIO_INT_CONFIG);
    button_init(&button1, read_c14gpio,0);
    button_attach(&button1, PRESS_UP, btn1_press_up_Handler);
//------------------编码器的初始化--------------//
    timer_quad_init(TIMER_2, TIMER2_CHA_A15, TIMER2_CHB_B3);
    timer_quad_init(TIMER_3, TIMER3_CHA_B4, TIMER3_CHB_B5);
    button_start(&button1);
    EnableGlobalIRQ(0);
    while(1)
    {
        if(mt9v03x_finish_flag)
                {
                    mt9v03x_finish_flag = 0;
                    //使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
                    /*GetHistGram1(mt9v03x_image[0], MT9V03X_W*MT9V03X_H);
                    yu1 = OSTUThreshold1();
                    twwo(yu1);*/
                    whu();
                    if (STA_FLAG)
                    {
                        symbals = 10;
                    }
                    else
                    {
                        symbals = 35;
                    }
                }
        // 此处编写需要循环执行的代码
    }
}

