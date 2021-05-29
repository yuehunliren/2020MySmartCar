#include "headfile.h"
#include "encoder.h"
void Encoder_master_init()
{
    timer_quad_init(TIMER_2, TIMER2_CHA_A15, TIMER2_CHB_B3);
    timer_quad_init(TIMER_3, TIMER3_CHA_B4, TIMER3_CHB_B5);
}
void Encoder_master_get(int16 *speed1, int16 *speed2)
{
    *speed1 = timer_quad_get(TIMER_2);
    *speed2 = -timer_quad_get(TIMER_3);
    timer_quad_clear(TIMER_2);          //清空定时器的计数器值
    timer_quad_clear(TIMER_3);          //清空定时器的计数器值
}
