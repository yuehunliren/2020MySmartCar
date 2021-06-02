#include "headfile.h"
#include "Motor.h"
#include <stdlib.h>
void Motorinit(void)
{
    gpio_init(A4, GPO, 1, GPIO_PIN_CONFIG);
    gpio_init(A6, GPO, 1, GPIO_PIN_CONFIG);
    gpio_init(C14, GPO, 1, GPIO_PIN_CONFIG);
    gpio_init(C15, GPO, 1, GPIO_PIN_CONFIG);
    pwm_init(PWM4_CH1_B6, 10000, 0); //初始化PWM4 通道1 使用引脚B6  输出PWM频率10KHZ   占空比为百分之 5000/PWM_DUTY_MAX*100
    pwm_init(PWM4_CH2_B7, 10000, 0);
    pwm_init(PWM4_CH3_B8, 10000, 0);
    pwm_init(PWM4_CH4_B9, 10000, 0);
}
uint32 xianfu(int32 PWM)
{
    uint32 PWM1;
    PWM1 = abs(PWM);
    if (PWM1 >= 9000)
    PWM1 = PWM_xianfu;
    return PWM1;
}
void Motor_Set(int32 PWM1, int32 PWM2, int32 PWM3, int32 PWM4)
{
    if (PWM1 < 0)gpio_set(A6, 1);
    else gpio_set(A6, 0);
    if (PWM2 < 0)gpio_set(A4, 0);
    else gpio_set(A4, 1);
    if (PWM3 < 0)gpio_set(C14, 0);
    else gpio_set(C14, 1);
    if (PWM4 < 0)gpio_set(C15, 0);
    else gpio_set(C15, 1);
    pwm_duty(PWM4_CH2_B7, xianfu(PWM1));//LF
    pwm_duty(PWM4_CH1_B6, xianfu(PWM2));//RF
    pwm_duty(PWM4_CH3_B8, xianfu(PWM3));//LB
    pwm_duty(PWM4_CH4_B9, xianfu(PWM4));//RB
}
void PID_Init(PID_Typedef *PID,float kp,float ki,float kd)
{
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->err = 0;
    PID->last_err = 0;
    PID->last_last_err = 0;
    PID->output = 0;
}
void GetspeedPID(int16 target,int16 current,PID_Typedef *PID)
{
    float output;
    PID->err = target - current;
    output = PID->Kp *(PID->err - PID->last_err) + PID->Ki * PID->err +PID->Kd * (PID->err - 2*PID->last_err +PID->last_last_err);
    PID->output = PID->output+output;
    PID->last_err = PID->err;
    PID->last_last_err = PID->last_err;
}
void GetposPID(float target,float current ,PID_Typedef *PID)
{
    PID->err = target - current;
    if(PID->err <=1 && PID->err>=-1)
    {
        PID->err = 0;
    }
    PID->output = PID->Kp*PID->err+PID->Ki*PID->errsum+PID->Kd*(PID->last_err-PID->err);
    PID->errsum += PID->err;
    PID->last_err = PID->err;
}

