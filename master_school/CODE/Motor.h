#ifndef __MOTOR_H__
#define __MOTOR_H__
#define PWM_xianfu 5000
#include "headfile.h"
typedef struct PID
{
   float target_value;   //璁惧畾鍊�
   float current_value;  //褰撳墠鍊�
   float err;           //璇樊
   float last_err;      //涓婃鐨勮宸�
   float last_last_err; //涓婁笂娆＄殑璇樊
   float errsum;
   float Ki,Kp,Kd;      //PID鍙傛暟
   float output;        //杈撳嚭
   float integral;      //绉垎鍊硷紙璇樊绱姞鍊硷級
} PID_Typedef;
void Motorinit(void);  //鐢垫満鍒濆鍖�
void Motor_Set(int32 PWM1, int32 PWM2, int32 PWM3, int32 PWM4); //杈撳嚭PWM鍒扮數鏈�
void PID_Init(PID_Typedef *PID,float kp,float ki,float kd);
void GetspeedPID(int16 target,int16 current,PID_Typedef *PID);
void GetposPID(float target,float current ,PID_Typedef *PID);
#endif

