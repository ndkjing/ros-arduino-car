/**
 * 麦克纳姆轮方向控制 
 * **/

#ifndef MCNAMUMCONTROL_H
#define MCNAMUMCONTROL_H

#include "Arduino.h"
#include "L298N.h"

/*小车运行状态枚举*/
 typedef enum {
  enSTOP = 1,
  enFORWARD,
  enBACKWARD,
  enLEFT,
  enRIGHT,
  en45,
  en135,
  en225,
  en315,
  enROTATE,
  enANTIROTATE
} enCarState;

class McNamumControl
{
public:
//初始化电机引脚配置    右上：1 右下：2 左上：3  左下4
   McNamumControl(uint8_t pinEnable_1,
                  uint8_t pinIN1_1,
                  uint8_t pinIN2_1,
                  uint8_t pin_encoder1_a,
                  uint8_t pin_encoder1_b,
                  uint8_t pinEnable_2,
                  uint8_t pinIN1_2,
                  uint8_t pinIN2_2,
                  uint8_t pin_encoder2_a,
                  uint8_t pin_encoder2_b,
                  uint8_t pinEnable_3,
                  uint8_t pinIN1_3,
                  uint8_t pinIN2_3,
                  uint8_t pin_encoder3_a,
                  uint8_t pin_encoder3_b,
                  uint8_t pinEnable_4,
                  uint8_t pinIN1_4,
                  uint8_t pinIN2_4,
                  uint8_t pin_encoder4_a,
                  uint8_t pin_encoder4_b
       );



   void setSpeed(unsigned short pwmVal);
   //移动方向空置  按枚举值和速度方向运动
   void move(enCarState directions,unsigned short pwmVal);
   void forword(unsigned short pwm_val);  //前进
   void backword(unsigned short pwm_val);  //后退
   void stop();   //停止
   void clockwise_rotate(unsigned short pwm_val);   //顺时针旋转
   void anticlockwise_rotate(unsigned short pwm_val);   //逆时针旋转
   void left_translation(unsigned short pwm_val);   //   向左平移
   void right_translation(unsigned short pwm_val);   //  向右平移
   void go45(unsigned short pwm_val);   //   45度前进
   void go135(unsigned short pwm_val);   // 135度前进
   void go225(unsigned short pwm_val);   //   225度前进
   void go315(unsigned short pwm_val);   //   315度前进

public:
// 四驱电机左右轮子
   L298N L298N1;    
   L298N L298N2;
   L298N L298N3;
   L298N L298N4;
   uint8_t _pin_encoder1_a;
   uint8_t _pin_encoder1_b ;
   uint8_t _pin_encoder2_a;
   uint8_t _pin_encoder2_b;
   uint8_t _pin_encoder3_a;
   uint8_t _pin_encoder3_b;
   uint8_t _pin_encoder4_a;
   uint8_t _pin_encoder4_b;
   unsigned long _lastMs;
   boolean _canMove;
   float speed;
   int pps[4];
   static void fakeCallback();
};

#endif



