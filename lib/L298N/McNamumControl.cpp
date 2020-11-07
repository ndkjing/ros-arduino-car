#include "McNamumControl.h"
#include "L298NX2.h"

//初始化电机引脚配置    右上：1 右下：2 左上：3  左下4
McNamumControl::McNamumControl(uint8_t pinEnable_1,
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
                            uint8_t pin_encoder4_b):L298N1(pinEnable_1,
                                                            pinIN1_1,
                                                            pinIN2_1),
                                                    L298N2(pinEnable_2,
                                                            pinIN1_2,
                                                            pinIN2_2),
                                                    L298N3(pinEnable_3,
                                                                pinIN1_3,
                                                                pinIN2_3),
                                                    L298N4(pinEnable_4,
                                                                pinIN1_4,
                                                                pinIN2_4)
{

    _pin_encoder1_a = pin_encoder1_a;
    _pin_encoder1_b = pin_encoder1_b;
    _pin_encoder2_a = pin_encoder2_a;
    _pin_encoder2_b = pin_encoder2_b;
    _pin_encoder3_a = pin_encoder3_a;
    _pin_encoder3_b = pin_encoder3_b;
    _pin_encoder4_a = pin_encoder4_a;
    _pin_encoder4_b = pin_encoder4_b;



}


void McNamumControl::move(enCarState directions,unsigned short pwm_val=100)
{
    switch (directions)
    {
        case enSTOP:
        {
            // Serial.println("  go_stop");
            stop();
            break;
        }
        case enFORWARD:
        {
            // Serial.println("  forword");
            McNamumControl::forword(pwm_val);
            break;
        }
        case enBACKWARD: 
        {
            Serial.println("  backword");
            McNamumControl::backword(pwm_val);
            break;
        }
        case enLEFT:
        {
            Serial.println("  left_translation");
            McNamumControl::left_translation(pwm_val);
            break;
        }
        case enRIGHT: 
        {
            Serial.println("  right_translation");
            McNamumControl::right_translation(pwm_val);
            break;
        }
        case enROTATE:
        {
            Serial.println("  clockwise_rotate");
            McNamumControl::clockwise_rotate(pwm_val);
            break;
        }
        case enANTIROTATE:
        {
            Serial.println(" anti rotate");
            McNamumControl::anticlockwise_rotate(pwm_val);
            break;
        }
        case en45:
        {
            Serial.println(" go_45");
            McNamumControl::go45(pwm_val);
            break;
        }
        case en135:
        {
            Serial.println(" go_135");
            McNamumControl::go135(pwm_val);
            break;
        }
        case en225:
        {
            Serial.println(" go_225");
            McNamumControl::go225(pwm_val);
            break;
        }
        case en315:
        {
            Serial.println(" go_315");
            McNamumControl::go315(pwm_val);
            break;
        }
    default: break;
    }
}

/////////编码电机输入///////
//右一电机   19号对应中断4
// #define pin_right_1A 19			//A相
// #define pin_right_1B 37		//B相

// //右二电机  18号对应中断5
// #define pin_right_2A 18			//A相
// #define pin_right_2B 39		//B相

// //左一电机  21号对应中断2
// #define pin_left_1A 21			//A相
// #define pin_left_1B 38		//B相

// //左二电机  20号对应中断3
// #define pin_left_2A 20			//A相
// #define pin_left_2B 36		//B相
/////////////////////////////// 电机运动控/////////////////
//前进
void McNamumControl::forword(unsigned short pwm_val) //motor rotate clockwise -->robot go ahead
{
  // 左侧电机控制
  L298N1.forward();
  L298N2.forward();
  L298N3.forward();
  L298N4.forward();
  // 左侧点击控制位
// #define IN1_L  24   //K1、K2 motor direction
// #define IN2_L  25    //K1、K2 motor direction
// #define IN3_L  26   //K3、K4 motor direction
// #define IN4_L  27   //K3、K4 motor direction

// // 右侧点击控制位
// #define IN1_R  30   //K1、K2 motor direction
// #define IN2_R  31    //K1、K2 motor direction
// #define IN3_R  32   //K3、K4 motor direction
// #define IN4_R  33   //K3、K4 motor direction
  //左2
  // digitalWrite(IN1_L, LOW);     
  // digitalWrite(IN2_L, HIGH);     
  //左1
  // digitalWrite(IN3_L, LOW);   
  // digitalWrite(IN4_L, HIGH);

  // // 右侧电机控制
  // // 右2
  //   digitalWrite(IN1_R, HIGH);
  // digitalWrite(IN2_R, LOW);
  // // 右1
  // digitalWrite(IN3_R, HIGH);
  // digitalWrite(IN4_R, LOW);
}

// 后退
void McNamumControl::backword(unsigned short pwm_val)  //motor rotate counterclockwise -->robot go back
{
  L298N1.backward();
  L298N2.backward();
  L298N3.backward();
  L298N4.backward();
  // 左侧电机控制
  // digitalWrite(IN1_L, HIGH);   
  // digitalWrite(IN2_L, LOW);

  // digitalWrite(IN3_L, HIGH);
  // digitalWrite(IN4_L, LOW);

  // // 右侧电机控制
  //   digitalWrite(IN1_R, LOW);
  // digitalWrite(IN2_R, HIGH);

  // digitalWrite(IN3_R, LOW);
  // digitalWrite(IN4_R, HIGH);
}


//停止
void McNamumControl::stop()   //motor brake  -->robot stop
{
  L298N1.stop();
  L298N2.stop();
  L298N3.stop();
  L298N4.stop();
  // digitalWrite(IN1_L, LOW);
  // digitalWrite(IN2_L, LOW);
  // digitalWrite(IN3_L, LOW);
  // digitalWrite(IN4_L, LOW);
  //   // 右侧电机控制
  // digitalWrite(IN1_R, LOW);
  // digitalWrite(IN2_R, LOW);
  // digitalWrite(IN3_R, LOW);
  // digitalWrite(IN4_R, LOW);
}

//左移
void McNamumControl::left_translation(unsigned short pwm_val)  //left motor rotate counterclockwise and right motor rotate clockwise -->robot turn left
{
  L298N1.forward();
  L298N2.backward();
  L298N3.backward();
  L298N4.forward();
    // 左侧电机控制

  //左一
  // digitalWrite(IN3_L, HIGH);   
  // digitalWrite(IN4_L, LOW);

  // //左二
  // digitalWrite(IN1_L, LOW);     
  // digitalWrite(IN2_L, HIGH);   
  
  // // 右侧电机控制
  // // 右1
  // digitalWrite(IN3_R, HIGH);
  // digitalWrite(IN4_R, LOW);
  //   // 右2
  //   digitalWrite(IN1_R, LOW);
  // digitalWrite(IN2_R, HIGH);
}

//右移
void McNamumControl::right_translation(unsigned short pwm_val) //left motor rotate clockwise and right motor rotate counterclockwise -->robot turn right
{
  L298N1.backward();
  L298N2.forward();
  L298N3.forward();
  L298N4.backward();
    // 左侧电机控制
  //左二
  // digitalWrite(IN1_L, HIGH);     
  // digitalWrite(IN2_L, LOW);   
  // //左一
  // digitalWrite(IN3_L, LOW);   
  // digitalWrite(IN4_L, HIGH);

  // // 右侧电机控制
  // // 右1
  // digitalWrite(IN3_R, LOW);
  // digitalWrite(IN4_R, HIGH);
  //   // 右2
  //   digitalWrite(IN1_R, HIGH);
  // digitalWrite(IN2_R, LOW);
}

//顺时针旋转
void McNamumControl::clockwise_rotate(unsigned short pwm_val)
{
     L298N1.backward();
  L298N2.backward();
  L298N3.forward();
  L298N4.forward();
      // 左侧电机控制
  //左一
  // digitalWrite(IN1_L, HIGH);     
  // digitalWrite(IN2_L, LOW);   
  // //左二
  // digitalWrite(IN3_L, LOW);   
  // digitalWrite(IN4_L, HIGH);

  // // 右侧电机控制
  // // 右2
  //   digitalWrite(IN1_R, LOW);
  // digitalWrite(IN2_R, HIGH);
  // // 右1
  // digitalWrite(IN3_R, HIGH);
  // digitalWrite(IN4_R, LOW);
}

//逆时针旋转
void McNamumControl::anticlockwise_rotate(unsigned short pwm_val)
{
  L298N1.forward();
  L298N2.forward();
  L298N3.backward();
  L298N4.backward();
      // 左侧电机控制
  //左一
  // digitalWrite(IN1_L, LOW);     
  // digitalWrite(IN2_L, HIGH);   
  // //左二
  // digitalWrite(IN3_L, HIGH);   
  // digitalWrite(IN4_L, LOW);

  // // 右侧电机控制
  // // 右2
  //   digitalWrite(IN1_R, HIGH);
  // digitalWrite(IN2_R, LOW);
  // // 右1
  // digitalWrite(IN3_R, LOW);
  // digitalWrite(IN4_R, HIGH);
}

//0点方向45度移动旋转
void McNamumControl::go45(unsigned short pwm_val)
{
  L298N1.stop();
  L298N2.forward();
  L298N3.forward();
  L298N4.stop();
    // 左侧电机控制
  //左一
  // digitalWrite(IN1_L, LOW);     
  // digitalWrite(IN2_L, LOW);   
  // //左二
  // digitalWrite(IN3_L, LOW);   
  // digitalWrite(IN4_L, HIGH);

  // // 右侧电机控制
  // // 右2
  //   digitalWrite(IN1_R, LOW);
  // digitalWrite(IN2_R, LOW);
  // // 右1
  // digitalWrite(IN3_R, HIGH);
  // digitalWrite(IN4_R, LOW);
}

//0点方向135度移动旋转
void McNamumControl::go135(unsigned short pwm_val)
{
  L298N1.backward();
  L298N2.stop();
  L298N3.stop();
  L298N4.backward();
    // 左侧电机控制
  //左一
//   digitalWrite(IN1_L, HIGH);     
//   digitalWrite(IN2_L, LOW);   
//   //左二
//   digitalWrite(IN3_L, LOW);   
//   digitalWrite(IN4_L, LOW);

//   // 右侧电机控制
//   // 右2
//     digitalWrite(IN1_R, LOW);
//   digitalWrite(IN2_R, HIGH);
//   // 右1
//   digitalWrite(IN3_R, LOW);
//   digitalWrite(IN4_R, LOW);
}

//0点方向225度移动旋转
void McNamumControl::go225(unsigned short pwm_val)
{
    L298N1.stop();
  L298N2.backward();
  L298N3.backward();
  L298N4.stop();
    // 左侧电机控制
  //左一
  // digitalWrite(IN1_L, LOW);     
  // digitalWrite(IN2_L, HIGH);   
  // //左二
  // digitalWrite(IN3_L, LOW);   
  // digitalWrite(IN4_L, HIGH);

  // // 右侧电机控制
  // // 右2
  //   digitalWrite(IN1_R, HIGH);
  // digitalWrite(IN2_R, LOW);
  // // 右1
  // digitalWrite(IN3_R, HIGH);
  // digitalWrite(IN4_R, LOW);
}

//0点方向315度移动旋转
void McNamumControl::go315(unsigned short pwm_val)
{
  L298N1.forward();
  L298N2.stop();
  L298N3.stop();
  L298N4.forward();
    // 左侧电机控制
  //左一
  // digitalWrite(IN1_L, LOW);     
  // digitalWrite(IN2_L, HIGH);   
  // //左二
  // digitalWrite(IN3_L, LOW);   
  // digitalWrite(IN4_L, HIGH);

  // // 右侧电机控制
  // // 右2
  //   digitalWrite(IN1_R, HIGH);
  // digitalWrite(IN2_R, LOW);
  // // 右1
  // digitalWrite(IN3_R, HIGH);
  // digitalWrite(IN4_R, LOW);
}

