#if 0
#include <PS2X_lib.h>        //PS2手柄
#include<MsTimer2.h>     //定时器
#include "McNamumControl.h"
/*************************************************************
PS2手柄接线方式：
*************************************************************/
#define PS2_DAT        8    //MOSI
#define PS2_CMD        9    //MISO
#define PS2_SEL        10    //CS
#define PS2_CLK        11   //SCLK

//#define PS2_DAT        A0    //MOSI
//#define PS2_CMD        A1   //MISO
//#define PS2_SEL        A2    //CS
//#define PS2_CLK        A3   //SCLK

#define pinEnable_1   4
#define pinIN1_1  32
#define pinIN2_1   33
#define pin_encoder1_a  19
#define pin_encoder1_b  37
#define pinEnable_2  5
#define pinIN1_2   30
#define pinIN2_2    31
#define pin_encoder2_a  18
#define pin_encoder2_b  39
#define pinEnable_3  6
#define pinIN1_3    27
#define pinIN2_3     26 
#define pin_encoder3_a  21
#define pin_encoder3_b  38
#define pinEnable_4   7
#define pinIN1_4  25
#define pinIN2_4   24
#define pin_encoder4_a  20
#define pin_encoder4_b  36
/*************************************************************/
#define pressures   false
#define rumble      false

int error = 0;
PS2X ps2x;                  // create PS2 Controller Class
byte type = 1;
byte vibrate = 0;
enCarState g_carstate = enSTOP; //  1前2后3左4右0停止
int g_servostate = 0;  //1左摇 2 右摇

unsigned char servo, PS2_LY, PS2_LX, PS2_RY, PS2_RX, PS2_KEY; //定义L侧Y轴，X轴以及R侧Y轴，X轴的变量

void (* resetFunc) (void) = 0;                            // Reset func
void PS2_Ctrol(void);     //手柄控制
void show_value(void);   // 显示手柄按键

volatile long m[4] = {0,0, 0, 0};   //
int pps[4] = {0,0,0,0};        // 四个编码器的计数
float velocity[4] = {0, 0, 0, 0};  // 四个轮子的速度
int t = 50;   //检测速度间隔时间单位（ms）

void get_speed();   // 获取编码器速度
void count_encoder1();  // 统计编码器次数 编码器A相 编码器B相  轮子索引
void count_encoder2();
void count_encoder3();
void count_encoder4();

McNamumControl mc_namum_control = McNamumControl(
                          pinEnable_1,
                          pinIN1_1,
                            pinIN2_1,
                            pin_encoder1_a,
                            pin_encoder1_b,
                            pinEnable_2,
                            pinIN1_2,
                            pinIN2_2,
                            pin_encoder2_a,
                            pin_encoder2_b,
                            pinEnable_3,
                            pinIN1_3,
                            pinIN2_3,
                            pin_encoder3_a,
                            pin_encoder3_b,
                            pinEnable_4,
                            pinIN1_4,
                            pinIN2_4,
                            pin_encoder4_a,
                            pin_encoder4_b
);
void setup()   {
  // char error;
  Serial.begin(9600);        //开启串口，波特率9600
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);//PS2控制

  // 编码器获取
  pinMode(pin_encoder1_a, INPUT);
  pinMode(pin_encoder1_b, INPUT);
  pinMode(pin_encoder2_a, INPUT);
  pinMode(pin_encoder2_b, INPUT);
  pinMode(pin_encoder3_a, INPUT);
  pinMode(pin_encoder3_b, INPUT);
  pinMode(pin_encoder4_a, INPUT);
  pinMode(pin_encoder4_b, INPUT);
  attachInterrupt(2, count_encoder1, HIGH);
  attachInterrupt(3, count_encoder2, HIGH);
  attachInterrupt(4, count_encoder3, HIGH);
  attachInterrupt(5, count_encoder4, HIGH);
  MsTimer2::set(t, get_speed);
  MsTimer2::start();

}


/******函数功能：主循环程序体*******/
void loop() {
  if (1)
  {
    ps2x.read_gamepad(false, 0); //read controller and set large motor to spin at 'vibrate' speed
    PS2_LX = ps2x.Analog(PSS_LX); //读取L侧X轴的模拟值
    PS2_LY = ps2x.Analog(PSS_LY); //读取L侧Y轴的模拟值
    PS2_RX = ps2x.Analog(PSS_RX); //读取R侧X轴的模拟值
    PS2_RY = ps2x.Analog(PSS_RY); //读取R侧Y轴的模拟值
    Serial.print("PS2_LX:");
    Serial.print(PS2_LX);
    Serial.print("   PS2_LY:");
    Serial.print(PS2_LY);
    Serial.print("   PS2_RX:");
    Serial.print(PS2_RX);
    Serial.print("   PS2_RY:");
    Serial.print(PS2_RY);

    if (ps2x.Button(PSB_TRIANGLE))
      Serial.println("  PSB_TRIANGLE");
    else if (ps2x.Button(PSB_CROSS))
      Serial.println("  PSB_CROSS");
    else if (ps2x.Button(PSB_CIRCLE))
      Serial.println("  PSB_CIRC");
    else if (ps2x.Button(PSB_SQUARE))
      Serial.println("  PSB_SQUARE");
    else if (ps2x.Button(PSB_PAD_UP))
      Serial.println("  PSB_PAD_UP");
    else if (ps2x.Button(PSB_PAD_DOWN))
      Serial.println("  PSB_PAD_DOWN");
    else if (ps2x.Button(PSB_PAD_RIGHT))
      Serial.println("  PSB_PAD_RIGHT");
    else if (ps2x.Button(PSB_PAD_LEFT))
      Serial.println("  PSB_PAD_LEFT");
    else if (ps2x.Button(PSB_R1))
      Serial.println("  PSB_R1");
    else if (ps2x.Button(PSB_R2))
      Serial.println("  PSB_R2");
    else if (ps2x.Button(PSB_SELECT))
      Serial.println("  PSB_SELECT");
    else if (ps2x.Button(PSB_START))
      Serial.println("  PSB_START");
    else if (ps2x.Button(PSB_L1))
      Serial.println("  PSB_L1");
    else  if (ps2x.Button(PSB_L2))
      Serial.println("  PSB_L2");
    else  Serial.println("  KEY_RELEASE");
  }

  PS2_Ctrol();
  mc_namum_control.move(g_carstate,254);
  delay(t);
}

void PS2_Ctrol(void)
{
//  show_value();
  int X1, Y1, X2, Y2;
  if (error == 1) //skip loop if no controller found
  {
        Serial.println("error == 1");
  return;
    }

  if (type != 1) //skip loop if no controller found
  {
    Serial.println("type != 1");
        return;
    }

  //DualShock Controller
  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

  if (ps2x.Button(PSB_START))        //will be TRUE as long as button is pressed
    {
      Serial.println("Start is being held");
      // g_carstate = enROTATE;
    }
  else if (ps2x.Button(PSB_SELECT))
      {
      Serial.println("Select is being held");
      // g_carstate = enROTATE;
    }
  else if (ps2x.Button(PSB_TRIANGLE))
      {
        Serial.println("  PSB_TRIANGLE");
        g_carstate = en45;
      }
  else if (ps2x.Button(PSB_CROSS))
  {
        Serial.println("  PSB_CROSS");
        g_carstate = en225;
  }
  else if (ps2x.Button(PSB_CIRCLE))
  {
  Serial.println("  PSB_CIRC");
  g_carstate = en135;
  }
  else if (ps2x.Button(PSB_SQUARE))
  {
  Serial.println("  PSB_SQUARE");
  g_carstate = en315;
  }
  else if (ps2x.Button(PSB_L2))
  {
  Serial.println("  PSB_CIRC");
  g_carstate = enANTIROTATE;
  }
  else if (ps2x.Button(PSB_R2))
  {
  Serial.println("  PSB_SQUARE");
  g_carstate = enROTATE;
  }

if (ps2x.Button(PSB_PAD_UP)) 
{     //will be TRUE as long as button is pressed
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    g_carstate = enFORWARD;
}
else if (ps2x.Button(PSB_PAD_RIGHT)) 
{
  Serial.print("Right held this hard: ");
  Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
  g_carstate = enRIGHT;
}
else if (ps2x.Button(PSB_PAD_LEFT)) 
{
  Serial.print("LEFT held this hard: ");
  Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
  g_carstate = enLEFT;
}
else if (ps2x.Button(PSB_PAD_DOWN)) 
{
  Serial.print("DOWN held this hard: ");
  Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
  g_carstate = enBACKWARD;
}
else
{
  g_carstate = enSTOP;
}

vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
if (ps2x.NewButtonState())
{ //will be TRUE if any button changes state (on to off, or off to on)
  if (ps2x.Button(PSB_L3))//停止
  {
    g_carstate = enSTOP;
    Serial.println("L3 pressed");
  }
}
if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
  { //print stick values if either is TRUE   只有当L1 或者R1 被按下时候遥感才有效
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);
    Y1 = ps2x.Analog(PSS_LY);
    X1 = ps2x.Analog(PSS_LX);
    Y2 = ps2x.Analog(PSS_RY);
    X2 = ps2x.Analog(PSS_RX);
    
    /*左摇杆*/
    if (Y1 < 5 && X1 > 80 && X1 < 180) //上
    {
      g_carstate = enFORWARD;
    }
    else if (Y1 > 230 && X1 > 80 && X1 < 180) //下
    {
      g_carstate = enBACKWARD;
    }
    else if (X1 < 5 && Y1 > 80 && Y1 < 180) //左
    {
      g_carstate = enLEFT;
    }
    else if (Y1 > 80 && Y1 < 180 && X1 > 230)//右
    {
      g_carstate = enRIGHT;
    }
    else if (Y1 <= 80 && X1 <= 80) //左上
    {
      g_carstate = en315;
    }
    else if (Y1 <= 80 && X1 >= 180) //右上
    {
      g_carstate = en45;
    }
    else if (X1 <= 80 && Y1 >= 180) // 左下
    {
      g_carstate = en225;
    }
    else if (Y1 >= 180 && X1 >= 180) //右下
    {
      g_carstate = en135;
    }
    else//停
    {
      g_carstate = enSTOP;
    }
    Serial.print("g_carstate:  ");
    Serial.println(g_carstate);
    /*右摇杆*/
    if (X2 < 5 && Y2 > 110 && Y2 < 150) //左
    {
      g_servostate = 1;
    }
    else if (Y2 > 110 && Y2 < 150 && X2 > 230)//右
    {
      g_servostate = 2;
    }
    else//归位
    {
      g_servostate = 0;
    }
  }
}


void show_value(void)
{
   ps2x.read_gamepad(false, 0); //read controller and set large motor to spin at 'vibrate' speed
    PS2_LX = ps2x.Analog(PSS_LX); //读取L侧X轴的模拟值
    PS2_LY = ps2x.Analog(PSS_LY); //读取L侧Y轴的模拟值
    PS2_RX = ps2x.Analog(PSS_RX); //读取R侧X轴的模拟值
    PS2_RY = ps2x.Analog(PSS_RY); //读取R侧Y轴的模拟值
    Serial.print("PS2_LX:");
    Serial.print(PS2_LX);
    Serial.print("   PS2_LY:");
    Serial.print(PS2_LY);
    Serial.print("   PS2_RX:");
    Serial.print(PS2_RX);
    Serial.print("   PS2_RY:");
    Serial.print(PS2_RY);
    if (ps2x.Button(PSB_TRIANGLE))
      Serial.println("  PSB_TRIANGLE");
    else if (ps2x.Button(PSB_CROSS))
      Serial.println("  PSB_CROSS");
    else if (ps2x.Button(PSB_CIRCLE))
      Serial.println("  PSB_CIRC");
    else if (ps2x.Button(PSB_SQUARE))
      Serial.println("  PSB_SQUARE");
    else if (ps2x.Button(PSB_PAD_UP))
      Serial.println("  PSB_PAD_UP");
    else if (ps2x.Button(PSB_PAD_DOWN))
      Serial.println("  PSB_PAD_DOWN");
    else if (ps2x.Button(PSB_PAD_RIGHT))
      Serial.println("  PSB_PAD_RIGHT");
    else if (ps2x.Button(PSB_PAD_LEFT))
      Serial.println("  PSB_PAD_LEFT");
    else if (ps2x.Button(PSB_R1))
      Serial.println("  PSB_R1");
    else if (ps2x.Button(PSB_R2))
      Serial.println("  PSB_R2");
    else if (ps2x.Button(PSB_SELECT))
      Serial.println("  PSB_SELECT");
    else if (ps2x.Button(PSB_START))
      Serial.println("  PSB_START");
    else if (ps2x.Button(PSB_L1))
      Serial.println("  PSB_L1");
    else  if (ps2x.Button(PSB_L2))
      Serial.println("  PSB_L2");
    else  Serial.println("  KEY_RELEASE");  
}


//获取所有编码器速度
void get_speed()
{
  detachInterrupt(2);
  detachInterrupt(3);
  detachInterrupt(4);
  detachInterrupt(5);
  for(int i=0;i<4;i++)
  {
    float rpm=int(pps[i]); 
    Serial.print("  M: ");
    Serial.print(i);
    Serial.print("   ");
    Serial.print(rpm);
    //    Serial.print(i + 1);
    //    Serial.print(": ");
    //    Serial.print(velocity[i]);
    //    Serial.print("r/s ");
    pps[i] = 0;
  }
  Serial.println(" ");
    attachInterrupt(2, count_encoder1, HIGH);
    attachInterrupt(3, count_encoder2, HIGH);
    attachInterrupt(4, count_encoder3, HIGH);
    attachInterrupt(5, count_encoder4, HIGH);
}

//编码器计数
void count_encoder1()
{
  if (digitalRead(pin_encoder1_a) == LOW) 
  {
    if (digitalRead(pin_encoder1_b) == HIGH) pps[0]--;
    else if (digitalRead(pin_encoder1_b) == LOW) pps[0]++;
  }
  else 
  {
    if (digitalRead(pin_encoder1_b) == HIGH) pps[0]++;
    else if (digitalRead(pin_encoder1_b) == LOW) pps[0]--;
  }
}

void  count_encoder2()
{
  if (digitalRead(pin_encoder2_a) == LOW) 
  {
    if (digitalRead(pin_encoder2_b) == HIGH) pps[1]--;
    else if (digitalRead(pin_encoder2_b) == LOW) pps[1]++;
  }
  else 
  {
    if (digitalRead(pin_encoder2_b) == HIGH) pps[1]++;
    else if (digitalRead(pin_encoder2_b) == LOW) pps[1]--;
  }

}

void  count_encoder3()
{
  if (digitalRead(pin_encoder3_a) == LOW) 
  {
    if (digitalRead(pin_encoder3_b) == HIGH) pps[2]--;
    else if (digitalRead(pin_encoder3_b) == LOW) pps[2]++;
  }
  else 
  {
    if (digitalRead(pin_encoder3_b) == HIGH) pps[2]++;
    else if (digitalRead(pin_encoder3_b) == LOW) pps[2]--;
  }

}

void  count_encoder4()
{
  if (digitalRead(pin_encoder4_a) == LOW) 
  {
    if (digitalRead(pin_encoder4_b) == HIGH) pps[3]--;
    else if (digitalRead(pin_encoder4_b) == LOW) pps[3]++;
  }
  else 
  {
    if (digitalRead(pin_encoder4_b) == HIGH) pps[3]++;
    else if (digitalRead(pin_encoder4_b) == LOW) pps[3]--;
  }

}


#endif