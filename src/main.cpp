#include <PS2X_lib.h>        //PS2手柄
#include<MsTimer2.h>     //定时器

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

// 左侧点击控制位
#define IN1_L  24   //K1、K2 motor direction
#define IN2_L  25    //K1、K2 motor direction
#define IN3_L  26   //K3、K4 motor direction
#define IN4_L  27   //K3、K4 motor direction

// 右侧点击控制位
#define IN1_R  30   //K1、K2 motor direction
#define IN2_R  31    //K1、K2 motor direction
#define IN3_R  32   //K3、K4 motor direction
#define IN4_R  33   //K3、K4 motor direction
/*************************************************************/
#define pressures   false
#define rumble      false

/*小车运行状态枚举*/
enum {
  enSTOP = 1,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  en45,
  en135,
  en225,
  en315,
  enROTATE,
  enANTIROTATE
} enCarState;


int error = 0;
PS2X ps2x;                  // create PS2 Controller Class
byte type = 1;
byte vibrate = 0;
int g_carstate = enSTOP; //  1前2后3左4右0停止
int g_servostate = 0;  //1左摇 2 右摇

unsigned char servo, PS2_LY, PS2_LX, PS2_RY, PS2_RX, PS2_KEY; //定义L侧Y轴，X轴以及R侧Y轴，X轴的变量

void (* resetFunc) (void) = 0;                            // Reset func
void PS2_Ctrol(void);
void show_value(void);

/////////编码电机输入///////
//右一电机
#define pin_right_1A 19			//A相
#define pin_right_1B 37		//B相
//右二电机
#define pin_right_2A 19			//A相
#define pin_right_2B 37		//B相

//左一电机
#define pin_left_1A 19			//A相
#define pin_left_1B 37		//B相
//左二电机
#define pin_left_2A 19			//A相
#define pin_left_2B 37		//B相


volatile long m[4] = {0,0, 0, 0};
int pps[4] = {0,0,0,0};
float velocity[4] = {0, 0, 0, 0};
int t = 500;

void count_right_1() 
{
  if (digitalRead(pin_right_1A) == LOW) 
  {
    if (digitalRead(pin_right_1B) == HIGH) pps[0]++;
    else if (digitalRead(pin_right_1B) == LOW) pps[0]--;
  }
  else 
  {
    if (digitalRead(pin_right_1B) == HIGH) pps[0]--;
    else if (digitalRead(pin_right_1B) == LOW) pps[0]++;
  }
}

void count_right_2() 
{
  if (digitalRead(pin_right_2A) == LOW) 
  {
    if (digitalRead(pin_right_2B) == HIGH) pps[1]++;
    else if (digitalRead(pin_right_1B) == LOW) pps[0]--;
  } 
  else 
  {
    if (digitalRead(pin_right_1B) == HIGH) pps[1]--;
    else if (digitalRead(pin_right_1B) == LOW) pps[0]++;
  }
}

void count_left_1() 
{
  if (digitalRead(pin_right_1A) == LOW) 
  {
    if (digitalRead(pin_right_1B) == HIGH) pps[0]++;
    else if (digitalRead(pin_right_1B) == LOW) pps[0]--;
  } 
  else
  {
    if (digitalRead(pin_right_1B) == HIGH) pps[0]--;
    else if (digitalRead(pin_right_1B) == LOW) pps[0]++;
  }
}

void count_left_2() 
{
  if (digitalRead(pin_right_1A) == LOW) 
  {
    if (digitalRead(pin_right_1B) == HIGH) pps[1]++;
    else if (digitalRead(pin_right_1B) == LOW) pps[0]--;
  } 
  else 
  {
    if (digitalRead(pin_right_1B) == HIGH) pps[1]--;
    else if (digitalRead(pin_right_1B) == LOW) pps[0]++;
  }
}

void SpeedDetection() 
{
  detachInterrupt(4);
  for(int i=0;i<4;i++)
  {
    float rpm=int(pps[i]/37.4); 
    Serial.print("M:   ");
    //    Serial.print(i + 1);
    //    Serial.print(": ");
    //    Serial.print(velocity[i]);
    //    Serial.print("r/s ");
    Serial.print(rpm);

    pps[i] = 0;
  }

  attachInterrupt(4, count_left_1, CHANGE);
}


/////////////////////////////// 电机运动控/////////////////
//前进
void go_ahead() //motor rotate clockwise -->robot go ahead
{
  // 左侧电机控制
  //左2
  digitalWrite(IN1_L, LOW);     
  digitalWrite(IN2_L, HIGH);   
  //左1
  digitalWrite(IN3_L, LOW);   
  digitalWrite(IN4_L, HIGH);

  // 右侧电机控制
  // 右2
    digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  // 右1
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
}

// 后退
void go_back()  //motor rotate counterclockwise -->robot go back
{
  // 左侧电机控制
  digitalWrite(IN1_L, HIGH);   
  digitalWrite(IN2_L, LOW);

  digitalWrite(IN3_L, HIGH);
  digitalWrite(IN4_L, LOW);

  // 右侧电机控制
    digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);

  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, HIGH);
}


//停止
void go_stop()   //motor brake  -->robot stop
{
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN3_L, LOW);
  digitalWrite(IN4_L, LOW);
    // 右侧电机控制
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, LOW);
}

//左移
void turn_left()  //left motor rotate counterclockwise and right motor rotate clockwise -->robot turn left
{
    // 左侧电机控制

  //左一
  digitalWrite(IN3_L, HIGH);   
  digitalWrite(IN4_L, LOW);

  //左二
  digitalWrite(IN1_L, LOW);     
  digitalWrite(IN2_L, HIGH);   
  
  // 右侧电机控制
  // 右1
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
    // 右2
    digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
}

//右移
void turn_right() //left motor rotate clockwise and right motor rotate counterclockwise -->robot turn right
{
    // 左侧电机控制
  //左二
  digitalWrite(IN1_L, HIGH);     
  digitalWrite(IN2_L, LOW);   
  //左一
  digitalWrite(IN3_L, LOW);   
  digitalWrite(IN4_L, HIGH);

  // 右侧电机控制
  // 右1
  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, HIGH);
    // 右2
    digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
}

//顺时针旋转
void clockwise_rotate()
{
      // 左侧电机控制
  //左一
  digitalWrite(IN1_L, HIGH);     
  digitalWrite(IN2_L, LOW);   
  //左二
  digitalWrite(IN3_L, LOW);   
  digitalWrite(IN4_L, HIGH);

  // 右侧电机控制
  // 右2
    digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
  // 右1
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
}

//逆时针旋转
void anticlockwise_rotate()
{
      // 左侧电机控制
  //左一
  digitalWrite(IN1_L, LOW);     
  digitalWrite(IN2_L, HIGH);   
  //左二
  digitalWrite(IN3_L, HIGH);   
  digitalWrite(IN4_L, LOW);

  // 右侧电机控制
  // 右2
    digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  // 右1
  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, HIGH);
}

//0点方向45度移动旋转
void go_45()
{
    // 左侧电机控制
  //左一
  digitalWrite(IN1_L, LOW);     
  digitalWrite(IN2_L, LOW);   
  //左二
  digitalWrite(IN3_L, LOW);   
  digitalWrite(IN4_L, HIGH);

  // 右侧电机控制
  // 右2
    digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
  // 右1
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
}

//0点方向135度移动旋转
void go_135()
{
    // 左侧电机控制
  //左一
  digitalWrite(IN1_L, HIGH);     
  digitalWrite(IN2_L, LOW);   
  //左二
  digitalWrite(IN3_L, LOW);   
  digitalWrite(IN4_L, LOW);

  // 右侧电机控制
  // 右2
    digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
  // 右1
  digitalWrite(IN3_R, LOW);
  digitalWrite(IN4_R, LOW);
}

//0点方向225度移动旋转
void go_225()
{
    // 左侧电机控制
  //左一
  digitalWrite(IN1_L, LOW);     
  digitalWrite(IN2_L, HIGH);   
  //左二
  digitalWrite(IN3_L, LOW);   
  digitalWrite(IN4_L, HIGH);

  // 右侧电机控制
  // 右2
    digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  // 右1
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
}

//0点方向315度移动旋转
void go_315()
{
    // 左侧电机控制
  //左一
  digitalWrite(IN1_L, LOW);     
  digitalWrite(IN2_L, HIGH);   
  //左二
  digitalWrite(IN3_L, LOW);   
  digitalWrite(IN4_L, HIGH);

  // 右侧电机控制
  // 右2
    digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  // 右1
  digitalWrite(IN3_R, HIGH);
  digitalWrite(IN4_R, LOW);
}




void setup()   {
  // char error;
  Serial.begin(9600);        //开启串口，波特率9600
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);//PS2控制
  // 编码器获取
  pinMode(pin_left_1A, INPUT);
  pinMode(pin_left_1B, INPUT);
  pinMode(pin_left_2A, INPUT);
  pinMode(pin_left_2B, INPUT);
  pinMode(pin_right_1A, INPUT);
  pinMode(pin_right_1B, INPUT);
  pinMode(pin_right_2A, INPUT);
  pinMode(pin_right_2B, INPUT);

  attachInterrupt(4, count_left_1, HIGH);
  MsTimer2::set(t, SpeedDetection);
  MsTimer2::start();

}


/******函数功能：主循环程序体*******/
void loop() {
  if (0)
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
  int temp = 0;
  PS2_Ctrol();
  switch (g_carstate)
  {
    case enSTOP:
      {
        Serial.println("  go_stop");
        go_stop();
        break;
      }
    case enRUN:
      {
        Serial.println("  go_ahead");
        go_ahead();
        break;
      }
    case enLEFT:
      {
        Serial.println("  turn_left");
        turn_left();
        break;
      }
    case enRIGHT: 
      {
        Serial.println("  turn_right");
        turn_right();
        break;
      }
    case enBACK: 
    {
        Serial.println("  go_back");
        go_back();
        break;
      }
    case enROTATE:
    {
        Serial.println("  rotate");
        clockwise_rotate();
        break;
    }
    case enANTIROTATE:
    {
        Serial.println(" anti rotate");
        anticlockwise_rotate();
        break;
    }
    case en45:
    {
        Serial.println(" go_45");
        go_45();
        break;
    }
    case en135:
    {
        Serial.println(" go_135");
        go_135();
        break;
    }

    //    case enUPLEFT: upleft();  break;
    //    case enUPRIGHT: upright(); break;
    //    case enDOWNLEFT: downleft();  break;
    //    case enDOWNRIGHT: downright(); break;
    default: break;
  }
  delay(500);
  //  switch (g_servostate)
  //  {
  //    case 0: if (temp != 0) {
  //        temp = 0;
  //        front_detection();
  //      } break;
  //    case 1: if (temp != 1) {
  //        temp = 1;
  //        left_detection();
  //      } break;
  //    case 2: if (temp != 2) {
  //        temp = 2;
  //        right_detection();
  //      } break;
  //    default: break;
  //  }

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
  if (ps2x.Button(PSB_SELECT))
      {
      Serial.println("Select is being held");
      // g_carstate = enROTATE;
    }
if (ps2x.Button(PSB_TRIANGLE))
    {
      Serial.println("  PSB_TRIANGLE");
      g_carstate = enROTATE;
    }
if (ps2x.Button(PSB_CROSS))
{
      Serial.println("  PSB_CROSS");
      g_carstate = enROTATE;
}

else if (ps2x.Button(PSB_CIRCLE))
  Serial.println("  PSB_CIRC");
else if (ps2x.Button(PSB_SQUARE))
  Serial.println("  PSB_SQUARE");
if (ps2x.Button(PSB_PAD_UP)) 
{     //will be TRUE as long as button is pressed
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    g_carstate = enRUN;
}
  else if (ps2x.Button(PSB_PAD_RIGHT)) {
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    g_carstate = enRIGHT;
  }
  else if (ps2x.Button(PSB_PAD_LEFT)) {
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    g_carstate = enLEFT;
  }
  else if (ps2x.Button(PSB_PAD_DOWN)) {
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    g_carstate = enBACK;
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
      g_carstate = enRUN;
    }
    else if (Y1 > 230 && X1 > 80 && X1 < 180) //下
    {
      g_carstate = enBACK;
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