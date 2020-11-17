#ifdef 0

#include <PS2X_lib.h>        //PS2手柄
#include "Arduino.h"
#include<MsTimer2.h>     //定时器
// ros包使用
 #include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <PID_v1.h>
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

int error = 0;
PS2X ps2x;                  // create PS2 Controller Class
byte type = 1;
byte vibrate = 0;
enCarState g_carstate = enSTOP; //  1前2后3左4右0停止


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
void move(enCarState directions,uint8_t pwm_val);
void forward(uint8_t pwm_val);
void backward(uint8_t pwm_val);
void stop();
void clockwise_rotate(uint8_t pwm_val);   //顺时针旋转
void anticlockwise_rotate(uint8_t pwm_val);   //逆时针旋转
void left_translation(uint8_t pwm_val);   //   向左平移
void right_translation(uint8_t pwm_val);   //  向右平移
void go45(uint8_t pwm_val);   //   45度前进
void go135(uint8_t pwm_val);   // 135度前进
void go225(uint8_t pwm_val);   //   225度前进
void go315(uint8_t pwm_val);   //   315度前进

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void publishSpeed(double time);
//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;      

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 4;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 5;              //B channel for encoder of right motor 

const int PIN_SIDE_LIGHT_LED = 46;                  //Side light blinking led pin

unsigned long lastMilli = 0;

//--- Robot-specific constants ---
const double radius = 0.04;                   //Wheel radius, in m
const double wheelbase = 0.187;               //Wheelbase, in m
const double encoder_cpr = 990;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00235;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 
                    
const double max_speed = 0.4;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 
                                                  
// PID Parameters
const double PID_left_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor
ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type


void setup()   {
  // char error;
  Serial.begin(57600);        //开启串口，波特率9600
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);//PS2控制
  
  // 输出初始化
  pinMode(pinEnable_1,OUTPUT);
  pinMode(pinIN1_1, OUTPUT);
  pinMode(pinIN2_1, OUTPUT);
  pinMode(pinEnable_2,OUTPUT);
  pinMode(pinIN1_2, OUTPUT);
  pinMode(pinIN2_2, OUTPUT);
  pinMode(pinEnable_3,OUTPUT);
  pinMode(pinIN1_3, OUTPUT);
  pinMode(pinIN2_3, OUTPUT);
  pinMode(pinEnable_4,OUTPUT);
  pinMode(pinIN1_4, OUTPUT);
  pinMode(pinIN2_4, OUTPUT);

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
  // MsTimer2::set(t, get_speed);
  // MsTimer2::start();
  
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic

  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

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
  //使用手柄进行控制
  PS2_Ctrol();
  move(g_carstate,200);
  nh.spinOnce();
  get_speed();
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
    
    if (!nh.connected()){
     Serial.println("nh  connected error");
    }
    
    
    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }
    
    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }
    
    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
         stop();
    }
    else if (speed_req_left == 0){                        //Stopping
      stop();
    }
    else if (PWM_leftMotor > 0){                          //Going forward
      backward(abs(PWM_leftMotor));
    }
    else {                                               //Going backward
      forward(abs(PWM_leftMotor));
    }
    
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
     stop();
    }
    else if (speed_req_right == 0){                       //Stopping
      stop();
    }
    else if (PWM_rightMotor > 0){                         //Going forward
      forward(abs(PWM_rightMotor));
    }
    else {                                                //Going backward
      backward(abs(PWM_rightMotor));
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
  
  delay(t);
}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
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
  //   if (X2 < 5 && Y2 > 110 && Y2 < 150) //左
  //   {
  //     g_servostate = 1;
  //   }
  //   else if (Y2 > 110 && Y2 < 150 && X2 > 230)//右
  //   {
  //     g_servostate = 2;
  //   }
  //   else//归位
  //   {
  //     g_servostate = 0;
  //   }
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


void move(enCarState directions,uint8_t pwm_val)
{
  Serial.println(directions);
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
            forward(pwm_val);
            break;
        }
        case enBACKWARD: 
        {
            Serial.println("  backword");
            backward(pwm_val);
            break;
        }
        case enLEFT:
        {
            Serial.println("  left_translation");
            left_translation(pwm_val);
            break;
        }
        case enRIGHT: 
        {
            Serial.println("  right_translation");
           right_translation(pwm_val);
            break;
        }
        case enROTATE:
        {
            Serial.println("  clockwise_rotate");
            clockwise_rotate(pwm_val);
            break;
        }
        case enANTIROTATE:
        {
            Serial.println(" anti rotate");
            anticlockwise_rotate(pwm_val);
            break;
        }
        case en45:
        {
            Serial.println(" go_45");
            go45(pwm_val);
            break;
        }
        case en135:
        {
            Serial.println(" go_135");
            go135(pwm_val);
            break;
        }
        case en225:
        {
            Serial.println(" go_225");
            go225(pwm_val);
            break;
        }
        case en315:
        {
            Serial.println(" go_315");
            go315(pwm_val);
            break;
        }
    default: break;
    }
}

void forward1(uint8_t pwm_val)
{
  digitalWrite(pinIN1_1, HIGH);
  digitalWrite(pinIN2_1, LOW);
  analogWrite(pinEnable_1, pwm_val);
}
void backward1(uint8_t pwm_val)
{
  digitalWrite(pinIN1_1, LOW);
  digitalWrite(pinIN2_1, HIGH);
  analogWrite(pinEnable_1, pwm_val);
}
void forward2(uint8_t pwm_val)
{
      digitalWrite(pinIN1_2, HIGH);
  digitalWrite(pinIN2_2, LOW);
  analogWrite(pinEnable_2, pwm_val);
}
void backward2(uint8_t pwm_val)
{
  digitalWrite(pinIN1_2, LOW);
  digitalWrite(pinIN2_2, HIGH);
  analogWrite(pinEnable_2, pwm_val);
}
void forward3(uint8_t pwm_val)
{
      digitalWrite(pinIN1_3, HIGH);
  digitalWrite(pinIN2_3, LOW);
  analogWrite(pinEnable_3, pwm_val);
}
void backward3(uint8_t pwm_val)
{
      digitalWrite(pinIN1_3, LOW);
  digitalWrite(pinIN2_3, HIGH);
  analogWrite(pinEnable_3, pwm_val);
}
void forward4(uint8_t pwm_val)
{
      digitalWrite(pinIN1_4, HIGH);
  digitalWrite(pinIN2_4, LOW);
  analogWrite(pinEnable_4, pwm_val);
}
void backward4(uint8_t pwm_val)
{
  digitalWrite(pinIN1_4, LOW);
  digitalWrite(pinIN2_4, HIGH);
  analogWrite(pinEnable_4, pwm_val);
}

void stop1()
{
analogWrite(pinEnable_1, 0);
}

void stop2()
{
analogWrite(pinEnable_2, 0);
}
void stop3()
{
  analogWrite(pinEnable_3, 0);
}
void stop4()
{
  analogWrite(pinEnable_4, 0);
}

void forward(uint8_t pwm_val)
{
    forward1(pwm_val);
    forward2(pwm_val);
    forward3(pwm_val);
    forward4(pwm_val);
}

void backward(uint8_t pwm_val)
{
    backward1(pwm_val);
    backward2(pwm_val);
    backward3(pwm_val);
    backward4(pwm_val);
}

void stop()
{
  stop1();
  stop2();
  stop3();
  stop4();
}

void left_translation(uint8_t pwm_val)
{
  forward1(pwm_val);
  backward2(pwm_val);
  backward3(pwm_val);
  forward4(pwm_val);
}

void right_translation(uint8_t pwm_val)
{
  backward1(pwm_val);
  forward2(pwm_val);
  forward3(pwm_val);
  backward4(pwm_val);
}

void clockwise_rotate(uint8_t pwm_val)   //顺时针旋转
{
  backward1(pwm_val);
  backward2(pwm_val);
  forward3(pwm_val);
  forward4(pwm_val);
}

void anticlockwise_rotate(uint8_t pwm_val)   //逆时针旋转
{
  forward1(pwm_val);
  forward2(pwm_val);
 backward3(pwm_val);
  backward4(pwm_val);
}

void go45(uint8_t pwm_val)   //   45度前进
{
  stop1();
  forward2(pwm_val);
  forward3(pwm_val);
  stop4();
}

void go135(uint8_t pwm_val)  // 135度前进
{
  backward1(pwm_val);
  stop2();
  stop3();
  backward4(pwm_val);
}
void go225(uint8_t pwm_val)  //   225度前进
{
  stop1();
  backward2(pwm_val);
  backward3(pwm_val);
  stop4();
}

void go315(uint8_t pwm_val) //   315度前进
{
  forward1(pwm_val);
  stop1();
  stop2();
  forward4(pwm_val);
}


#endif