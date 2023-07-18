/*读取两相编码*/
#define encodeA 2 
#define encodeB 3
/*电机引脚*/
#define motorA 11
#define motorB 10
/*霍尔传感器*/
#define A0_pin 9;//编码0
#define A1_pin 8;//编码1
#define A2_pin 6;//编码2
/*光电传感器*/
#define optoElecPin 7
/*PID控制器*/
#define Kp 200 // 瞬态响应
#define Ki 2.0  // 稳态误差、精准度
#define Kd 2000  // 震荡
#define divisor 10 //pwm分频
/*电机运动参数*/ 
volatile long int nowPosition = 0; // 电机编码数，判断位置、方向
unsigned int aimPosition=315; // 调试目的值
char incomingByte; // 串口位解析
int channelNum=1;
int initValue=0;
int speedLimit=255;
/*PID控制参数*/
int motorPwmValue = 255; // 最后控制输出的PID值
double lastTime=0;
double errSum=0;
double lastErr=0;
double output=0;


void setup(){
  
  Serial.begin(9600); 
  pinMode(encodeA, INPUT); 
  pinMode(encodeB, INPUT); 
  pinMode(motorA, OUTPUT); 
  pinMode(motorB, OUTPUT); 

  attachInterrupt(digitalPinToInterrupt(encodeA), ISR_encoder, RISING);
  motorInit();
}

void loop() {
  
  while (Serial.available() > 0) {
    channelNum = Serial.parseInt(); // 目标按键
    chooseChannel(channelNum);
    Serial.read(); // 读取换位符
  }
  motorWork(nowPosition, aimPosition, speedLimit);
}

/*电机相关函数*/
//1、电机反馈编码
void ISR_encoder(){
  if (digitalRead(encodeB) == HIGH) 
    nowPosition++; 
  else 
    nowPosition--; 
}
//2、电机pid运动
void motorWork(double nowPoint, double aimPoint, int maxPWM){
  delay(5);
  /*PID控制*/
  unsigned long now=millis();
  double timeChange=(double)(now-lastTime);
  //计算误差
  double error=aimPoint-nowPoint; //误差
  errSum+=error*timeChange; //累积误差
  errSum=constrain(errSum, maxPWM*(-1.1), maxPWM*1.1); 
  double dErr=(error-lastErr)/timeChange; //误差变化率
  //输出
  double newOutput=(Kp*error+Ki*errSum+Kd*dErr)/divisor;
  output=constrain(newOutput, maxPWM*(-1), maxPWM);//限制输出
  //  output=newOutput;//无限制输出

  /*信号到电机正反转*/
  lastErr=error;
  lastTime=now;
  if (output > 0) //顺时针
    motorReserve(output);
  else //逆时针
    motorForward(abs(output));  

}
//3、电机正反转
void motorForward(int power) {
  if (power > 50) {
    analogWrite(motorA, power);
    digitalWrite(motorB, LOW);
  }
  else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}
void motorReserve(int power) {
  if (power > 50) {
    analogWrite(motorB, power);
    digitalWrite(motorA, LOW);
  }
  else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}
//4、选择电机通道
void chooseChannel(int chosenPosition){
  switch(chosenPosition){
    case 1:
      aimPosition=initValue+378;
      break;
    case 2:
      aimPosition=initValue+315;
      break;
    case 3:
      aimPosition=initValue+252;
      break;
    case 4:
      aimPosition=initValue+189;
      break;
    case 5:
      aimPosition=initValue+126;
      break;
    case 6:
      aimPosition=initValue+63;
      break;
    default:
      aimPosition=initValue+378;
  }
}
//电机位置初始化
void motorInit(){
  while(1){
    motorWork(nowPosition, aimPosition, 100);
    if(digitalRead(optoElecPin)==0){
      aimPosition=nowPosition;
      break;
    }
  }
}
