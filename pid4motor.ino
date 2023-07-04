#include <PIDController.h>

#define encodeA 2 
#define encodeB 5
/*读取两相编码*/
#define motorA 9
#define motorB 10
/*电机引脚*/
#define Kp 260 // 瞬态响应
#define Ki 2.0  // 稳态误差、精准度
#define Kd 2000  // 震荡

volatile long int encodeCount = 0; // 电机编码数，判断位置、方向
unsigned int aimPosition=378; // 调试目的值
char incomingByte; // 串口位解析
int motorPwmValue = 255; // 最后控制输出的PID值
int chosenPositionNum=1;

PIDController pidcontroller;

void setup() {
  Serial.begin(9600); 
  pinMode(encodeA, INPUT); 
  pinMode(encodeB, INPUT); 
  pinMode(motorA, OUTPUT); 
  pinMode(motorB, OUTPUT); 

  attachInterrupt(digitalPinToInterrupt(encodeA), encoder, RISING);
//  attachInterrupt(1, getCurrentPosition, RISING);
  pidcontroller.begin(); // 初始化PID控制器
  pidcontroller.tune(Kp , Ki , Kd); //设置参数
  pidcontroller.limit(-255, 255); //设置PID输出范围
}

void loop() {
  while (Serial.available() > 0) {
    chosenPositionNum = Serial.parseInt(); // 目标按键
    choosePosition(chosenPositionNum);
    Serial.read(); // 读取换位符
    pidcontroller.setpoint(aimPosition); // 目标值
  }
  motorPwmValue = pidcontroller.compute(encodeCount);  //PID计算所需的值
  Serial.print(motorPwmValue); // PWN值
  Serial.print("   ");
  if (motorPwmValue > 0) //逆时针
    motorReserve(motorPwmValue);
  else // 顺时针
    motorForward(abs(motorPwmValue));
  Serial.println(chosenPositionNum); // 打印当前选择的位置
}

void encoder() {
  if (digitalRead(encodeB) == HIGH) 
    encodeCount++; 
  else 
    encodeCount--; 
}

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
void choosePosition(int chosenPosition){
  switch(chosenPosition){
    case 1:
      aimPosition=378;
      break;
    case 2:
      aimPosition=315;
      break;
    case 3:
      aimPosition=252;
      break;
    case 4:
      aimPosition=189;
      break;
    case 5:
      aimPosition=126;
      break;
    case 6:
      aimPosition=63;
      break;
    default:
      aimPosition=378;
  }
}

void getCurrentPosition(){
  
}
