/*读取两相编码*/
#define encodeA 2 
#define encodeB 5
/*电机引脚*/
#define motorA 9
#define motorB 10
/*霍尔传感器*/
#define A0Pin 23 //编码0
#define A1Pin 24 //编码1
#define A2Pin 11 //编码2
        
volatile double nowPosition = 0; // 电机编码数，判断位置、方向
double aimPosition=0;; // 目的值
int motorPwmValue = 255; // 最后控制输出的PID值
int chosenChannelNum=1; //通道选择
bool initialized=false; //初始化完成标志

/*pid控制器参数*/
double Kp=260;// 瞬态响应
double Ki=2.0;// 稳态误差、精准度
double Kd=2000;// 震荡
double divisor=10;
double minOut=-255;
double maxOut=255;
double output=0;
double lastErr=0;
double errSum=0;
unsigned long lastTime=0;


void setup() {
  Serial.begin(9600); 
  pinMode(encodeA, INPUT); 
  pinMode(encodeB, INPUT); 
  pinMode(motorA, OUTPUT); 
  pinMode(motorB, OUTPUT); 

  attachInterrupt(digitalPinToInterrupt(encodeA), encoder, RISING);
  attachInterrupt(1, motorInit, RISING);
}

void loop() {
  while (Serial.available() > 0) {
    chosenChannelNum = Serial.parseInt(); // 目标按键
    Serial.read(); // 读取换位符
  }
  chooseChannel(chosenChannelNum);
  motorPwmValue=PIDController(aimPosition, nowPosition);
  if (motorPwmValue > 0) //逆时针
    motorReserve(motorPwmValue);
  else // 顺时针
    motorForward(abs(motorPwmValue));
  Serial.println(chosenChannelNum); // 打印当前选择的位置
}

void encoder() {
  if (digitalRead(encodeB) == HIGH) 
    nowPosition++; 
  else 
    nowPosition--; 
}

void motorForward(int inputPwmValue) {
  if (inputPwmValue > 50) { //消除震荡
    analogWrite(motorA, inputPwmValue);
    digitalWrite(motorB, LOW);
  }
  else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}
void motorReserve(int inputPwmValue) {
  if (inputPwmValue > 50) {
    analogWrite(motorB, inputPwmValue);
    digitalWrite(motorA, LOW);
  }
  else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}

/*光电获取初始化*/
void motorInit(){
  if(initialized==false){
    getPosition();
    aimPosition=nowPosition;
    initialized=true;
  }
}

void getPosition(){
  if(digitalRead(A0Pin)==1 && digitalRead(A1Pin)==1 &&digitalRead(A2Pin)==0){
    nowPosition=0;
  }
  else if(digitalRead(A0Pin)==1 && digitalRead(A1Pin)==0 &&digitalRead(A2Pin)==1){
    nowPosition=63;
  }
  else if(digitalRead(A0Pin)==0 && digitalRead(A1Pin)==1 &&digitalRead(A2Pin)==1){
    nowPosition=126;
  }
  else if(digitalRead(A0Pin)==0 && digitalRead(A1Pin)==1 &&digitalRead(A2Pin)==0){
    nowPosition=189;
  }
  if(digitalRead(A0Pin)==1 && digitalRead(A1Pin)==0 &&digitalRead(A2Pin)==0){
    nowPosition=252;
  }
  else if(digitalRead(A0Pin)==0 && digitalRead(A1Pin)==0 &&digitalRead(A2Pin)==1){
    nowPosition=315;
  }
}



void chooseChannel(int inputChannel){
  switch(inputChannel){
    case 1:
      aimPosition=0;
      break;
    case 2:
      aimPosition=63;
      break;
    case 3:
      aimPosition=126;
      break;
    case 4:
      aimPosition=189;
      break;
    case 5:
      aimPosition=252;
      break;
    case 6:
      aimPosition=315;
      break;
    default:
      aimPosition=0;
  }
}

/*pid控制器*/
double PIDController(double aimPoint, double nowPoint){
  //计算时间差
  unsigned long now=millis();
  double timeChange=(double)(now-lastTime);
  
  double error=aimPoint-nowPoint; //误差
  errSum+=error*timeChange; //累积误差
  //限制累积误差
  errSum=constrain(errSum, minOut*1.1, maxOut*1.1); 
  double dErr=(error-lastErr)/timeChange; //误差变化率

  double newOutput=(Kp*error+Ki*errSum+Kd*dErr)/divisor;
  output=constrain(newOutput, minOut, maxOut);
//  output=newOutput;

  lastErr=error;
  lastTime=now;
  return output;
}
