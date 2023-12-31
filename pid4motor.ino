 /*读取两相编码*/
#define encodeA 2 
#define encodeB 3
/*电机引脚*/
#define motorA 11
#define motorB 10
/*霍尔传感器*/
#define A0Pin 9//编码0
#define A1Pin 8//编码1
#define A2Pin 6//编码2
/*光电传感器*/
#define optoElecPin 7
/*PID控制器*/
#define Kp 200 // 瞬态响应
#define Ki 2.0  // 稳态误差、精准度
#define Kd 2000  // 震荡
#define divisor 10 //pwm分频

/*LED*/
#define LED0Pin 14
#define LED1Pin 15  
#define LED2Pin 16
#define LED3Pin 19
#define LED4Pin 18
#define LED5Pin 17

/*电机运动参数*/ 
volatile long int nowPosition = 0; // 电机编码数，判断位置、方向
unsigned int aimPosition=360; // 调试目的值
int channelNum=1;
int reachCount=0;
bool arrived=false;
/*PID控制参数*/
int motorPwmValue = 255; // 最后控制输出的PID值
double dErr=0;
double error=0;
double lastTime=0;
double errSum=0;
double lastErr=0;
double output=0;
/*通信相关参数*/
#define baudrate 115200  //定义通讯波特率
#define slaveID 2  //定义modbus RTU从站站号
#define modbusDataSize 255  //定义modbus数据库空间大小,可根据实际情况自行修改大小
#define bufferSize 255  //一帧数据的最大字节数量
unsigned char frame[bufferSize];  //
unsigned char frame_red[bufferSize];  //
HardwareSerial* ModbusPort;
int RS485_EN_PIN=4;//操作面板也底板RS485控制口
int delay485_time=1000;//800us,通信延时释放总线时间，下位机
byte PageRead[12]; //保存内存数据
unsigned int modbusData[modbusDataSize]={};   //建立modbus数据库


/*按键*/
//#define button0Pin 25
//#define button1Pin 26
//#define button2Pin 13
//#define button3Pin 24
//#define button4Pin 23
//#define button5Pin 12
//int buttons[6] = {button0Pin, button1Pin, button2Pin, button3Pin, button4Pin, button5Pin};
void setup(){
  
  Serial.begin(115200);
  pinMode(encodeA, INPUT);
  pinMode(encodeB, INPUT);
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT); 
//  for(int i=0; i<6; i++){
//    pinMode(buttons[i], INPUT); 
//  }
//  for(int i=0; i<6; i++){
//    pinMode(LEDs[i], OUTPUT); 
//  }
  pinMode(RS485_EN_PIN,OUTPUT);  //485控制管脚
  PORTD&=~0x10; //485芯片关闭接收
  modbusRTU_INI(&Serial); 
  attachInterrupt(digitalPinToInterrupt(encodeA), ISR_encoder, RISING);
  motorInit();
}

void loop() {

  chooseChannel(channelNum);
  motorWork(nowPosition, aimPosition, 240);
  modbusRTU_slave();
  
////  串口调试
//  while (Serial.available() > 0) {
//    arrived=false;
//    reachCount=0;
//    channelNum = Serial.parseInt(); // 目标按键
//    Serial.read(); // 读取换位符
//  }
}

/*电机相关函数*/
//1、电机反馈编码
void ISR_encoder(){
  if (digitalRead(encodeB) == HIGH) 
    nowPosition++; 
  else
    nowPosition--; 
}

//2、电机运动函数
void motorWork(double nowPoint, double aimPoint, int maxPWM){
  
  if(!arrived){ //如果还没到达目标位置，持续用pid控制
    delay(1); //缓冲时间，更新太快电机没反应
    /*PID控制*/
    unsigned long now=millis();
    double timeChange=(double)(now-lastTime);
    //计算误差
    error=aimPoint-nowPoint; //误差
    errSum+=error*timeChange; //累积误差
    errSum=constrain(errSum, maxPWM*(-1.1), maxPWM*1.1); 
    dErr=(error-lastErr)/timeChange; //误差变化率
    //输出
    double newOutput=(Kp*error+Ki*errSum+Kd*dErr)/10;
    output=constrain(newOutput, maxPWM*(-1), maxPWM);
    lastErr=error;
    lastTime=now;
    /*信号到电机正反转*/
    if (output > 0) //顺时针
      motorForward(output);
    else //逆时针
      motorReserve(abs(output));  
  }
  else{ //到位之后释放pid
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}

//3、电机正反转子函数
void motorForward(int inputPwmValue) {
  if (inputPwmValue > 0) {
    analogWrite(motorA, inputPwmValue);
    digitalWrite(motorB, LOW);
  }
  else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}
void motorReserve(int inputPwmValue) {
  if (inputPwmValue > 0) {
    analogWrite(motorB, inputPwmValue);
    digitalWrite(motorA, LOW);
  }
  else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}

//4、电机通道选择
void chooseChannel(int chosenChannelNum){

  //是否到位
  if(abs(error)<2 || digitalRead(optoElecPin)==0){
    if(++reachCount>500){
      reachCount=500;
      if(digitalRead(optoElecPin)==0)
      getPosition();
      arrived=true;
    }
  }else{
    reachCount=0;
  }
 
  switch(chosenChannelNum){
    case 1:
      aimPosition=0;
      break;
    case 2:
      aimPosition=125;
      break;
    case 3:
      aimPosition=246;
      break;
    case 4:
      aimPosition=370;
      break;
    case 5:
      aimPosition=494;
      break;
    case 6:
      aimPosition=611;
      break;
    default:
      aimPosition=0;
  }
}
//5、电机位置初始化
void motorInit(){
  analogWrite(motorA, 150);
  digitalWrite(motorB, LOW);
  while(1){
    if(digitalRead(optoElecPin)==0){
      aimPosition=nowPosition;
      digitalWrite(motorA, LOW);
      digitalWrite(motorB, LOW);
      break;
    }
  }
  while(1){
      motorWork(nowPosition, aimPosition, 200);
      if(digitalRead(optoElecPin)==0){
        reachCount++;
        digitalWrite(LED0Pin,HIGH); 
      }else{
        reachCount=0;
        digitalWrite(LED0Pin,LOW); 
     }
     if(reachCount>1000){
        reachCount=1000;
        getPosition(); //停稳之后获取磁吸信号，得到对应位置。如果没停稳会跳变
        break;
     }
  }
}
void getPosition(){
  if(digitalRead(A0Pin)==1 && digitalRead(A1Pin)==0 &&digitalRead(A2Pin)==0){
    channelNum=3;
    nowPosition=246;
  }
  else if(digitalRead(A0Pin)==0 && digitalRead(A1Pin)==0 &&digitalRead(A2Pin)==1){
    channelNum=4;
    nowPosition=370;
  }
  else if(digitalRead(A0Pin)==1 && digitalRead(A1Pin)==1 &&digitalRead(A2Pin)==0){
    channelNum=5;
    nowPosition=494;
  }
  else if(digitalRead(A0Pin)==1 && digitalRead(A1Pin)==0 &&digitalRead(A2Pin)==1){
    channelNum=6;
    nowPosition=611;
  }
  else if(digitalRead(A0Pin)==0 && digitalRead(A1Pin)==1 &&digitalRead(A2Pin)==1){
    channelNum=1;
    nowPosition=0;
  }
  else if(digitalRead(A0Pin)==0 && digitalRead(A1Pin)==1 &&digitalRead(A2Pin)==0){
    channelNum=2;
    nowPosition=125;
  }
}


/*485通信相关*/
//1、初始化
void modbusRTU_INI(HardwareSerial *SerialPort)
{
  ModbusPort = SerialPort;
  (*ModbusPort).begin(baudrate);
  (*ModbusPort).flush();  
}
//2、通信从机主函数
void modbusRTU_slave()
{
  unsigned int characterTime; //字符时间
  unsigned char errorFlag=0;  //错误标志
  unsigned int crc16;  //校验位
  unsigned char address=0;
  unsigned char function=0;

  if (baudrate > 19200)  //波特率大于19200时进入条件
  {
    characterTime = 750;
  }
  else
  {
    characterTime = 15000000/baudrate;  //1.5字符时间
  }
  while((*ModbusPort).available()>0)  //如果串口缓冲区数据量大于0进入条件
  {
    if(address<bufferSize)  //接收的数据量应小于一帧数据的最大字节数量
    {
      frame_red[address]=(*ModbusPort).read();
      address++;
    }
    else  //条件不满足时直接清空缓冲区
    {
       (*ModbusPort).read();
    }
    delayMicroseconds(characterTime);  //等待1.5个字符时间
    if((*ModbusPort).available()==0)  //1.5个字符时间后缓冲区仍然没有收到数据,认为一帧数据已经接收完成,进入条件
    {
      function=frame_red[1];  //读取功能码
            
      if(frame_red[0]==slaveID||frame_red[0]==0)  //站号匹配或者消息为广播形式,进入条件
      {
        crc16 = ((frame_red[address - 2] << 8) | frame_red[address - 1]);
        if(calculateCRC(&frame_red[0],address - 2)==crc16)  //数据校验通过,进入条件
        {
          if (frame_red[0]!=0 && (function == 8))  //功能码不支持广播消息
          {
            unsigned int startData=((frame_red[2] << 8) | frame_red[3]);  //读取modbus数据库起始地址           
            unsigned int dataSize=((frame_red[4] << 8) | frame_red[5]);  //需要读取的modbus数据库数据长度
            unsigned int endData=startData+dataSize;    //需要读取的modbus数据库数据的结束地址
            unsigned char responseSize=5+dataSize*2;  //计算应答的数据长度
            unsigned int temp1,temp2,temp3;

          }
          else if(function == 16)  //功能码为06时进入条件
          {
            unsigned int startData=((frame_red[2] << 8) | frame_red[3]);  //写入modbus数据库的地址           
            unsigned int setData=((frame_red[4] << 8) | frame_red[5]);  //写入modbus数据库的数值
            if(startData>=modbusDataSize)
            {
              errorFlag=0x02;  //数据超过范围
              responseError(slaveID,function,errorFlag);  //返回错误消息
            }
          }
          else if(function == 1)  
          {
            channelNum=frame_red[2];         
            frame[0]=slaveID;  //设定站号
            frame[1]=function;  //设定功能码
            frame[2]=channelNum ;  //填写数据库地址
            frame[3]=0x00;            
            frame[4]=0x00; 
            frame[5]=0x00;            
            crc16 = calculateCRC(&frame[0],6);  //计算校验值
            frame[6] = crc16 >> 8;  //填写校验位
            frame[7] = crc16 & 0xFF;
            PORTD|=0x10;
            (*ModbusPort).write(&frame[0],8);  //返回功能码06的消息
            arrived=false;
            reachCount=0;
            delayMicroseconds(characterTime); 
            PORTD&=~0x10;
          }                                                                              
          else  //其他功能码
          {
            errorFlag = 0x01;  //不支持收到的功能码
            responseError(slaveID,function,errorFlag);  //返回错误消息
          }      
        }
        else //数据校验错误
        {
          errorFlag = 0x03;
          responseError(slaveID,function,errorFlag);  //返回错误消息
        }
      }
    }
  }
  
}
//3、错误信息返回函数
void responseError(unsigned char ID,unsigned char function,unsigned char wrongNumber)  
{
  unsigned int crc16;  //校验位  
  frame[0] = ID;  //设定站号
  frame[1] = function+0x80;
  frame[2] = wrongNumber;  //填写错误代码
  crc16 = calculateCRC(&frame[0],3);  //计算校验值
  frame[3] = crc16 >> 8;  //填写校验位
  frame[4] = crc16 & 0xFF;
  PORTD|=0x10;//
   //digitalWrite(RS485_EN_PIN,HIGH);
  (*ModbusPort).write(&frame[0],5);  //返回功能码06的消息 
   delayMicroseconds(delay485_time);
  //digitalWrite(RS485_EN_PIN,LOW);
  PORTD&=~0x10;         
}
//4、CRC校验函数
unsigned int calculateCRC(unsigned char* _regs,unsigned char arraySize)
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < arraySize; i++)
  {
    temp = temp ^ *(_regs+i);
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  return temp;
}
