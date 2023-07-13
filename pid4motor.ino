#include <Wire.h>
#include <EEPROM.h>
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
/*光电传感器*/
#define optoElecPin 7
/*通信相关*/
#define baudrate 115200  //定义通讯波特率
#define slaveID 1  //定义modbus RTU从站站号
#define modbusDataSize 255  //定义modbus数据库空间大小,可根据实际情况自行修改大小
#define bufferSize 255  //一帧数据的最大字节数量
#define RS485EnPIN 4

/*电机运动相关*/
volatile double nowPosition=0; // 电机当前编码数
double aimPosition=0; // 电机目标编码数
int motorPwmValue = 255; // 最后控制输出的PID值
int chosenChannelNum=1; //通道选择
bool initialized=false; //初始化完成标志
/*Modbus数据相关*/
unsigned int modbusData[modbusDataSize]={};   //建立modbus数据库
unsigned char returnCodes[bufferSize];  //返回主机的数据码
unsigned char receiveCodes[bufferSize];  //从主机接收的数据码
HardwareSerial* ModbusPort;
int delay485_time=1000;//800us,通信延时释放总线时间，下位机
int selectedChannel;
int EEPROM_ADDR=0;//起始地址
int EEPROM_NUM=6;//保存内存个数
byte PageRead[12]; //保存内存数据
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
  delay(500);
  Serial.begin(9600); 
  pinMode(encodeA, INPUT); 
  pinMode(encodeB, INPUT); 
  pinMode(motorA, OUTPUT); 
  pinMode(motorB, OUTPUT); 

  attachInterrupt(digitalPinToInterrupt(encodeA), ISR_encoder, RISING);
  motorInit();
  modbusRTU_INI(ModbusPort);
}

void loop() {
  while (Serial.available() > 0) {
    chosenChannelNum = Serial.parseInt(); // 目标按键
    Serial.read(); // 读取换位符
  }
  chooseChannel(chosenChannelNum);
  motorWork(aimPosition, nowPosition);
}



/*电机相关函数*/
// 电机初始化
void motorInit(){
  while(!initialized){
    if(digitalRead(optoElecPin)==1){
      getPosition();
      aimPosition=nowPosition;
      initialized=true;
    }
    motorWork(aimPosition, nowPosition);
  }
}
// 电机运动
void motorWork(double aimPositionInput, double nowPositionInput){
  motorPwmValue=PIDController(aimPositionInput, nowPositionInput);
  if (motorPwmValue > 0) //逆时针
    motorReserve(motorPwmValue);
  else // 顺时针
    motorForward(abs(motorPwmValue));
  Serial.println(chosenChannelNum); // 打印当前选择的位置
}
// 电机动作单元
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
// 获取光电位置
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
// 选择位置通道
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
//电机编码中断服务
void ISR_encoder() {
  if (digitalRead(encodeB) == HIGH) 
    nowPosition++; 
  else 
    nowPosition--; 
}

/*pid控制器*/
double PIDController(double aimPoint, double nowPoint){
  //计算时间差
  unsigned long now=millis();
  double timeChange=(double)(now-lastTime);
  //计算误差
  double error=aimPoint-nowPoint; //误差
  errSum+=error*timeChange; //累积误差
  errSum=constrain(errSum, minOut*1.1, maxOut*1.1); 
  double dErr=(error-lastErr)/timeChange; //误差变化率
  //输出
  double newOutput=(Kp*error+Ki*errSum+Kd*dErr)/divisor;
  output=constrain(newOutput, minOut, maxOut);//限制输出
//  output=newOutput;//无限制输出
  lastErr=error;
  lastTime=now;
  return output;
}

/*RS485-Modbus_RTU通信*/
//从机应答函数
void modbusRTU_slave()
{
  unsigned int characterTime; //字符时间
  unsigned char errorFlag=0;  //错误标志
  unsigned int crc16;  //校验位
  unsigned char address=0;
  //通过波特率设置传输时间
  if (baudrate > 19200){
    characterTime = 750;
  }else{
    characterTime = 15000000/baudrate;
  }

  while((*ModbusPort).available()>0){  //如果串口缓冲区数据量大于0进入条件
    if(address<bufferSize){  //接收的数据量应小于一帧数据的最大字节数量
      receiveCodes[address]=(*ModbusPort).read();
      address++;
    }
    else{  //条件不满足时直接清空缓冲区
       (*ModbusPort).read();
    }
    delayMicroseconds(characterTime);  //等待1.5个字符时间
    if((*ModbusPort).available()==0){  //1.5个字符时间后缓冲区仍然没有收到数据,认为一帧数据已经接收完成,进入条件
      unsigned char function=receiveCodes[1];  //读取功能码
      if(receiveCodes[0]==slaveID){
        crc16 = ((receiveCodes[address - 2] << 8) | receiveCodes[address - 1]);
        if(calculateCRC(&receiveCodes[0],address - 2)==crc16){  //数据校验通过,进入条件
          if(function == 1){  //进入电机通道控制
            selectedChannel=receiveCodes[2];        
            returnCodes[0]=slaveID;  //设定站号
            returnCodes[1]=function;  //设定功能码
            returnCodes[2]=selectedChannel; //返回通道数
            returnCodes[3]=0x00;            
            returnCodes[4]=0x00;  //填写校验位;  //填写数据库数值
            returnCodes[5]=0x00;            
            crc16 = calculateCRC(&returnCodes[0],6);  //计算校验值
            returnCodes[6]=crc16>>8;  //填写校验位
            returnCodes[7]=crc16&0xFF;
            digitalWrite(RS485EnPIN, HIGH);
            (*ModbusPort).write(&returnCodes[0],8);  //返回功能码01的消息 
            digitalWrite(RS485EnPIN, LOW);
          }else{  //其他功能码
            errorFlag = 0x01;  //不支持收到的功能码
            responseError(slaveID,function,errorFlag);  //返回错误消息
          }      
        }
        else{ //数据校验错误
          errorFlag = 0x03;
          responseError(slaveID,function,errorFlag);  //返回错误消息
        }
      }
    }
  }
}
//错误信息返回函数
void responseError(unsigned char ID,unsigned char function,unsigned char wrongNumber){
  unsigned int crc16;  //校验位  
  returnCodes[0] = ID;  //设定站号
  returnCodes[1] = function+0x80;
  returnCodes[2] = wrongNumber;  //填写错误代码
  crc16 = calculateCRC(&returnCodes[0],3);  //计算校验值
  returnCodes[3] = crc16 >> 8;  //填写校验位
  returnCodes[4] = crc16 & 0xFF;
  digitalWrite(RS485EnPIN, HIGH);
  (*ModbusPort).write(&returnCodes[0],5);  //返回功能码01的消息 
  delayMicroseconds(delay485_time);
  digitalWrite(RS485EnPIN, LOW);    
}
//CRC校验函数
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
// Modbus初始化
void modbusRTU_INI(HardwareSerial *SerialPort)
{
  ModbusPort = SerialPort;
  (*ModbusPort).begin(baudrate);
  (*ModbusPort).flush();  
}
