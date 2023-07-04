#include <PIDController.h>

#define encodeA 2 
#define encodeB 5
/*读取两相编码*/
#define motorA 9
#define motorB 10

/*In this section we have defined the gain values for the 
 * proportional, integral, and derivative controller I have set
 * the gain values with the help of trial and error methods.
*/ 
#define Kp 260 // Proportional constant
#define Ki 2.7  // Integral Constant
#define Kd 2000  // Derivative Constant

volatile long int encodeCount = 0; // stores the current encoder count
unsigned int integerValue = 0; // stores the incoming serial value. Max value is 65535
char incomingByte; // parses and stores each character one by one
int motorPwmValue = 255; // after PID computation data is stored in this variable.
PIDController pidcontroller;

void setup() {
  Serial.begin(9600); // Serial for Debugging
  pinMode(encodeA, INPUT); // encodeA as Input
  pinMode(encodeB, INPUT); // encodeB as Input
  pinMode(motorA, OUTPUT); // motorA as Output
  pinMode(motorB, OUTPUT); // motorA as Output
/* attach an interrupt to pin encodeA of the Arduino, and when the pulse is in the RISING edge called the function encoder().
*/
  attachInterrupt(digitalPinToInterrupt(encodeA), encoder, RISING);
  pidcontroller.begin(); // initialize the PID instance
  pidcontroller.tune(Kp , Ki , Kd); // Tune the PID, arguments: kP, kI, kD
  pidcontroller.limit(-255, 255); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!
}

void loop() {
  while (Serial.available() > 0) {
    integerValue = Serial.parseInt(); // stores the integerValue
    incomingByte = Serial.read(); // stores the /n character
    pidcontroller.setpoint(integerValue); // The "goal" the PID controller tries to "reach",
    Serial.println(integerValue); // print the incoming value for debugging
    if (incomingByte == '\n') // if we receive a newline character we will continue in the loop
      continue;
  }
  motorPwmValue = pidcontroller.compute(encodeCount);  //Let the PID compute the value, returns the calculated optimal output
  Serial.print(motorPwmValue); // print the calculated value for debugging
  Serial.print("   ");
  if (motorPwmValue > 0) // if the motorPwmValue is greater than zero we rotate the  motor in clockwise direction
    motorReserve(motorPwmValue);
  else // else, we move it in a counter-clockwise direction
    motorForward(abs(motorPwmValue));
  Serial.println(encodeCount);// print the final encoder count.
}

void encoder() {
  if (digitalRead(encodeB) == HIGH) // if encodeB is high increase the count
    encodeCount++; // increment the count
  else // else decrease the count
    encodeCount--; // decrement the count
}

void motorForward(int power) {
  if (power > 100) {
    analogWrite(motorA, power);
    digitalWrite(motorB, LOW);
  }
// both of the pins are set to low
  else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}

void motorReserve(int power) {
  if (power > 100) {
    analogWrite(motorB, power);
    digitalWrite(motorA, LOW);
  }
  else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }
}
