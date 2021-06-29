#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include "DCMotor.h"
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_2 true
#include "TimerInterrupt.h"

#define Kp 35
#define Ki 65
#define Kd 0.5
#define sampleTime 0.005
#define targetAngle -2.5

MPU6050 mpu;

DCMotor M1;
DCMotor M2;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
//volatile byte count=0;

void setup() {
  Serial.begin(250000);
  ITimer2.init();
  ITimer2.setInterval(5L, pid, 0);
  M1.Pinout(6,5);
  M2.Pinout(9,10);
  
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setXAccelOffset(-1312);
  mpu.setYAccelOffset(76);
  mpu.setZAccelOffset(1872);
  mpu.setXGyroOffset(110);
  mpu.setYGyroOffset(16);
  mpu.setZGyroOffset(0);

}

void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  // set motor power after constraining it

  if (abs(motorPower) >= 255) digitalWrite(13, HIGH);
  else digitalWrite(13, LOW);
  Serial.print(currentAngle);
  Serial.print(' ');
  Serial.println(motorPower);
  
  if (motorPower > 0){
    M1.Forward();
    M2.Forward();
  }
  else{
    M1.Backward();
    M2.Backward();
  }
  M1.Speed(abs(motorPower));
  M2.Speed(abs(motorPower));
  
}
// The ISR will be called every 5 milliseconds
void pid(){

     // calculate the angle of inclination
    accAngle = atan2(accY, accZ)*RAD_TO_DEG-3;
    gyroRate = map(gyroX, -32768, 32767, -250, 250);
    gyroAngle = (float)gyroRate*sampleTime;  
    currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
    error = currentAngle - targetAngle;
    errorSum = errorSum + error;  
    errorSum = constrain(errorSum, -300, 300);
    //calculate output from P, I and D values
    motorPower = Kp*(error) + Ki*(errorSum)*sampleTime + Kd*(currentAngle-prevAngle)/sampleTime;
    prevAngle = currentAngle;
  
//  count++;
//  if(count == 200)  {
//    count = 0;
//     digitalWrite(13, !digitalRead(13));
//  }
}
