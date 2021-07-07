#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include "DCMotor.h"
#include <stdio.h>
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_2 true
#include "TimerInterrupt.h"
#define sampleTime 0.005

MPU6050 mpu;

DCMotor M1;
DCMotor M2;

float targetAngle = -.4;
float Kp = 35;
float Ki = 65;
float Kd = 0.5;
int offset1=0, offset2=0;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
bool secureLock = true;

byte buffer[] = {0,0,0,0,0,0};
bool motorEnable = true;
bool dataStream = false;

/** SERIAL COMMANDS
 * A: set motor enable (M1xx\n or M1xx\n)
 * a: get motor state
 * S: set data stream (D1xx\n or D0xx\n)
 * s: get data stream state
 * T: set target angle (float)
 * t: get target angle
 * P: set proporcional gain (float)
 * p: get proporcional gain
 * I: set integral gain (float)
 * i: get integral gain
 * D: set derivative gain (float)
 * d: get derivative gain
 * M: set motor offsets   byte[0]<<0|byte[1]<<8, y = byte[2]<<0|byte[3]<<8
 * m: get motor offsets
 */


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

void send(byte b0, byte b1, byte b2, byte b3, byte b4){
  Serial.write(b0);
  Serial.write(b1);
  Serial.write(b2);
  Serial.write(b3);
  Serial.write(b4);
  Serial.write((byte)(b0+b1+b2+b3+b4%256));
}

void send(byte b0, byte* bytes){
  Serial.write(b0);
  Serial.write(bytes[0]);
  Serial.write(bytes[1]);
  Serial.write(bytes[2]);
  Serial.write(bytes[3]);
  Serial.write((byte)(b0+bytes[0]+bytes[1]+bytes[2]+bytes[3]%256));
}

byte buffer_checksum(){
  return (byte)(buffer[0]+buffer[1]+buffer[2]+buffer[3]+buffer[4]%256);
}

void loop() {
  
  if (Serial.available()){
    for (byte i = 0; i < 5; i++)
      buffer[i] = buffer[i+1];  
    buffer[5] = Serial.read();
    //Serial.write(buffer[5]);
    if (buffer[5] == buffer_checksum()){
      char comando = buffer[0];
      float value;
      byte* bytes = (byte*)&value;
      for (byte i = 0; i < 4; i++)
        bytes[i] = buffer[1+i];
      switch(comando){
        case 'A':
          motorEnable = bytes[0] == '1';
        case 'a':
          send('a', motorEnable, motorEnable, motorEnable, motorEnable);
        break;
        case 'S':
          dataStream = bytes[0] == '1';
        case 's':
          send('s', dataStream, dataStream, dataStream, dataStream);
        break;
        case 'T':
          targetAngle = value;
        case 't':
          send('t',(byte*)&targetAngle);
        break;
        case 'P':
          Kp = value;
        case 'p':
          send('p',(byte*)&Kp);
        break;
        case 'I':
          Ki = value;
        case 'i':
          send('i',(byte*)&Ki);
        break;
        case 'D':
          Kd = value;
        case 'd':
          send('d',(byte*)&Kd);
        break;
        case 'M':
          offset1 = bytes[0]<<0|bytes[1]<<8;
          offset2 = bytes[2]<<0|bytes[3]<<8;
        break;
        case 'o':
          send('m',*((byte*)&offset1),*((byte*)(&offset1+1)),*((byte*)&offset2),*((byte*)(&offset2+1)));
        break;
        case '0':
          send('0',(byte*)&currentAngle);
        break;
        case '1':
          send('1',*((byte*)&motorPower), *((byte*)(&motorPower+1)), 0x0, 0x0);
        break;
      }
    }
  }
  
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  // set motor power after constraining it

  if (dataStream) send('0',(byte*)&currentAngle);
  
  if (secureLock && abs(accAngle) < 2){
    secureLock = false;
  }
  else if (abs(currentAngle) > 80){
    secureLock = true;
  }
  
  if (secureLock || !motorEnable){
    M1.Stop();
    M2.Stop();
  }
  else{
    int m1 = motorPower+offset1;
    int m2 = motorPower+offset2;
    if (m1 > 0) M1.Forward();
    else M1.Backward();
    M1.Speed(abs(m1));
    if (m2 > 0) M2.Forward();
    else M2.Backward();
    M2.Speed(abs(m2));
    if (dataStream) send('1',*((byte*)&m1), *((byte*)(&m1+1)),*((byte*)&m2), *((byte*)(&m2+1)));
  }
}
// The ISR will be called every 5 milliseconds
void pid(){

     // calculate the angle of inclination
    accAngle = atan2(accY, accZ)*RAD_TO_DEG;
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
