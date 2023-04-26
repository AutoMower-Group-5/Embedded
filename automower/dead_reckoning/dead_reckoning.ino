#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <MeAuriga.h>
#include "Timer.h"

Timer timer;

int16_t moveSpeed = 255;
int timer_start = 0;
int timer_end = 0;

float position_x = 0;
float position_y = 0;
double angleZ = 0;


MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeGyro gyro(0, 0x69);

void SetMotors(float motor1Percent, float motor2Percent) {
  Encoder_1.setMotorPwm((motor1Percent / 100.0) * moveSpeed);
  Encoder_2.setMotorPwm((motor2Percent / 100.0) * moveSpeed);
}

//enum status_t {STOPPED, RUNNING, PAUSED};

void UpdatePos(){

  angleZ = gyro.getAngleZ() * 0.0174532925;

  float hypo = ((float)moveSpeed/255.0)*(float)(timer_end - timer_start)/1000;
  position_x += hypo*cos(angleZ);
  position_y += hypo*sin(angleZ);

  Serial.print("Angle:");
  Serial.print(angleZ);
  Serial.print(",");
  Serial.print("X_POS:");
  Serial.print(position_x * 100);
  Serial.print(",");
  Serial.print("Y_POS:");
  Serial.println(position_y * 100);
}

void SetSpeed(int newSpeed){
  timer_end = timer.read();
  timer.stop();
  UpdatePos();

  if(newSpeed > 255)
    newSpeed = 255;
  else if(newSpeed < -255)
    newSpeed = -255;
      
  moveSpeed = newSpeed;
  Encoder_1.setMotorPwm(-moveSpeed); 
  Encoder_2.setMotorPwm(moveSpeed);

  timer.start();
  timer_start = timer.read();  
}

void MoveBot(int direction){ //Forward = 1, Backward = -1
  SetSpeed(direction*moveSpeed); 
}

void StopBot(void) {
  SetSpeed(0);
}

void setup() {
  gyro.begin();
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}



void loop() {
  gyro.update();
  // put your main code here, to run repeatedly:
  SetSpeed(255);
  delay(10);
}
