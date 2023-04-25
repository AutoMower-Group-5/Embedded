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

void SetMotors(float motor1Percent, float motor2Percent) {
  Encoder_1.setMotorPwm((motor1Percent / 100.0) * moveSpeed);
  Encoder_2.setMotorPwm((motor2Percent / 100.0) * moveSpeed);
}

//enum status_t {STOPPED, RUNNING, PAUSED};

void UpdatePos(){
  float hypo = ((float)moveSpeed/255.0)*(float)(timer_end - timer_start)/1000;
  position_x += hypo*cos(angle);
  position_y += hypo*sin(angle);
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

void Stop(void) {
  SetSpeed(0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}



void loop() {
  // put your main code here, to run repeatedly:
  SetSpeed(255);
  delay(2000);
  Serial.print("position_x 1:  ");
  Serial.println(position_x);

  SetSpeed(127);
  delay(2000);
  Serial.print("position_x 2:  ");
  Serial.println(position_x);
  
  SetSpeed(-255);
  delay(500);
  Serial.print("position_x 3: ");
  Serial.println(position_x);
}
