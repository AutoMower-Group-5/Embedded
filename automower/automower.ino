#include <MeAuriga.h>
#include <SoftwareSerial.h>

unsigned char table[128] = {0};
SoftwareSerial softuart(13,12);
MeSerial mySerial(PORT_8);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
int16_t moveSpeed = 200;

void Forward(void)
{
  Encoder_1.setMotorPwm(-moveSpeed); // setMotorPwm writes to the encoder controller
  Encoder_2.setMotorPwm(moveSpeed);  // so setting the speed change instantly
}
void Backward(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void BackwardAndTurnLeft(void)
{
  Encoder_1.setMotorPwm(moveSpeed/4);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void BackwardAndTurnRight(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed/4);
}
void TurnLeft(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed/2);
}
void TurnRight(void)
{
  Encoder_1.setMotorPwm(-moveSpeed/2);
  Encoder_2.setMotorPwm(moveSpeed);
}
void TurnLeft1(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void TurnRight1(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}
void Stop(void)
{
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}
void ChangeSpeed(int16_t spd)
{
  moveSpeed = spd;
}
void setup()
{
  Serial.begin(115200);
 
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void loop()
{
  //ChangeSpeed(100);
  //Forward();
  //delay(500);
  //Backward();
  //delay(500);
  //TurnLeft1();
  //delay(500);
  //TurnRight1();
  //delay(500);
  //Stop();
  //delay(1000);
  if(Serial.available())
  {
    char rec_str = Serial.read();
    delay(1);
    Serial.println(rec_str);

    switch(rec_str)
    {
      case 'w':
      {
        Forward();
        delay(1);
        break;
      }
      case ' ':
      {
        Stop();
        delay(1);
      }
    }
  }

}
