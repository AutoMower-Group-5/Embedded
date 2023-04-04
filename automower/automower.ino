#include <MeAuriga.h>
#include <SoftwareSerial.h>

unsigned char table[128] = { 0 };
SoftwareSerial softuart(13, 12);
MeSerial mySerial(PORT_8);
MeUltrasonicSensor ultraSensor(PORT_7); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
int16_t moveSpeed = 200;


void SetMotors(float motor1Percent, float motor2Percent) {
  Encoder_1.setMotorPwm((motor1Percent / 100.0) * moveSpeed);
  Encoder_2.setMotorPwm((motor2Percent / 100.0) * moveSpeed);
}

void Forward(void) {
  Encoder_1.setMotorPwm(-moveSpeed);  // setMotorPwm writes to the encoder controller
  Encoder_2.setMotorPwm(moveSpeed);   // so setting the speed change instantly
}

void Backward(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void BackwardAndTurnLeft(void) {
  Encoder_1.setMotorPwm(moveSpeed / 4);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void BackwardAndTurnRight(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed / 4);
}
void TurnLeft(void) {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed / 2);
}
void TurnRight(void) {
  Encoder_1.setMotorPwm(-moveSpeed / 2);
  Encoder_2.setMotorPwm(moveSpeed);
}
void TurnLeft1(void) {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}
void TurnRight1(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}
void Stop(void) {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}
void ChangeSpeed(int16_t spd) {
  moveSpeed = spd;
}

void Rotate(int16_t deg, int clockwise) {
  ChangeSpeed(100);
  for (int i = 0; i < deg; i++) {
    TurnRight1();
    delay(10);
  }
}




void setup() {
  Serial.begin(115200);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

enum Mode { Manual,
            Auto };
enum Mode MODE = Auto;

void loop() {

  switch (MODE) {
    case Manual:
      {

        if (Serial.available()) {
          char rec_str = Serial.read();
          switch (rec_str) {
            case 'w':
              {
                Forward();
                break;
              }
            case 's':
              {
                Backward();
                break;
              }
            case 'a':
              {
                TurnLeft();
                break;
              }
            case 'd':
              {
                TurnRight();
                break;
              }
            case 'q':
              {
                Stop();
                break;
              }
          }
        }
      }
    case Auto:
      {

        enum state_AutoMower { Locate_Path,
                               Forward_Fast,
                               Forward_Approach,
                               Colission
        };

        enum state_AutoMower state = Locate_Path;
        int distance_cm = ultraSensor.distanceCm();
        ChangeSpeed(255);

        switch (state) {

          case Locate_Path:
            {
              Rotate(90, 0);
            }

          default:
            state = Locate_Path;
            break;
        }
      }
    default:
      break;
  }
}
