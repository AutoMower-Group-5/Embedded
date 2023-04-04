#include <MeAuriga.h>
#include <SoftwareSerial.h>
#include <Wire.h>


MeGyro gyro(0, 0x69);

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

void Rotate(float d_deg, int clockwise) {
  ChangeSpeed(100);

  gyro.begin();
  gyro.update();
  float start_deg = gyro.getAngleZ();
  float deg = start_deg;

  while ((deg - start_deg) < d_deg) {
    gyro.update();
    deg = gyro.getAngleZ();

    Serial.print(deg);
    Serial.print('\t');
    Serial.print(deg - start_deg);
    Serial.print(" < ");
    Serial.println(d_deg);
    delay(10);
  }
  Serial.println("Done!");
}




void setup() {
  gyro.begin();
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

        enum state_AutoMower { Idle,
                               Locate_Path,
                               Forward_Fast,
                               Forward_Approach,
                               Colission
        };

        enum state_AutoMower state = Locate_Path;
        int distance_cm = ultraSensor.distanceCm();
        ChangeSpeed(255);

        switch (state) {

          case Idle:
            {
              break;
            }

          case Locate_Path:
            {
              Rotate(90.0, 0);
              state = Idle;
              break;
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
