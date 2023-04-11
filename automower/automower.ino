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


#define AURIGARINGLEDNUM 12
#define RINGALLLEDS 0


#ifdef MeAuriga_H
// on-board LED ring, at PORT0 (onboard), with 12 LEDs
MeRGBLed led_ring(0, 12);
#endif


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

void Rotate(int16_t deg_d) {
  gyro.begin();
  gyro.update();
  float start_deg = gyro.getAngleZ();
  float deg = start_deg;
  while ((deg - start_deg) < deg_d) {
    TurnRight1();
    gyro.update();
    deg = gyro.getAngleZ();
  }
}


void SetLedRing(int r, int g, int b) {
  led_ring.setColor(RINGALLLEDS, r, g, b);
  led_ring.show();
}

int GetZAngle() {
  gyro.fast_update();
  int deg = (int)gyro.getAngleZ();
  return deg;
}



void setup() {
  gyro.begin();
  Serial.begin(115200);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

#ifdef MeAuriga_H
  // 12 LED Ring controller is on Auriga D44/PWM
  led_ring.setpin(44);
#endif
}

enum Mode { Manual,
            Auto };
enum Mode MODE = Auto;

enum state_AutoMower { Idle,
                       Locate_Path,
                       Forward_Fast,
                       Forward_Approach,
                       Colission,
                       Back
};

enum state_AutoMower state = Locate_Path;




float distance_cm = 400.0;
int deg = -200;
int start_deg = -200;
int to_deg = -200;
int16_t ROTATE_STEP_DEG = 37;

int16_t SPEED_HIGH = 255;
int16_t SPEED_MEDIUM = 150;
int16_t SPEED_SLOW = 100;

//Distance in cm
int16_t DISTANCE_LONG = 50;
int16_t DISTANCE_MEDIUM = 30;
int16_t DISTANCE_SHORT = 15;
int16_t DISTANCE_COLISSION = 5;

int16_t LED_BRIGHTNESS = 10;

int LOOP_PERIOD_MS = 100;
int DEBUG = 0;

void ChangeMowerState(int new_state) {
  state = new_state;

  switch (new_state) {
    default:
      {
        ChangeMowerState(Idle);
        break;
      }
    case Idle:
      {
        SetLedRing(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS);
        break;
      }
    case Locate_Path:
      {
        Stop();
        start_deg = deg;
        ChangeSpeed(SPEED_MEDIUM);
        SetLedRing(LED_BRIGHTNESS, 0, 0);
        break;
      }
    case Forward_Fast:
      {
        ChangeSpeed(SPEED_HIGH);
        SetLedRing(0, LED_BRIGHTNESS, 0);
        break;
      }
    case Forward_Approach:
      {
        ChangeSpeed(SPEED_MEDIUM);
        SetLedRing(LED_BRIGHTNESS, LED_BRIGHTNESS, 0);
        break;
      }
    case Colission:
      {
        Stop();
        SetLedRing(LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
        break;
      }
    case Back:
      {
        ChangeSpeed(SPEED_MEDIUM);
        SetLedRing(LED_BRIGHTNESS, 0, LED_BRIGHTNESS);
        break;
      }
  }
}




void loop() {

  delay(LOOP_PERIOD_MS);
  switch (MODE) {
    case Manual:
      {
        break;
      }
    case Auto:
      {

        distance_cm = ultraSensor.distanceCm();
        deg = GetZAngle();
        if(DEBUG){
          Serial.print("Distance: ");
          Serial.print(distance_cm);
          Serial.print("\tDegree: ");
          Serial.print(deg);
          Serial.print(" start_deg: ");
          Serial.print(start_deg);
          Serial.print(" to_deg: ");
          Serial.print(to_deg);
          Serial.print("\tState: ");
          Serial.println(state);
        }
        

        switch (state) {
          default:
            {
            }
          case Idle:
            {
              ChangeMowerState(Locate_Path);
              break;
            }

          case Locate_Path:
            {
              if ((to_deg != -200) && (deg < to_deg)) {
                TurnRight1();

              } else if ((distance_cm <= DISTANCE_MEDIUM) || (distance_cm >= 400.0)) {
                Stop();
                gyro.begin();
                start_deg = GetZAngle();
                to_deg = (start_deg + ROTATE_STEP_DEG) % 360;
              } else {
                Stop();
                start_deg = -200;
                to_deg = -200;
                ChangeMowerState(Forward_Fast);
              }

              break;
            }
          case Forward_Fast:
            {
              if (distance_cm < DISTANCE_LONG) {
                ChangeMowerState(Forward_Approach);
              }
              Forward();
              break;
            }

          case Forward_Approach:
            {
              if (distance_cm >= DISTANCE_LONG) {

                ChangeMowerState(Forward_Fast);

              } else if (distance_cm > DISTANCE_MEDIUM) {
                ChangeSpeed(SPEED_MEDIUM);
              } else if (distance_cm > DISTANCE_SHORT) {
                ChangeSpeed(SPEED_SLOW);
              } else if (distance_cm <= DISTANCE_COLISSION || (distance_cm >= 400.0)) {
                ChangeMowerState(Colission);
              }
              Forward();
              break;
            }

          case Colission:
            {
              ChangeMowerState(Back);
            }
          case Back:
            {
              if (distance_cm >= DISTANCE_SHORT) {
                ChangeMowerState(Locate_Path);
              }
              Backward();
              break;
            }
        }
        break;
      }
  }
}
