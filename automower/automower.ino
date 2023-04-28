#include <MeAuriga.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include "Timer.h"

Timer timer;
Timer postition_timer;

MeGyro gyro(0, 0x69);
MeGyro position_gyro(0, 0x69);

unsigned char table[128] = { 0 };
SoftwareSerial softuart(13, 12);
MeSerial mySerial(PORT_8);

MeUltrasonicSensor ultraSensor(PORT_7); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
MeLineFollower lineFinder(PORT_9);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
int16_t moveSpeed = 0;


#define AURIGARINGLEDNUM 12
#define RINGALLLEDS 0


#ifdef MeAuriga_H
// on-board LED ring, at PORT0 (onboard), with 12 LEDs
MeRGBLed led_ring(0, 12);
#endif

//States
enum Mode { Manual,
            Auto };
enum Mode MODE = Auto;

enum state_AutoMower {
  Idle,
  Locate_Path,
  Away_From_Line,
  Forward_Fast,
  Forward_Approach,
  Colission,
  Back
};

enum state_ManualMower {
  Stopped,
  Forwards,
  Forwards_Left,
  Forwards_Right,
  Backwards,
  Backwards_Left,
  Backwards_Right,
  Right,
  Left
};
enum state_Colission {
  c_stop,
  c_rotate,
  c_resume,
  c_idle
};

enum state_AutoMower state_auto = Idle;
enum state_ManualMower state_manual = Stopped;
enum state_Colission state_colission = c_idle;

//Communication

const char* MSG_TO_LIDAR_STOP = "A:STOP\n";
const char* MSG_TO_LIDAR_OK = "A:OK\n";
const int   MSG_TO_LIDAR_N_INT = 3;

const char* MSG_FROM_LIDAR_DETECT = "DET";
const char* MSG_FROM_LIDAR_DONE = "DON";
const char* MSG_FROM_LIDAR_RIGHT = "RHT";
const char* MSG_FROM_LIDAR_LEFT = "LFT";



//Configuration

const int DEBUG = 0;


const int SPEED_FACTOR = 1 + DEBUG;
const int16_t SPEED_HIGH    = 100/SPEED_FACTOR;
const int16_t SPEED_MEDIUM  = 75/SPEED_FACTOR;
const int16_t SPEED_SLOW    = 50/SPEED_FACTOR;

const int16_t SPEED_MANUAL = SPEED_MEDIUM;
const int16_t SPEED_ROTATION = SPEED_HIGH;



const int16_t LED_BRIGHTNESS = 10;

const int SERIAL_DELAY_MS = 20;

const int WALL_DETECTION_ACTIVE   = 0;
  const int16_t ROTATE_AVOIDANCE_DEG  = 37;
  const int16_t DISTANCE_LONG       = 50;
  const int16_t DISTANCE_MEDIUM     = 30;
  const int16_t DISTANCE_SHORT      = 15;
  const int16_t DISTANCE_COLISSION  = 5;

const int LINE_DETECTION_ACTIVE   = 1; 
  const int BACK_WHEN_LINE_DETECED_MS = 500;
  const int16_t ROTATE_LINE_DEG       = 97;

const int LINE_IS_BLACK = 1;

int is_backing = 0;


void SetMotors(float motor1Percent, float motor2Percent) {
  Encoder_1.setMotorPwm((motor1Percent / 100.0) * moveSpeed);
  Encoder_2.setMotorPwm((motor2Percent / 100.0) * moveSpeed);
}

//void Forward(void) {
//  is_backing = 0;
//  Encoder_1.setMotorPwm(-moveSpeed);  // setMotorPwm writes to the encoder controller
//  Encoder_2.setMotorPwm(moveSpeed);   // so setting the speed change instantly
//}
//
//void Backward(void) {
//  is_backing = 1;
//  Encoder_1.setMotorPwm(moveSpeed);
//  Encoder_2.setMotorPwm(-moveSpeed);
//}
//
//
//
//
//
//
//
//
//void Stop(void) {
//  Encoder_1.setMotorPwm(0);
//  Encoder_2.setMotorPwm(0);
//}

void SetLedRing(int r, int g, int b) {
  led_ring.setColor(RINGALLLEDS, r, g, b);
  led_ring.show();
}

int GetZAngle() {
  gyro.fast_update();
  int deg = (int)gyro.getAngleZ();
  return deg;
}



char* substring(char* destination, const char* source, int beg, int n) {
  while (n > 0) {
    *destination = *(source + beg);

    destination++;
    source++;
    n--;
  }

  *destination = '\0';

  return destination;
}


void int_to_arr(char* res, int input, const int depth){
   int i = 0;
   for(; i < depth; i++){
       res[i] = '0' + (input/(int)pow(10, depth-1 -i)) % 10;
   }
   res[i] = '\0';
}

void lidar_pos_msg(char* res, int x, int y){
    char* start = "A:POS:";
    char x_str[5];
    char y_str[5];
    
    int_to_arr(x_str, x, MSG_TO_LIDAR_N_INT);
    int_to_arr(y_str, y, MSG_TO_LIDAR_N_INT);
    
    int res_ix = 0;
    for(; start[res_ix] != '\0'; res_ix++){
        res[res_ix] = start[res_ix];
    }
    for(int ix = 0; x_str[ix] != '\0'; ix++, res_ix++){
        res[res_ix] = x_str[ix];
    }
    res[res_ix++] = ':';
    for(int ix = 0; y_str[ix] != '\0'; ix++, res_ix++){
        res[res_ix] = y_str[ix];
    }
    res[res_ix++] = '\n';
    res[res_ix] = '\0';
    
    
}






int colission_rotate_deg = 0;
char colission_rotate_dir = ' ';

float distance_cm = 400.0;

int line_sensor_r = 0;
int line_sensor_l = 0;
char line_direction = ' ';

int deg = -200;
int start_deg = -200;
int to_deg = -200;

int backing_stop_time = 0;

float position_x = 0;
float position_y = 0;
float position_angle = 0;

//Dead reckoning
double x_position = 0.0;
double y_position = 0.0;
double wheel_base = 0.15; // Distance between wheels in meters
double wheel_diameter = 0.045; // Wheel diameter in meters
double wheel_circumference = M_PI * wheel_diameter;
long previous_left_pulse = 0;
long previous_right_pulse = 0;
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 500;

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void update_position() {
  long current_left_pulse = Encoder_1.getPulsePos();
  long current_right_pulse = Encoder_2.getPulsePos();
  double left_distance = getDistance(current_left_pulse, previous_left_pulse);
  double right_distance = getDistance(current_right_pulse, previous_right_pulse);

  double delta_distance = (left_distance + right_distance) / 2.0;
  double heading = position_gyro.getAngleZ(); // Get the heading from the gyro
  double delta_heading = heading * (M_PI / 180.0); // Convert the heading from degrees to radians

  x_position += delta_distance * cos(delta_heading) * 10;
  y_position += delta_distance * sin(delta_heading) * 10;

  previous_left_pulse = current_left_pulse;
  previous_right_pulse = current_right_pulse;
}

double getDistance(long current_pulse, long previous_pulse) {
  long delta_pulse = current_pulse - previous_pulse;
  double distance = (delta_pulse / 360.0) * wheel_circumference;
  return distance;
}

void print_position() {
  //Serial.print("X: ");
  Serial.print(x_position, 4);
  //Serial.print("m, Y: ");
  Serial.print(",");
  Serial.print(y_position, 4);
  //Serial.print("m, Heading: ");
  Serial.print(",");
  double heading_degrees = position_gyro.getAngleZ();
  Serial.println(heading_degrees, 2);
  //Serial.println(" degrees");
}


void SetSpeed(int newSpeed){
  if(newSpeed > 255)
    newSpeed = 255;
  else if(newSpeed < -255)
    newSpeed = -255;
      
  moveSpeed = newSpeed;
  Encoder_1.setMotorPwm(-moveSpeed); 
  Encoder_2.setMotorPwm(moveSpeed);

 // Serial.print("\tNEW SPEED: ");
 // Serial.println(moveSpeed);
}

void StopBot(void) {
  SetSpeed(0);
}


int rot_dir = 1;

void TurnLeft1(void) {
  Encoder_1.setMotorPwm(-SPEED_ROTATION);
  Encoder_2.setMotorPwm(-SPEED_ROTATION);
}
void TurnRight1(void) {
  Encoder_1.setMotorPwm(SPEED_ROTATION);
  Encoder_2.setMotorPwm(SPEED_ROTATION);
}

void handleSerialInput(char* inputStr) {
  const int len = strlen(inputStr);

  if (inputStr[0] == 'L' && inputStr[1] == ':' && MODE == Auto) {  //Messages from LIDAR
    char lidar_msg[32];
    int substr_depth = 3;
    substring(lidar_msg, inputStr, 2, substr_depth);
    state_colission = c_idle;

    if (!strcmp(lidar_msg, "DET")) {  //Colission detected by lidar
      state_colission = c_stop;

      ChangeMowerState(Colission);

    } else if (!strcmp(lidar_msg, MSG_FROM_LIDAR_RIGHT) || !strcmp(lidar_msg, MSG_FROM_LIDAR_LEFT)) {  //Lidar requests rotation by some degrees
      state_colission = c_rotate;

      char ch_angle[32];
      substring(ch_angle, inputStr, 6, substr_depth);

      sscanf(ch_angle, "%d", &colission_rotate_deg);
      colission_rotate_dir = lidar_msg[0];

      if(colission_rotate_dir == 'L')
        colission_rotate_deg = -colission_rotate_deg;


    } else if (!strcmp(lidar_msg, "DON")) {  //Lidar is done taking photo
      state_colission = c_resume;
    }
  }

  else if (inputStr[0] == 'M' && inputStr[1] == ':') {
    switch (inputStr[2]) {
      case 'M':
        {
          if (MODE == Auto) {
            SetSpeed(SPEED_MANUAL);
            MODE = Manual;
          }
          break;
        }
      case 'A':
        {
          if (MODE == Manual) {
            deg = -200;
            start_deg = -200;
            to_deg = -200;
            ChangeMowerState(Locate_Path);
            MODE = Auto;
          }
          break;
        }
    }

  }

  else if (inputStr[0] == 'D' && inputStr[1] == ':' && MODE == Manual) {

    switch (inputStr[2]) {
      case 'W':
        {
          state_manual = Forwards;
          SetSpeed(SPEED_HIGH);
          break;
        }
      case 'A':
        {
          state_manual = Left;
          StopBot();
          TurnLeft1();
          break;
        }
      case 'S':
        {
          state_manual = Backwards;
          SetSpeed(-SPEED_HIGH);
          break;
        }
      case 'D':
        {
          state_manual = Right;
          StopBot();
          TurnRight1();
          break;
        }
      case 'Q':
        {
          state_manual = Stopped;
          StopBot();
          break;
        }
    }
  }
}

void ChangeMowerState(int new_state) {

  switch (new_state) {
    default:
      {
        ChangeMowerState(Idle);
        break;
      }
    case Idle:
      {
        state_auto = new_state;
        //SetLedRing(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS);
        break;
      }
    case Locate_Path:
      {
        if(WALL_DETECTION_ACTIVE){
          StopBot();
          state_auto = new_state;
          start_deg = deg;
          //SetLedRing(LED_BRIGHTNESS, 0, 0);

        }
        else{
          ChangeMowerState(Forward_Fast);
        }
        break;
      }
    case Away_From_Line:
    {
      if(LINE_DETECTION_ACTIVE){
        StopBot();
        state_auto = new_state;
        gyro.begin();
      }
        else{
          ChangeMowerState(Locate_Path);
        }
      break;
    }
    case Forward_Fast:
      {
        state_auto = new_state;
        SetSpeed(SPEED_HIGH);
        break;
      }
    case Forward_Approach:
      {
        state_auto = new_state;
        SetSpeed(SPEED_MEDIUM);
        //SetLedRing(LED_BRIGHTNESS, LED_BRIGHTNESS, 0);
        break;
      }
    case Colission:
      {
        state_auto = new_state;
        StopBot();
        backing_stop_time = 0;
        //SetLedRing(LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
        break;
      }
    case Back:
      {
        state_auto = new_state;
        SetSpeed(SPEED_MEDIUM);
        //SetLedRing(LED_BRIGHTNESS, 0, LED_BRIGHTNESS);
        break;
      }
  }
}

void setup() {
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  gyro.begin();
  position_gyro.begin();
  Serial.begin(9600);
  delay(2000);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  update_position();
  print_position();

#ifdef MeAuriga_H
  // 12 LED Ring controller is on Auriga D44/PWM
  led_ring.setpin(44);
#endif
}

int prev_state = 0;

void loop() {
  
  position_gyro.update();
  update_position();
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdateTime >= updateInterval) {

    //print_position();
    lastUpdateTime = currentTime;
  }
  
//  Serial.print("Angle:");
//  Serial.print(position_angle);
//  Serial.print(",");
//  Serial.print("X_POS:");
//  Serial.print(position_x * 100);
//  Serial.print(",");
//  Serial.print("Y_POS:");
//  Serial.println(position_y * 100);

  int i = 0;
  char str_serial[32];

  if(Serial.available()){
    delay(SERIAL_DELAY_MS);
    while (Serial.available()) {
      char c = Serial.read();
      if (c != '\n') {
        str_serial[i] = c;
        i++;
      }
    }
    Serial.flush();
  }
  

  if(DEBUG){
    if(prev_state != state_auto)
    {
      Serial.print("State: ");
      Serial.println(state_auto);
      if(state_auto == Away_From_Line){
        Serial.print("L: ");
        Serial.print(line_sensor_l);
        Serial.print("\tR: ");
        Serial.println(line_sensor_r);
      }

    }
  }
  prev_state = state_auto; 

  if (i > 0) {
    str_serial[i] = '\0';
    handleSerialInput(str_serial);
  }

  switch (MODE) {
    case Manual:
      {
        SetLedRing(0, 0, LED_BRIGHTNESS);
        switch (state_manual) {
          case Stopped:
            {
              break;
            }
          case Forwards:
            {
              break;
            }
          case Forwards_Left:
            {
              break;
            }
          case Forwards_Right:
            {
              break;
            }
          case Backwards:
            {
              break;
            }
          case Backwards_Left:
            {
              break;
            }
          case Backwards_Right:
            {
              break;
            }
          case Right:
            {
              break;
            }
          case Left:
            {
              break;
            }
        }
        break;
      }
    case Auto:
      {

        SetLedRing(0, LED_BRIGHTNESS, 0);
        if(WALL_DETECTION_ACTIVE){
          distance_cm = ultraSensor.distanceCm();
        }

        if(LINE_IS_BLACK){
          line_sensor_r = !lineFinder.readSensor2();
          line_sensor_l = !lineFinder.readSensor1();
        } 
        else{
          line_sensor_r = lineFinder.readSensor2();
          line_sensor_l = lineFinder.readSensor1();
        }

        int line_sensor_detect = line_sensor_r + line_sensor_l;
        deg = GetZAngle();



        switch (state_auto) {
          default:
            {
            }
          case Idle:
            {
              if(WALL_DETECTION_ACTIVE){
                ChangeMowerState(Locate_Path);
              } else{
                ChangeMowerState(Forward_Fast);
              }
              break;
            }

          case Locate_Path:
            {
              if ((to_deg != -200) && (deg < to_deg)) {
                TurnRight1();

              } else if ((distance_cm <= DISTANCE_MEDIUM) || (distance_cm >= 400.0)) {

                StopBot();
                gyro.begin();
                start_deg = GetZAngle();
                to_deg = (start_deg + ROTATE_AVOIDANCE_DEG) % 360;

              } else {
                StopBot();
                start_deg = -200;
                to_deg = -200;
                ChangeMowerState(Forward_Fast);
              }

              break;
            }
          case Away_From_Line:
          {
            if(line_sensor_detect >= 1){
              
              if(backing_stop_time == 0){
                timer.start();
                backing_stop_time = timer.read() + BACK_WHEN_LINE_DETECED_MS;
                SetSpeed(-SPEED_HIGH);
                print_position();
              }
              
              if(line_sensor_l && !line_sensor_r){
                line_direction = 'R';
              }
              else if(!line_sensor_l && line_sensor_r){
                line_direction = 'L';
              }
              else if(line_sensor_l && line_sensor_r){
                line_direction = '?';
              }
           
            } 
            
            else if((timer.read() >= backing_stop_time) && line_sensor_detect == 0){
                backing_stop_time = 0;
                if(timer.state() == 1){
                  timer.stop();
                  StopBot();
                }
                if(start_deg == -200){
                  
                  start_deg = GetZAngle();
                  to_deg = start_deg + ROTATE_LINE_DEG;

                  if(line_direction == 'L'){
                    to_deg = abs(start_deg - ROTATE_LINE_DEG);
                  }
                  
                }
                else if (line_direction == 'R' || line_direction == '?'){
                  if(abs(deg) < to_deg){
                    TurnRight1();
                  }
                  else{
                    ChangeMowerState(Locate_Path);
                  }
                }
                else if (line_direction == 'L'){
                  if(abs(deg) < to_deg){
                    TurnLeft1();
                  }
                  else{
                    ChangeMowerState(Locate_Path);
                  }
                }
            }

            break;
          }
          case Forward_Fast:
            {
              if(line_sensor_detect){
                ChangeMowerState(Away_From_Line);
              }
              else if (distance_cm < DISTANCE_LONG) {
                ChangeMowerState(Forward_Approach);
              } 
              break;
            }

          case Forward_Approach:
            {
              if(line_sensor_detect){
                ChangeMowerState(Away_From_Line);
                //print_position();
              }
              else if (distance_cm >= DISTANCE_LONG) {

                ChangeMowerState(Forward_Fast);

              } else if (distance_cm > DISTANCE_MEDIUM) {
                SetSpeed(SPEED_MEDIUM);
              } else if (distance_cm > DISTANCE_SHORT) {
                SetSpeed(SPEED_SLOW);
              } else if (distance_cm <= DISTANCE_COLISSION || (distance_cm >= 400.0)) {
                ChangeMowerState(Back);
              }
              break;
            }

          case Colission:
            {
              switch (state_colission)
              {
                case c_idle:
                {
                  StopBot();
                  break;
                }
                case c_stop:
                {
                  StopBot();
                  Serial.print(MSG_TO_LIDAR_STOP);
                  state_colission = c_idle;
                  break;
                }
                case c_rotate:
                {
                  if (start_deg == -200) { 
                      gyro.begin();
                      start_deg = GetZAngle();
                      deg = start_deg;
                      to_deg = start_deg + colission_rotate_deg;

                  } else if (colission_rotate_dir == 'R' && deg < to_deg){
                    
                    TurnRight1();
                  } else if (colission_rotate_dir == 'L' && deg > to_deg){
                    TurnLeft1();
                  } else{
                    StopBot();
                    char msg_to_lidar_pos[32];
                    lidar_pos_msg(msg_to_lidar_pos, position_x, position_y);
                    Serial.print(msg_to_lidar_pos);
                    start_deg = -200;
                    to_deg = -200;
                    state_colission = c_idle;
                  }
                  break;
                }
                case c_resume:
                {
                  ChangeMowerState(Locate_Path);
                  state_colission = c_idle;
                  Serial.print(MSG_TO_LIDAR_OK);
                  break;
                }
                default:
                {
                  break;
                }
              }
              
              break;
            }
          case Back:
            {
              if (distance_cm >= DISTANCE_SHORT) {
                ChangeMowerState(Locate_Path);
              }
              SetSpeed(-SPEED_HIGH);
              break;
            }
        }
        break;
      }
  }
}