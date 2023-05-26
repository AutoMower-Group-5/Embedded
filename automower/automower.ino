#include <MeAuriga.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include "Timer.h"

Timer timer;
Timer rotation_timer;

//MeGyro gyro(0, 0x69);
MeGyro position_gyro(0, 0x69);

unsigned char table[128] = { 0 };
SoftwareSerial softuart(13, 12);
MeSerial mySerial(PORT_8);

//MeUltrasonicSensor ultraSensor(PORT_7); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
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
enum Mode { Boot,
            Manual,
            Auto };
enum Mode MODE = Boot;

enum state_BootMower{
  Send_Notification,
  Wait_For_Ack
};

enum state_AutoMower {
  Idle,
  Locate_Path,
  Away_From_Line,
  Reset_Pos,
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

enum state_BootMower state_boot = Send_Notification;
enum state_AutoMower state_auto = Idle;
enum state_ManualMower state_manual = Stopped;
enum state_Colission state_colission = c_idle;

//Communication

const char* MSG_TO_LIDAR_STOP = "A:STOP\n";
const char* MSG_TO_LIDAR_OK = "A:OK\n";
const int MSG_TO_LIDAR_N_INT = 3;

const char* MSG_FROM_LIDAR_DETECT = "DET";
const char* MSG_FROM_LIDAR_DONE = "DON";
const char* MSG_FROM_LIDAR_RIGHT = "RHT";
const char* MSG_FROM_LIDAR_LEFT = "LFT";

const char* BOOT_MSG = "BOOTED\n";

//Configuration

const int DEFAULT_MODE = Auto;

const int DEBUG = 0;

unsigned long INTERVAL_POS_PRINT_MS = 500;

const float LIDAR_DIST_COLISSION = 25.0;
const float LIDAR_MOUNT_DIST_TO_MIDDLE = 8.0;
const float LIDAR_MOUNT_ANGLE = 22.0*((float)M_PI/180.0);

const float LIDAR_MOUNT_OFFSET_TO_OBJ = LIDAR_MOUNT_DIST_TO_MIDDLE + LIDAR_DIST_COLISSION*cos(LIDAR_MOUNT_ANGLE);


const int SPEED_FACTOR = 1 + DEBUG;
const int16_t SPEED_HIGH = 100 / SPEED_FACTOR;
const int16_t SPEED_MEDIUM = 85 / SPEED_FACTOR;
const int16_t SPEED_SLOW = 50 / SPEED_FACTOR;

const int16_t SPEED_MANUAL = SPEED_MEDIUM;
const int16_t SPEED_ROTATION = SPEED_HIGH;

const float LENGTH_TO_ROTATION_AXIS = 11.0;

const int16_t LED_BRIGHTNESS = 10;

const int SERIAL_DELAY_MS = 20;

const int WALL_DETECTION_ACTIVE = 0;
const int16_t ROTATE_AVOIDANCE_DEG = 37;
const int16_t DISTANCE_LONG = 50;
const int16_t DISTANCE_MEDIUM = 30;
const int16_t DISTANCE_SHORT = 15;
const int16_t DISTANCE_WALL_COLISSION = 5;

const int LINE_DETECTION_ACTIVE = 1;
const int BACK_WHEN_LINE_DETECED_MS = 500;
const int16_t ROTATE_LINE_TIM_MIN = 1*1000;
const int16_t ROTATE_LINE_TIM_MAX = 2*1000;
const int16_t ROTATE_OBJECT_TIME = 200;

const int WAIT_FOR_BOOT = 1;

const int LINE_IS_BLACK = 1;



void set_motors(float motor1Percent, float motor2Percent) {
  Encoder_1.setMotorPwm((motor1Percent / 100.0) * moveSpeed);
  Encoder_2.setMotorPwm((motor2Percent / 100.0) * moveSpeed);
}


void set_led_ring(int r, int g, int b) {
  led_ring.setColor(RINGALLLEDS, r, g, b);
  led_ring.show();
}


int rotate_line_time(){
  int rot_tim = random(ROTATE_LINE_TIM_MIN, ROTATE_LINE_TIM_MAX);
  return rot_tim;
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


void int_to_arr(char* res, int input, const int depth) {
  int i = 0;
  for (; i < depth; i++) {
    res[i] = '0' + (input / (int)pow(10, depth - 1 - i)) % 10;
  }
  res[i] = '\0';
}

void lidar_pos_msg(char* res, int x, int y) {
  char* start = "A:POS:";
  char x_str[5];
  char y_str[5];

  int_to_arr(x_str, x, MSG_TO_LIDAR_N_INT);
  int_to_arr(y_str, y, MSG_TO_LIDAR_N_INT);

  int res_ix = 0;
  for (; start[res_ix] != '\0'; res_ix++) {
    res[res_ix] = start[res_ix];
  }
  for (int ix = 0; x_str[ix] != '\0'; ix++, res_ix++) {
    res[res_ix] = x_str[ix];
  }
  res[res_ix++] = ':';
  for (int ix = 0; y_str[ix] != '\0'; ix++, res_ix++) {
    res[res_ix] = y_str[ix];
  }
  res[res_ix++] = '\n';
  res[res_ix] = '\0';
}



char next_rotation = 'R';
char rotate_dir = next_rotation;


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
int rotate_stop_time = 0;

float position_angle = 0;

//Dead reckoning
double x_position = 0.0;
double y_position = 0.0;
double wheel_base = 0.15;       // Distance between wheels in meters
double wheel_diameter = 0.045;  // Wheel diameter in meters
double wheel_circumference = M_PI * wheel_diameter;
long previous_left_pulse = 0;
long previous_right_pulse = 0;
unsigned long lastUpdateTime = 0;

int r_pi_booted = 0; 

void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void) {
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosPlus();
  } else {
    Encoder_2.pulsePosMinus();
  }
}

void update_position() {
  long current_left_pulse = Encoder_1.getPulsePos();
  long current_right_pulse = Encoder_2.getPulsePos();
  double left_distance = get_distance(current_left_pulse, previous_left_pulse);
  double right_distance = get_distance(current_right_pulse, previous_right_pulse);

  position_gyro.fast_update();

  double delta_distance = (left_distance + right_distance) / 2.0;
  double heading = position_gyro.getAngleZ();       // Get the heading from the gyro
  
  double delta_heading = heading * (M_PI / 180.0);  // Convert the heading from degrees to radians

  x_position += delta_distance * cos(delta_heading) * 10;
  y_position += delta_distance * sin(delta_heading) * 10;

  previous_left_pulse = current_left_pulse;
  previous_right_pulse = current_right_pulse;
}

double get_distance(long current_pulse, long previous_pulse) {
  long delta_pulse = current_pulse - previous_pulse;
  double distance = (delta_pulse / 360.0) * wheel_circumference;
  return distance;
}


void print_position(char type = 'P') {

  //Types:  A <=> Lidar position
  //        P <=> Normal position
  //Message structure: P:POS:XX...X.XX:YY...Y.YY:A..A.AA\n

  char* header = ":POS";
  char separator = ':';
  int decimals = 4;

  Serial.print(type);

  for (int header_ix = 0; header[header_ix] != '\0'; header_ix++) {
    Serial.print(header[header_ix]);
  }

  //position_gyro.fast_update();
  float heading_degrees = position_gyro.getAngleZ();

  if (type == 'P') {
    Serial.print(separator);
    Serial.print(x_position, decimals);
    Serial.print(separator);
    Serial.print(y_position, decimals);
    Serial.print(separator);
    Serial.print(heading_degrees, decimals);
  } else if (type == 'A') {

    float obj_pos_x = x_position + LIDAR_MOUNT_OFFSET_TO_OBJ*cos(heading_degrees* (M_PI/180.0));
    float obj_pos_y = y_position + LIDAR_MOUNT_OFFSET_TO_OBJ*sin(heading_degrees* (M_PI/180.0));
    
    Serial.print(separator);
    Serial.print(x_position, decimals);
    Serial.print(separator);
    Serial.print(y_position, decimals);
  }
  Serial.print('\n');
}


void set_speed(int newSpeed) {
  if (newSpeed > 255)
    newSpeed = 255;
  else if (newSpeed < -255)
    newSpeed = -255;

  moveSpeed = newSpeed;
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);

}

void stop_bot(void) {
  set_speed(0);
}


void rotate_left(void) {
  Encoder_1.setMotorPwm(-SPEED_ROTATION);
  Encoder_2.setMotorPwm(-SPEED_ROTATION);
}
void rotate_right(void) {
  Encoder_1.setMotorPwm(SPEED_ROTATION);
  Encoder_2.setMotorPwm(SPEED_ROTATION);
}

void handleSerialInput(char* inputStr) {
  const int len = strlen(inputStr);

  if(inputStr[0] == 'B' && !r_pi_booted){
    r_pi_booted = 1;
  }

  if (inputStr[0] == 'P' && inputStr[1] == 'O' && inputStr[2] == 'S') {
    print_position();
  }

  if (inputStr[0] == 'L' && inputStr[1] == ':' && MODE == Auto) {  //Messages from LIDAR
    char lidar_msg[32];
    int substr_depth = 3;
    substring(lidar_msg, inputStr, 2, substr_depth);
    state_colission = c_idle;

    if (!strcmp(lidar_msg, "DET")) {  //Colission detected by lidar
      start_deg = -200.0;
      state_colission = c_stop;

      change_mower_state(Colission);

    } else if (!strcmp(lidar_msg, MSG_FROM_LIDAR_RIGHT) || !strcmp(lidar_msg, MSG_FROM_LIDAR_LEFT)) {  //Lidar requests rotation by some degrees
      state_colission = c_rotate;

      char ch_angle[32];
      substring(ch_angle, inputStr, 6, substr_depth);

      sscanf(ch_angle, "%d", &colission_rotate_deg);
      colission_rotate_dir = lidar_msg[0];

      if (colission_rotate_dir == 'L')
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
            set_speed(0);
            state_manual = Stopped;
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
            change_mower_state(Locate_Path);
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
          set_speed(SPEED_HIGH);
          break;
        }
      case 'A':
        {
          state_manual = Left;
          stop_bot();
          rotate_left();
          break;
        }
      case 'S':
        {
          state_manual = Backwards;
          set_speed(-SPEED_HIGH);
          break;
        }
      case 'D':
        {
          state_manual = Right;
          stop_bot();
          rotate_right();
          break;
        }
      case 'Q':
        {
          state_manual = Stopped;
          stop_bot();
          break;
        }
    }
  }
}

void change_mower_state(int new_state) {

  start_deg = -200;

  switch (new_state) {
    default:
      {
        change_mower_state(Idle);
        break;
      }
    case Idle:
      {
        state_auto = new_state;
        break;
      }
    case Locate_Path:
      {
        if (WALL_DETECTION_ACTIVE) {
          stop_bot();
          state_auto = new_state;
          start_deg = deg;

        } else {
          change_mower_state(Forward_Fast);
        }
        break;
      }
    case Away_From_Line:
      {
        if (LINE_DETECTION_ACTIVE) {
          stop_bot();
          state_auto = new_state;
        } else {
          change_mower_state(Locate_Path);
        }
        break;
      }
    case Forward_Fast:
      {
        state_auto = new_state;
        set_speed(SPEED_HIGH);
        break;
      }
    case Forward_Approach:
      {
        state_auto = new_state;
        set_speed(SPEED_MEDIUM);
        //SetLedRing(LED_BRIGHTNESS, LED_BRIGHTNESS, 0);
        break;
      }
    case Colission:
      {
        state_auto = new_state;
        stop_bot();
        backing_stop_time = 0;
        //SetLedRing(LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
        break;
      }
    case Back:
      {
        state_auto = new_state;
        set_speed(SPEED_MEDIUM);
        //SetLedRing(LED_BRIGHTNESS, 0, LED_BRIGHTNESS);
        break;
      }
  }
}

void setup() {
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
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
  position_gyro.begin();
  set_led_ring(LED_BRIGHTNESS, 0, 0);
}

void loop() {

  update_position();
  
  
  int8_t start_area_detected = 0;

  if (!WALL_DETECTION_ACTIVE) {
    start_area_detected = (distance_cm <= DISTANCE_MEDIUM);
  }

  unsigned long currentTime = millis();

  int i = 0;
  char str_serial[32];

  if (Serial.available()) {
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

  if (i > 0) {
    str_serial[i] = '\0';
    handleSerialInput(str_serial);
  }

  switch (MODE) {
    case Boot:
    {
      if(WAIT_FOR_BOOT)
      {
        set_led_ring(LED_BRIGHTNESS, 0, 0);
        switch (state_boot) {
          case Send_Notification:
          {
            Serial.print(BOOT_MSG);
            state_boot = Wait_For_Ack;
            break;
          }
          case Wait_For_Ack:
          {
            if(r_pi_booted){
              MODE = DEFAULT_MODE;
            }
            break;
          }

          default:
          {
            break;
          }
        }
      }
      else{
        
        MODE = DEFAULT_MODE;
      }

      break;
    }
    case Manual:
      {
        set_led_ring(0, 0, LED_BRIGHTNESS);
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

        set_led_ring(0, LED_BRIGHTNESS, 0);
        if (WALL_DETECTION_ACTIVE) {
          //Qdistance_cm = ultraSensor.distanceCm();
        }

        if (LINE_IS_BLACK) {
          line_sensor_r = !lineFinder.readSensor2();
          line_sensor_l = !lineFinder.readSensor1();
        } else {
          line_sensor_r = lineFinder.readSensor2();
          line_sensor_l = lineFinder.readSensor1();
        }

        int line_sensor_detect = line_sensor_r + line_sensor_l;



        switch (state_auto) {
          default:
            {
            }
          case Idle:
            {
              if (WALL_DETECTION_ACTIVE) {
                change_mower_state(Locate_Path);
              } else {
                change_mower_state(Forward_Fast);
              }
              break;
            }

          case Locate_Path:
            {
              if ((to_deg != -200) && (deg < to_deg)) {
                rotate_right();

              } else if ((distance_cm <= DISTANCE_MEDIUM) || (distance_cm >= 400.0)) {

                stop_bot();
                //gyro.begin();
                //start_deg = get_z_angle();
                to_deg = (start_deg + ROTATE_AVOIDANCE_DEG) % 360;

              } else {
                stop_bot();
                start_deg = -200;
                to_deg = -200;
                change_mower_state(Forward_Fast);
              }

              break;
            }
          case Away_From_Line:
            {
              if (line_sensor_detect >= 1) {

                if (timer.state() == 0) {
                  timer.start();
                  backing_stop_time = timer.read() + BACK_WHEN_LINE_DETECED_MS;
                  set_speed(-SPEED_HIGH);
                }

                if (line_sensor_l && !line_sensor_r) {
                  line_direction = 'R';
                } else if (!line_sensor_l && line_sensor_r) {
                  line_direction = 'L';
                } else if (line_sensor_l && line_sensor_r) {
                  line_direction = '?';
                }
              }

              else if ((timer.read() >= backing_stop_time) && line_sensor_detect == 0) {
                if (timer.state() == 1) {
                  timer.stop();
                  set_speed(0);
                }
                if (rotation_timer.state() == 0) {

                  rotation_timer.start();
                  rotate_stop_time = rotation_timer.read() + rotate_line_time();
                  Serial.print("TIME: ");
                  Serial.println(rotate_stop_time);

                } else if (line_direction == 'R') {
                  if (rotation_timer.read() < rotate_stop_time) {
                    rotate_right();
                  } else {
                    rotation_timer.stop();
                    change_mower_state(Locate_Path);
                  }
                } else if (line_direction == 'L' || line_direction == '?') {
                  if (rotation_timer.read() < rotate_stop_time) {
                    rotate_left();
                  } else {
                    rotation_timer.stop();
                    change_mower_state(Locate_Path);
                  }
                }
              }

              break;
            }
          case Reset_Pos:
            {

            }
          case Forward_Fast:
            {
              if ((line_sensor_detect) && (distance_cm < DISTANCE_MEDIUM) && !WALL_DETECTION_ACTIVE) {
                set_speed(0);
                change_mower_state(Away_From_Line);
              } else if (line_sensor_detect) {
                change_mower_state(Away_From_Line);
              }

              else if (distance_cm < DISTANCE_LONG && WALL_DETECTION_ACTIVE) {
                change_mower_state(Forward_Approach);
              }
              break;
            }

          case Forward_Approach:
            {
              if (line_sensor_detect) {
                change_mower_state(Away_From_Line);
              } else if (distance_cm >= DISTANCE_LONG) {

                change_mower_state(Forward_Fast);

              } else if (distance_cm > DISTANCE_MEDIUM) {
                set_speed(SPEED_MEDIUM);
              } else if (distance_cm > DISTANCE_SHORT) {
                set_speed(SPEED_SLOW);
              } else if (distance_cm <= DISTANCE_WALL_COLISSION || (distance_cm >= 400.0)) {
                change_mower_state(Back);
              }
              break;
            }

          case Colission:
            {
              switch (state_colission) {
                case c_idle:
                  {
                    stop_bot();
                    break;
                  }
                case c_stop:
                  {
                    stop_bot();
                    Serial.print(MSG_TO_LIDAR_STOP);
                    state_colission = c_idle;
                    break;
                  }
                case c_rotate:
                  {
                    if (rotation_timer.state() == 0) {
                      rotation_timer.start();
                      rotate_stop_time = rotation_timer.read() + ROTATE_OBJECT_TIME;
                    } else if (colission_rotate_dir == 'R' && rotation_timer.read() < rotate_stop_time) {

                      rotate_right();
                    } else if (colission_rotate_dir == 'L' && rotation_timer.read() < rotate_stop_time) {
                      rotate_left();
                    } else {
                      stop_bot();
                      char msg_to_lidar_pos[32];
                      print_position('A');
                      rotation_timer.stop();
                      state_colission = c_idle;
                    }
                    break;
                  }
                case c_resume:
                  {
                    if(line_sensor_detect){
                      Serial.print(MSG_TO_LIDAR_OK);
                      change_mower_state(Away_From_Line);
                    }
                    else if (rotation_timer.state() == 0) {
                      rotation_timer.start();
                      rotate_stop_time = rotation_timer.read() + rotate_line_time();
                      rotate_dir = next_rotation;
                    }

                    else if ((rotate_dir == 'R') && (rotation_timer.read() < rotate_stop_time)) {
                      
                      rotate_right();
                      next_rotation = 'L';

                    } else if ((rotate_dir == 'L') && (rotation_timer.read() < rotate_stop_time)){
                      rotate_left();
                      next_rotation = 'R';

                    } else {
                      state_colission = c_idle;
                      Serial.print(MSG_TO_LIDAR_OK);                    
                      rotation_timer.stop();
                      change_mower_state(Locate_Path);
                    }
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
                change_mower_state(Locate_Path);
              }
              set_speed(-SPEED_HIGH);
              break;
            }
        }
        break;
      }
  }
}