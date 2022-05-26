#include <HopperRobot.h>
#include <Filter.h>

#define XBEE
#ifdef XBEE
#define Serial Serial3
#endif

HopperRobot robot_;

const float kControlFrequency = 500;
uint32_t last_control_step_;

float des_state_[8] = {0, 0, 0, 0, 0, 0, 0.6, 0.6};
float des_vel_ = 0;
float des_yaw_rate_ = 0;
float des_leg_height_ = 0.6;
float des_leg_tilt_ = 0;

const float kMinLegExtension = 0.25;
const float kMaxLegExtension = 0.75;

Filter velFilter(5, kControlFrequency);
Filter yawFilter(5, kControlFrequency);
Filter leftLegFilter(5, kControlFrequency);
Filter rightLegFilter(5, kControlFrequency);

#define MESSAGE_LENGTH 4 * sizeof(float)
#define SENTINAL 'H'

void readCommands() {
  static char buffer[MESSAGE_LENGTH];
  if (Serial.available() > MESSAGE_LENGTH) {
    if (Serial.read() == SENTINAL) {
      for (size_t i = 0; i < MESSAGE_LENGTH; i++) {
        buffer[i] = Serial.read();
      }
      des_vel_ = ((float *) buffer)[0];
      des_yaw_rate_ = ((float *) buffer)[1];
      des_leg_height_ = ((float *) buffer)[2];
      des_leg_tilt_ = ((float *) buffer)[3];
    } else {
      while (Serial.available() > 0) {
        Serial.read();
      }
    }
  }
}

void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  while (!Serial) {}
  delay(5000);

  robot_.homing_sequence();

  last_control_step_ = micros();
}

void loop(){
  readCommands();

  robot_.PollCAN();
  if(micros() - last_control_step_ > (uint32_t) (1000000 / kControlFrequency)) {
    last_control_step_ = micros();

    float left_leg_height = constrain(des_leg_height_ + des_leg_tilt_, kMinLegExtension, kMaxLegExtension);
    float right_leg_height = constrain(des_leg_height_ - des_leg_tilt_, kMinLegExtension, kMaxLegExtension);

    des_state_[6] = leftLegFilter.filter(left_leg_height);
    des_state_[7] = rightLegFilter.filter(right_leg_height);

    if (robot_.get_state() == IDLE) {
      des_state_[1] = 0.0;
      des_state_[4] = 0.0;
      velFilter.reset();
      yawFilter.reset();
    } else {
      float vel = velFilter.filter(des_vel_);
      float yaw_rate = yawFilter.filter(des_yaw_rate_);

      des_state_[3] = vel;
      des_state_[5] = yaw_rate;

      des_state_[1] += vel / kControlFrequency;
      des_state_[4] += yaw_rate / kControlFrequency;
    }
    
    robot_.control_step(des_state_);
  }
}


