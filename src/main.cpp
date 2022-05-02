#include <HopperRobot.h>
#include <Filter.h>

#define XBEE
#ifdef XBEE
#define Serial Serial3
#endif

HopperRobot robot_;

const float kControlFrequency = 500;
uint32_t last_control_step_;

float des_state_[6] = {0, 0, 0, 0, 0, 0};
float des_vel_ = 0;
float des_yaw_rate_ = 0;
const float kMaxYawRate = 1000;

Filter velFilter(2, kControlFrequency);
Filter yawFilter(2, kControlFrequency);

void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  Serial.setTimeout(1);
  while (!Serial) {}
  delay(5000);

  robot_.homing_sequence();

  last_control_step_ = micros();
}

void loop(){
  if (Serial.available() > 0) {
    des_yaw_rate_ = constrain(Serial.parseFloat(), -kMaxYawRate, kMaxYawRate);
    Serial.read();
    Serial.println(des_yaw_rate_);
  }

  robot_.PollCAN();
  if(micros() - last_control_step_ > (uint32_t) (1000000 / kControlFrequency)) {
    if (robot_.get_state() == IDLE) {
      des_state_[1] = 0.0;
      des_state_[4] = 0.0;
      velFilter.reset();
      yawFilter.reset();
    } else {
      float yaw_rate = yawFilter.filter(des_yaw_rate_);
      des_state_[5] = yaw_rate;
      des_state_[4] += yaw_rate / kControlFrequency;
    }
    last_control_step_ = micros();
    robot_.control_step(des_state_);
  }
}