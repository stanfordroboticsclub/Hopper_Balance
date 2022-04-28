#include <HopperRobot.h>
// #include <Adafruit_LSM9DS1.h> //NOTE: Without these inclusions, platformio throws an error
// #include <Adafruit_Sensor.h>

#define Serial Serial3

HopperRobot robot;
uint32_t last_control_step;
float des_state[6] = {0, 0, 0, 0, 0, 0};
float des_vel = 0;
float des_yaw = 0;
const float max_vel = 10;

void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  while (!Serial) {}
  delay(5000);
  robot.homing_sequence();
  des_state[3] = des_vel;
  last_control_step = micros();
}

void loop(){
  if (Serial.available() > 0) {
    // des_vel = constrain(Serial.parseFloat(), -max_vel, max_vel);
    des_yaw = constrain(Serial.parseFloat(), -max_vel, max_vel);
    Serial.read();
    // des_state[3] = des_vel;
    des_state[4] = des_yaw;
    Serial.println(des_yaw);
  }
  robot.PollCAN();
  if(micros() - last_control_step > 2000) {
    if (robot.get_state() == IDLE) {
      des_state[1] = 0.0;
      des_state[4] = 0.0;
    } else {
      des_state[1] += des_vel / 500;
    }
    last_control_step = micros();
    robot.control_step(des_state);
    // Serial.print("Timestamp (ms): ");
    // Serial.println(micros() - last_control_step);
    // Serial.println(robot.get_pitch(false));
  }
}