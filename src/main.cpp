#include <HopperRobot.h>
// #include <Adafruit_LSM9DS1.h> //NOTE: Without these inclusions, platformio throws an error
// #include <Adafruit_Sensor.h> 

HopperRobot robot;
uint32_t last_control_step;
float des_state[4] = {0, 0, 0, 0};
float des_velocity = -5;
void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  // while (!Serial) {}
  delay(5000);
  robot.homing_sequence();
  des_state[3] = des_velocity;
  last_control_step = micros();
}

void loop(){
  // Serial.println("LOOP");
  // delay(100);
  robot.PollCAN();
  if(micros() - last_control_step > 2000) {
    if (robot.get_state() == IDLE) {
      des_state[1] = 0.0;
    } else {
      des_state[1] += des_velocity / 500;
    }
    last_control_step = micros();
    robot.control_step(des_state);
    // Serial.print("Timestamp (ms): ");
    // Serial.println(micros() - last_control_step);
    // Serial.println(robot.get_pitch(false));
  }
}