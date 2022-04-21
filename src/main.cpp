#include <HopperRobot.h>
// #include <Adafruit_LSM9DS1.h> //NOTE: Without these inclusions, platformio throws an error
// #include <Adafruit_Sensor.h> 

HopperRobot robot;
uint32_t last_control_step;
void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  // while (!Serial) {}
  delay(8000);
  robot.homing_sequence();
  last_control_step = micros();
}

void loop(){
  // Serial.println("LOOP");
  // delay(100);
  robot.PollCAN();
  if(micros() - last_control_step > 2000) {
    last_control_step = micros();
    robot.control_step();
    // Serial.print("Timestamp (ms): ");
    // Serial.println(micros() - last_control_step);
    // Serial.println(robot.get_pitch(false));
  }
}