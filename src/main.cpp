#include <HopperRobot.h>
// #include <Adafruit_LSM9DS1.h> //NOTE: Without these inclusions, platformio throws an error
// #include <Adafruit_Sensor.h> 

HopperRobot robot;
long last_control_step;
void setup(){
  Serial.begin(115200);
  delay(500);
  robot.homing_sequence();
  last_control_step = micros();
}

void loop(){
  // Serial.println("LOOP");
  // delay(100);
  robot.PollCAN();
  if(micros() - last_control_step > 2000) {
    robot.control_step();
    Serial.print("Timestamp (ms): ");
    Serial.println(millis());
    last_control_step = micros();
  }
}