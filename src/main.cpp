#include <HopperRobot.h>
#include <Adafruit_LSM9DS1.h> //NOTE: Without these inclusions, platformio throws an error
#include <Adafruit_Sensor.h> 

HopperRobot robot;
void setup(){
  Serial.begin(115200);
  robot.homing_sequence();
}

void loop(){
  //robot.control_step();
  //robot.test_impedence_hold(0.0);

  Serial.println("------------");
}