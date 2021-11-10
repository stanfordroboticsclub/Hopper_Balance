#include <HopperRobot.h>
#include <Adafruit_LSM9DS1.h> //NOTE: Without these inclusions, platformio throws an error
#include <Adafruit_Sensor.h> 

HopperRobot robot;
void setup(){
  Serial.begin(115200);
}

void loop(){
  robot.control_step();

  Serial.print("est_pitch: ");
  Serial.println(robot.get_pitch(true));
}