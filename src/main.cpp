#include <HopperRobot.h>
// #include <Adafruit_LSM9DS1.h> //NOTE: Without these inclusions, platformio throws an error
// #include <Adafruit_Sensor.h> 

HopperRobot robot;
uint32_t last_control_step;
void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);
  while (!Serial) {}
  robot.homing_sequence();
  last_control_step = micros();
}

void loop(){
  // Serial.println("LOOP");
  // delay(100);
  robot.PollCAN();
  if(micros() - last_control_step > 2000) {
    robot.control_step();
    // Serial.print("Timestamp (ms): ");
    // Serial.println(millis());
    // Serial.println(robot.get_pitch(false));
    last_control_step = micros();
  }
}