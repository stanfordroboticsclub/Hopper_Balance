#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <MahonyAHRS.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Mahony filter;
void setupSensor()
{
  // 1.) Set the accelerometer range
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  // 3.) Setup the gyroscope
  // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}
void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  Serial.println("LSM9DS1 data read demo");
  // Try to initialise and warn if we couldn’t detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  // helper to just set the default scaling we want, see above!
  setupSensor();
  filter.begin(500.0);
}
void loop()
{
  lsm.read();  /* ask it to read in the data */
  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  // Serial.print(“Accel X: “); Serial.print(a.acceleration.x); Serial.print(” m/s^2");
  // Serial.print(“\tY: “); Serial.print(a.acceleration.y);     Serial.print(” m/s^2 “);
  // Serial.print(“\tZ: “); Serial.print(a.acceleration.z);     Serial.println(” m/s^2 “);
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  // Serial.print(“Mag X: “); Serial.print(m.magnetic.x);   Serial.print(” uT”);
  // Serial.print(“\tY: “); Serial.print(m.magnetic.y);     Serial.print(” uT”);
  // Serial.print(“\tZ: “); Serial.print(m.magnetic.z);     Serial.println(” uT”);
  // Serial.print(“Gyro X: “); Serial.print(g.gyro.x);   Serial.print(” rad/s”);
  // Serial.print(“\tY: “); Serial.print(g.gyro.y);      Serial.print(” rad/s”);
  // Serial.print(“\tZ: “); Serial.print(g.gyro.z);      Serial.println(” rad/s”);
  float gyroScale = 180.0 / 3.14159;
  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;
  filter.updateIMU(gx * gyroScale, gy * gyroScale, gz * gyroScale, ax, ay, az);
  // Serial.println();
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(roll);
  delay(2);
}