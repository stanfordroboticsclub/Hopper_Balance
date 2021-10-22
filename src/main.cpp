#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <MahonyAHRS.h>
#include <C610Bus.h>

/*
  NOTES: 
  LQR Gain Matrix: [-1.6864008e+00 -2.0663252e-16 -3.0060002e-01 -1.1180325e-02]
- LQR code outputs desired torque for both motors.
- Be sure to reverse the output on one motor
- DJI Command takes in current in milliamps
- Current -> Torque conversion: 0.25nM / Amp
*/

C610Bus<CAN1> bus;

//Motor Information (TODO: Update this to the actual motor indexes)
char left_wheel_idx = 0;
char right_wheel_idx = 1;
char left_extend_idx = 2;
char right_extend_idx = 3;

float HEIGHT_POS = 0; //TODO: Set Height Position for leg extensions
float lqr_gains[] = {-1.6864008e+00 -2.0663252e-16 -3.0060002e-01 -1.1180325e-02};


//Impedence Control
float impedence_alpha = 2000;
float impedence_beta = 2000;

float get_wheel_torque (float* imu_array){
  float op_sum = 0;
  int lqr_len = sizeof(lqr_gains) / sizeof(float);

  for (int i = 0; i < lqr_len; i++)
    op_sum += lqr_gains[i] * imu_array[i];
  
  return op_sum;
}

float get_impedence_command(int motor_idx, float desired_pos){
  C610 esc = bus.Get(motor_idx);
  float motor_pos = esc.Position();
  float motor_vec = esc.Velocity();

  return impedence_alpha * (desired_pos - motor_pos) - impedence_beta * motor_vec;
}

void set_motor_comms(float wheel_torque){
  //Calculate and send motor commands based on a specified wheel torque
  float torque_to_amps = 4.0;
  float amps_to_millis = 1000;

  float wheel_amps = torque_to_amps * amps_to_millis;

  float motor_comm_arr[4];

  //TODO: Calculate + Send commands 
  motor_comm_arr[right_extend_idx] = get_impedence_command(right_extend_idx, HEIGHT_POS);
  motor_comm_arr[left_extend_idx] = get_impedence_command(left_extend_idx, HEIGHT_POS);
  motor_comm_arr[left_wheel_idx] = wheel_amps;
  motor_comm_arr[right_wheel_idx] = -1.0 * wheel_amps;

  bus.CommandTorques(motor_comm_arr[0], motor_comm_arr[1], motor_comm_arr[2], motor_comm_arr[3]);
}

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


  float imu_array[] = {roll, pitch, heading, 0.0};
  float wheel_torque = get_wheel_torque(imu_array);
  set_motor_comms(wheel_torque);
  
  delay(2);
}