#include <Arduino.h>
#include <HopperRobot.h>

HopperRobot::HopperRobot(){
    _prev_sensor_time = micros();

    if (bmi160.softReset() != BMI160_OK){
        //Serial.println("reset false");
        while(1);
    }

    if (bmi160.I2cInit(kI2CAddr) != BMI160_OK){
        //Serial.println("init false");
        while(1);
    }
}

HopperRobot::~HopperRobot(){

}

//Private Methods
float HopperRobot::get_wheel_vel(){
    float left_vel = bus.Get(kLeftWheelIdx).Velocity();
    float right_vel = bus.Get(kRightWheelIdx).Velocity();

    float output = 0.5 * (left_vel - right_vel);
    return output;
}

float HopperRobot::get_extension_position(int legIndex){
    C610 esc = bus.Get(legIndex);
    float motor_pos = esc.Position();
    return motor_pos;
}

float HopperRobot::get_wheel_torque(float* imu_array){
    float op_sum = 0;
    int lqr_len = sizeof(_lqr_gains) / sizeof(float);
    
    for (int i = 0; i < lqr_len; i++)
        op_sum += _lqr_gains[i] * imu_array[i];
    
    return op_sum;
}

float HopperRobot::get_impedence_command(int motor_idx, float desired_pos){
    float motor_pos = bus.Get(motor_idx).Position() - _homed_positions[motor_idx - kLegIdxs[0]];
    float motor_vel = bus.Get(motor_idx).Velocity();

    // Serial.print("Position: ");
    // Serial.println(motor_pos);
    // Serial.print("Velocity: ");
    // Serial.println(motor_vel);

    float command = _impedence_stiffness * (desired_pos - motor_pos) - _impedence_damping * motor_vel;
    command = constrain(command, -_max_current, _max_current);
    return command;
}

void HopperRobot::set_motor_comms(float wheel_torque){
    //Calculate and send motor commands based on a specified wheel torque
    float torque_to_amps = 4.0;
    float amps_to_millis = 1000;

    float wheel_amps = wheel_torque * kTorqueToAmps * kAmpsToMillis;
    wheel_amps = constrain(wheel_amps, -7000, 7000);

    float motor_comm_arr[4];
  
    motor_comm_arr[kRightExtendIdx] = get_impedence_command(kRightExtendIdx, _height_pos);
    motor_comm_arr[kLeftExtendIdx] = get_impedence_command(kLeftExtendIdx, _height_pos);
    motor_comm_arr[kLeftWheelIdx] = 0;//wheel_amps;
    motor_comm_arr[kRightWheelIdx] = 0;//-1.0 * wheel_amps;

    Serial.print(motor_comm_arr[0]);
    Serial.print(" ");
    Serial.print(motor_comm_arr[1]);
    Serial.print(" ");
    Serial.print(motor_comm_arr[2]);
    Serial.print(" ");
    Serial.print(motor_comm_arr[3]);
    Serial.println();

    bus.CommandTorques(motor_comm_arr[0], motor_comm_arr[1], 
                        motor_comm_arr[2], motor_comm_arr[3], C610Subbus::kOneToFourBlinks);
}

float HopperRobot::complimentaryFilter(){
    float y_gyro = accel_gyro_values[1] * _dt;
    float accel_angle = -atan2(accel_gyro_values[5], -accel_gyro_values[3]) * (180 / 3.14);

    float alpha = 0.99;

    float new_angle = alpha * (_pitch_angle + y_gyro) + (1.0 - alpha) * (accel_angle);

    /*
    Serial.print("accel_pitch: ");
    Serial.println(accel_angle);
    */

    _pitch_angle = new_angle;

    return new_angle;
}

void HopperRobot::get_imu_data(){
    unsigned long curr_measure_time = micros();
    int rslt = bmi160.getAccelGyroData(_accel_gyro);
    if (rslt != 0){
        Serial.println("Cannot read IMU data from BMI160");
        return;
    }

    _dt = (curr_measure_time - _prev_sensor_time) / 1e6;

    float div_factors[2] = {kGyroFactor, kAccelFactor};
    for(int i = 0; i < 6; ++i)
        accel_gyro_values[i] = _accel_gyro[i] / div_factors[i / 3];
    
    _prev_sensor_time = curr_measure_time;
}

//Public Methods
void HopperRobot::homing_sequence(){
    bus.PollCAN();
    float des_pos[] = {get_extension_position(kLegIdxs[0]), get_extension_position(kLegIdxs[1])};

    long last_print = 0;
    long last_command = micros();
    while (true){
        bus.PollCAN();
        bool homing_done = true;
        float motor_torques[] = {0.0, 0.0};

        if(micros() - last_command > 5000) {
          for (int i = 0; i < 2; i++){
              if (!_homed_idxs[i]){
                  float torq_command = get_impedence_command(kLegIdxs[i], des_pos[i]);

                  if (abs(torq_command) > kHomingCurrentThreshold){
                      _homed_idxs[i] = true;
                      _homed_positions[i] = get_extension_position(kLegIdxs[i]);

                      Serial.print("homed index ");
                      Serial.println(i);
                      Serial.println(torq_command);
                      Serial.println(_homed_positions[i]);
                  }
                  else{
                      motor_torques[i] = torq_command;
                      des_pos[i] += kLegDirections[i] * kHomingVelocity;
                  }

                  homing_done = homing_done && _homed_idxs[i];

                  // Serial.println("--------------");
                  // Serial.print("Motor: ");
                  // Serial.println(i);
                  // Serial.print("current: ");
                  // Serial.println(torq_command);
              }
            }

          if (homing_done) break;

          // Serial.print(motor_torques[0]);
          // Serial.print(" ");
          // Serial.print(motor_torques[1]);
          // Serial.println();

          // Serial.print("dt: ");
          // Serial.println(micros() - last_print);
          last_print = micros();

          bus.CommandTorques(0, 0, 
                  motor_torques[0], motor_torques[1], C610Subbus::kOneToFourBlinks);
          last_command = micros();
      }
    }
}

float HopperRobot::get_pitch(bool in_degrees){
    if (in_degrees)
        return _pitch_angle;
    return _pitch_angle * kDegToRadians;
}

void HopperRobot::control_step(){
    get_imu_data();
    complimentaryFilter();

    float angular_vel = accel_gyro_values[1] * kDegToRadians;
    float wheel_vel = get_wheel_vel();
    float radian_est = get_pitch(false);

    float imu_array[] = {-1.0 * radian_est, 0.0, -1.0 * angular_vel, -1.0 * wheel_vel};
    float wheel_torque = get_wheel_torque(imu_array);

    set_motor_comms(wheel_torque);
}

void HopperRobot::PollCAN() {
  bus.PollCAN();
}