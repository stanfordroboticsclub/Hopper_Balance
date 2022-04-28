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
void HopperRobot::read_wheel_sensors() {
    _left_wheel_pos = -kGearRatio * (bus.Get(kLeftWheelIdx).Position() - _wheel_offsets[kLeftWheelIdx]);
    _right_wheel_pos = kGearRatio * (bus.Get(kRightWheelIdx).Position() - _wheel_offsets[kRightWheelIdx]);

    _left_wheel_vel = -kGearRatio * bus.Get(kLeftWheelIdx).Velocity();
    _right_wheel_vel = kGearRatio * bus.Get(kRightWheelIdx).Velocity();
}

float HopperRobot::get_wheel_pos(){
    float output = 0.5 * (_left_wheel_pos + _right_wheel_pos);
    return output;
}

float HopperRobot::get_wheel_vel(){
    float output = 0.5 * (_left_wheel_vel + _right_wheel_vel);
    return output;
}

float HopperRobot::get_yaw_angle(){
    float output = kWheelRadius / kWheelSpacing * (_left_wheel_pos - _right_wheel_pos);
    return output;
}

float HopperRobot::get_yaw_rate(){
    float output = kWheelRadius / kWheelSpacing * (_left_wheel_vel - _right_wheel_vel);
    return output;
}

float HopperRobot::get_extension_position(int legIndex){
    C610 esc = bus.Get(legIndex);
    float motor_pos = esc.Position();
    return motor_pos;
}

float HopperRobot::get_balance_torque(float robot_state[4], float des_state[4]){
    float op_sum = 0;
    for (int i = 0; i < 4; i++)
        op_sum += kLQRGains[i] * (des_state[i] - robot_state[i]);
    
    return op_sum;
}

float HopperRobot::get_yaw_torque(float des_yaw, float des_yaw_rate) {
    return kKpYaw * (des_yaw - get_yaw_angle()) + kKdYaw * (des_yaw_rate - get_yaw_rate());
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

void HopperRobot::set_motor_comms(float left_leg_torque, float right_leg_torque, float left_wheel_torque, float right_wheel_torque){
    //Calculate and send motor commands based on a specified wheel torque

    float left_wheel_amps = left_wheel_torque * kTorqueToAmps * kAmpsToMillis * kGearRatio;
    left_wheel_amps = constrain(left_wheel_amps, -_max_current, _max_current);
    float right_wheel_amps = right_wheel_torque * kTorqueToAmps * kAmpsToMillis * kGearRatio;
    right_wheel_amps = constrain(right_wheel_amps, -_max_current, _max_current);
    // Serial.println(wheel_amps);

    float motor_comm_arr[4];
  
    motor_comm_arr[kRightExtendIdx] = right_leg_torque;
    motor_comm_arr[kLeftExtendIdx] = left_leg_torque;

    motor_comm_arr[kLeftWheelIdx] = -left_wheel_amps;
    motor_comm_arr[kRightWheelIdx] = right_wheel_amps;

    // Serial.print(motor_comm_arr[0]);
    // Serial.print(" ");
    // Serial.print(motor_comm_arr[1]);
    // Serial.print(" ");
    // Serial.print(motor_comm_arr[2]);
    // Serial.print(" ");
    // Serial.print(motor_comm_arr[3]);
    // Serial.println();

    bus.CommandTorques(motor_comm_arr[0], motor_comm_arr[1], 
                        motor_comm_arr[2], motor_comm_arr[3], C610Subbus::kOneToFourBlinks);
}

float HopperRobot::complimentaryFilter(){
    float y_gyro = -accel_gyro_values[1] * _dt;
    float accel_angle = atan2(accel_gyro_values[5], -accel_gyro_values[3]) * (180 / 3.14);

    float alpha = 0.99;

    float new_angle = alpha * (_pitch_angle + y_gyro) + (1.0 - alpha) * (accel_angle);

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

bool HopperRobot::feet_on_ground() {
    return _foot_on_ground[0] && _foot_on_ground[1];
}

void HopperRobot::transition_to_idle() {
    _state = IDLE;
}

void HopperRobot::transition_to_balancing() {
    _state = BALANCING;
    _wheel_offsets[kLeftWheelIdx] = bus.Get(kLeftWheelIdx).Position();
    _wheel_offsets[kRightWheelIdx] = bus.Get(kRightWheelIdx).Position();
}

//Public Methods
int HopperRobot::get_state() {
    return _state;
}

void HopperRobot::homing_sequence(){
    bus.PollCAN();
    float des_pos[] = {get_extension_position(kLegIdxs[0]), get_extension_position(kLegIdxs[1])};

    // long last_print = 0;
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
              }
            }
          if (homing_done) break;
          bus.CommandTorques(0, 0, 
                  motor_torques[0], motor_torques[1], C610Subbus::kOneToFourBlinks);
          last_command = micros();
      }
    }
    digitalWrite(13, LOW);
}

float HopperRobot::get_pitch(bool in_degrees){
    if (in_degrees)
        return _pitch_angle;
    return _pitch_angle * kDegToRadians;
}

float HopperRobot::filter(float signal) {
    static float imu_input_buffer[NL];
    static int i = 0;

    static float imu_output_buffer[DL - 1];
    static int j = 0;

    imu_input_buffer[i] = signal;

    float filtered = 0.0;
    for (int n = 0; n < NL; n++) {
        filtered += NUM[n] * imu_input_buffer[i];
        i--;
        if (i < 0) i = NL - 1;
    }
    for (int n = 1; n < DL; n++) {
        filtered -= DEN[n] * imu_output_buffer[j];
        j--;
        if (j < 0) j = DL - 2;
    }

    i++;
    if (i == NL) i = 0;

    j++;
    if (j == DL - 1) j = 0;

    imu_output_buffer[j] = filtered;
    return filtered;
}

void HopperRobot::control_step(float des_state[6]){
    get_imu_data();
    complimentaryFilter();
    read_wheel_sensors();

    float angular_vel = -accel_gyro_values[1] * kDegToRadians;
    float filtered_angular_vel = filter(angular_vel);
    float wheel_pos = get_wheel_pos();
    float wheel_vel = get_wheel_vel();
    float radian_est = get_pitch(false);
    float robot_state[4] = {radian_est, wheel_pos, filtered_angular_vel, wheel_vel};
    // Serial.println(radian_est, 4);
    // for (int i = 0; i < 4; i++) {
    //     Serial.print(100.0 * robot_state[i]);
    //     Serial.print(' ');
    // }
    // Serial.println();
    float left_wheel_torque = 0;
    float right_wheel_torque = 0;

    float right_leg_torque = get_impedence_command(kRightExtendIdx, -_height_pos * kLegDirections[kRightExtendIdx - kLegIdxs[0]]);
    float left_leg_torque = get_impedence_command(kLeftExtendIdx, -_height_pos * kLegDirections[kLeftExtendIdx - kLegIdxs[0]]);

    _foot_on_ground[0] = left_leg_torque * kLegDirections[0] < -1000;
    _foot_on_ground[1] = right_leg_torque * kLegDirections[1] < -1000;

    if (get_state() == IDLE) {
        if (feet_on_ground()) {
            transition_to_balancing();
        }
    } else if (get_state() == BALANCING){
        if (feet_on_ground()) {
            float balance_torque = get_balance_torque(robot_state, des_state);
            float margin_torque = max(0.75 * _max_torque - abs(balance_torque), 0);
            float yaw_torque = constrain(get_yaw_torque(des_state[4], des_state[5]), -margin_torque, margin_torque);
            left_wheel_torque = balance_torque + yaw_torque;
            right_wheel_torque = balance_torque - yaw_torque;
        } else {
            transition_to_idle();
        }   
    }

    set_motor_comms(left_leg_torque, right_leg_torque, left_wheel_torque, right_wheel_torque);
}

void HopperRobot::PollCAN() {
  bus.PollCAN();
}