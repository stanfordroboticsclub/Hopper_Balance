#include <Arduino.h>
#include <HopperRobot.h>

HopperRobot::HopperRobot(){
    prev_sensor_time = micros();

    if (bmi160.softReset() != BMI160_OK){
        //Serial.println("reset false");
        while(1);
    }

    if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
        //Serial.println("init false");
        while(1);
    }
}

HopperRobot::~HopperRobot(){

}

//Private Methods
float HopperRobot::get_wheel_vel(){
    float left_vel = bus.Get(left_wheel_idx).Velocity();
    float right_vel = bus.Get(right_wheel_idx).Velocity();

    float output = 0.5 * (left_vel - right_vel);
    return output;
}

float HopperRobot::get_wheel_torque(float* imu_array){
    float op_sum = 0;
    int lqr_len = sizeof(lqr_gains) / sizeof(float);
    
    for (int i = 0; i < lqr_len; i++)
        op_sum += lqr_gains[i] * imu_array[i];
    
    return op_sum;
}

float HopperRobot::get_impedence_command(int motor_idx, float desired_pos){
    C610 esc = bus.Get(motor_idx);
    float motor_pos = esc.Position();
    float motor_vec = esc.Velocity();

    return impedence_alpha * (desired_pos - motor_pos) - impedence_beta * motor_vec;
}

void HopperRobot::set_motor_comms(float wheel_torque){
    //Calculate and send motor commands based on a specified wheel torque
    float torque_to_amps = 4.0;
    float amps_to_millis = 1000;

    float wheel_amps = wheel_torque * torque_to_amps * amps_to_millis;
    wheel_amps = constrain(wheel_amps, -7000, 7000);

    float motor_comm_arr[4];
  
    motor_comm_arr[right_extend_idx] = get_impedence_command(right_extend_idx, HEIGHT_POS);
    motor_comm_arr[left_extend_idx] = get_impedence_command(left_extend_idx, HEIGHT_POS);
    motor_comm_arr[left_wheel_idx] = wheel_amps;
    motor_comm_arr[right_wheel_idx] = -1.0 * wheel_amps;

    bus.CommandTorques(motor_comm_arr[0], motor_comm_arr[1], 
                        motor_comm_arr[2], motor_comm_arr[3], C610Subbus::kOneToFourBlinks);
}

float HopperRobot::complimentaryFilter(){
    float y_gyro = accel_gyro_values[1] * dt;
    float accel_angle = -atan2(accel_gyro_values[5], -accel_gyro_values[3]) * (180 / 3.14);

    float alpha = 0.99;

    float new_angle = alpha * (pitch_angle + y_gyro) + (1.0 - alpha) * (accel_angle);

    /*
    Serial.print("accel_pitch: ");
    Serial.println(accel_angle);
    */

    pitch_angle = new_angle;

    return new_angle;
}

void HopperRobot::get_imu_data(){
    unsigned long curr_measure_time = micros();
    int rslt = bmi160.getAccelGyroData(accelGyro);
    if (rslt != 0){
        Serial.println("Cannot read IMU data from BMI160");
        return;
    }

    dt = (curr_measure_time - prev_sensor_time) / 1e6;

    float div_factors[2] = {gyro_factor, accel_factor};
    for(int i = 0; i < 6; ++i)
        accel_gyro_values[0] = accelGyro[0] / div_factors[i / 3];
    
    prev_sensor_time = curr_measure_time;
}

//Public Methods
void HopperRobot::control_step(){
    get_imu_data();
    complimentaryFilter();

    float angular_vel = accel_gyro_values[1] * deg_to_radians;
    float wheel_vel = get_wheel_vel();
    float radian_est = pitch_angle * deg_to_radians;

    float imu_array[] = {-1.0 * radian_est, 0.0, -1.0 * angular_vel, -1.0 * wheel_vel};
    float wheel_torque = get_wheel_torque(imu_array);

    set_motor_comms(wheel_torque);
}