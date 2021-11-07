#include <Wire.h>
#include <SPI.h>
#include <C610Bus.h>
#include <DFRobot_BMI160.h>

class HopperRobot {
    private:
        C610Bus<CAN1> bus;
        DFRobot_BMI160 bmi160;
        const int8_t i2c_addr = 0x69;

         
        float accel_factor = 16384.0;
        float gyro_factor = 16.384;
        float deg_to_radians = 3.14 / 180;

        char left_wheel_idx = 0;
        char right_wheel_idx = 1;
        char left_extend_idx = 2;
        char right_extend_idx = 3;
        float HEIGHT_POS = 0;

        unsigned long prev_sensor_time;
        float dt; //The time difference between the current measurement and the previous one in seconds

        void get_imu_data();
        float get_wheel_vel();
        float get_wheel_torque (float* imu_array);
        float get_impedence_command(int motor_idx, float desired_pos);
        void set_motor_comms(float wheel_torque);
        float complimentaryFilter();

    public:
        int16_t accelGyro[6]={0};
        float lqr_gains[4] = {-1.3963862e+01, 0.0, 
                                -2.5197718e+00, -9.9999182e-02};
        
        float accel_gyro_values[6] = {0};

        //Impedence Gains for Leg Motors
        float impedence_alpha = 50000;
        float impedence_beta = 2000;

        float pitch_angle = 0; //Last measured pitch angle.  Updated by calls to complimentaryFilter

        void control_step();

        HopperRobot();
        ~HopperRobot();
};