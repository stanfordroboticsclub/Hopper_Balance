#include <Wire.h>
#include <SPI.h>
#include <C610Bus.h>
#include <DFRobot_BMI160.h>

class HopperRobot {
    private:
        C610Bus<CAN1> bus;
        DFRobot_BMI160 bmi160;
        const int8_t kI2CAddr = 0x69;
         
        const float kTorqueToAmps = 4.0;
        const float kAmpsToMillis = 1000;

        const float kAccelFactor = 16384.0;
        const float kGyroFactor = 16.384;
        const float kDegToRadians = 3.14 / 180;

        const char kLeftWheelIdx = 0;
        const char kRightWheelIdx = 1;
        const char kLeftExtendIdx = 2;
        const char kRightExtendIdx = 3;

        //Homing variables
        bool _homed_idxs[2] = {false, false};
        const int kLegIdxs[2] = {kLeftExtendIdx, kRightExtendIdx};

        const float kLegDirections[2] = {1, -1};
        float _homed_positions[2] = {0.0, 0.0};

        const float kHomingVelocity = 0.0001;
        const float kHomingCurrentThreshold = 30000; //5000;

        float _height_pos = 0;

        unsigned long _prev_sensor_time;
        float _dt; //The time difference between the current measurement and the previous one in seconds
        int16_t _accel_gyro[6]={0};

        float _lqr_gains[4] = {-1.3963862e+01, 0.0, 
                                -2.5197718e+00, -9.9999182e-02};
        

        float _pitch_angle = 0; //Last measured pitch angle.  Updated by calls to complimentaryFilter

        void get_imu_data();
        float get_wheel_vel();
        float get_extension_position(int legIndex);
        float get_wheel_torque (float* imu_array);
        float get_impedence_command(int motor_idx, float desired_pos);

        float get_impedence_command(int motor_idx, float desired_pos, float max_current);

        void set_motor_comms(float wheel_torque);
        float complimentaryFilter();

    public:
        float accel_gyro_values[6] = {0};

        //Impedence Gains for Leg Motors
        float _impedence_alpha = 0; //50000;
        float _impedence_beta = 2000;

        void test_impedence_hold(float position);

        void homing_sequence();

        float get_pitch(bool in_degrees);
        void control_step();

        HopperRobot();
        ~HopperRobot();
};