#include <Wire.h>
#include <SPI.h>
#include <C610Bus.h>
#include <DFRobot_BMI160.h>
#include "lpfcoefs.h"

#define IDLE 0
#define BALANCING 1

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

        const unsigned char kLeftWheelIdx = 0;
        const unsigned char kRightWheelIdx = 1;
        const unsigned char kLeftExtendIdx = 2;
        const unsigned char kRightExtendIdx = 3;

        // Max current
        float _max_current = 6000;

        //Homing variables
        bool _homed_idxs[2] = {false, false};
        const int kLegIdxs[2] = {kLeftExtendIdx, kRightExtendIdx};

        const float kLegDirections[2] = {1, -1};
        float _homed_positions[2] = {0.0, 0.0};

        float _wheel_offsets[2] = {0.0, 0.0};

        const float kHomingVelocity = 0.0015;
        const float kHomingCurrentThreshold = 5000;

        float _height_pos = 0.3;

        unsigned long _prev_sensor_time;
        float _dt; //The time difference between the current measurement and the previous one in seconds
        int16_t _accel_gyro[6]={0};

        float _lqr_gains[4] = {-6.029133,   -0.11180384, -0.88575554, -0.05971062};//{-36.618958, -0.7070806, -5.4262147, -0.37421146};
        

        float _pitch_angle = 0; //Last measured pitch angle.  Updated by calls to complimentaryFilter

        void get_imu_data();
        float get_wheel_pos();
        float get_wheel_vel();
        float get_extension_position(int legIndex);
        float get_wheel_torque (float* imu_array);
        float get_impedence_command(int motor_idx, float desired_pos);

        void set_motor_comms(float left_leg_torque, float right_leg_torque, float wheel_torque);
        float complimentaryFilter();

        bool feet_on_ground();
        void transition_to_idle();
        void transition_to_balancing();

        float filter(float signal);

    public:

        int state = IDLE;

        float accel_gyro_values[6] = {0};

        //Impedence Gains for Leg Motors
        float _impedence_stiffness = 50000;
        float _impedence_damping = 1000;

        bool _foot_on_ground[2] = {false, false};

        void test_impedence_hold(float position);

        void homing_sequence();

        float get_pitch(bool in_degrees);
        void control_step();
        void PollCAN();

        HopperRobot();
        ~HopperRobot();
};