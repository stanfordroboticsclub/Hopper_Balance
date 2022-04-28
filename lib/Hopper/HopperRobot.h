#include <Wire.h>
#include <SPI.h>
#include <C610Bus.h>
#include <DFRobot_BMI160.h>
#include "lpfcoefs.h"

#define IDLE 0
#define BALANCING 1

class HopperRobot {
    private:
        int _state = IDLE;

        C610Bus<CAN1> bus;
        DFRobot_BMI160 bmi160;
        const int8_t kI2CAddr = 0x69;
         
        const float kTorqueToAmps = 4.0;
        const float kAmpsToMillis = 1000;

        const float kAccelFactor = 16384.0;
        const float kGyroFactor = 16.384;
        const float kDegToRadians = 3.14 / 180;
        const float kGearRatio = 36.0;
        const float kPitchOffset = 0.0;

        const float kWheelSpacing = 0.18;
        const float kWheelRadius = 0.028;

        const unsigned char kLeftWheelIdx = 0;
        const unsigned char kRightWheelIdx = 1;
        const unsigned char kLeftExtendIdx = 2;
        const unsigned char kRightExtendIdx = 3;

        // Max current
        float _max_current = 8000;
        float _max_torque = _max_current / kTorqueToAmps / kAmpsToMillis / kGearRatio;

        //Homing variables
        bool _homed_idxs[2] = {false, false};
        const int kLegIdxs[2] = {kLeftExtendIdx, kRightExtendIdx};

        const float kLegDirections[2] = {1, -1};
        float _homed_positions[2] = {0.0, 0.0};

        float _wheel_offsets[2] = {0.0, 0.0};

        const float kHomingVelocity = 0.0015;
        const float kHomingCurrentThreshold = 5000;

        float _height_pos = 0.6;

        unsigned long _prev_sensor_time;
        float _dt; //The time difference between the current measurement and the previous one in seconds
        int16_t _accel_gyro[6]={0};

        const float kLQRGains[4] = {-0.7563237,  -0.01118034, -0.08060046, -0.00765402};
        const float kKpYaw = 0.005;
        const float kKdYaw = 0.003;
        

        float _pitch_angle = 0; //Last measured pitch angle.  Updated by calls to complimentaryFilter

        float _left_wheel_pos, _right_wheel_pos, _left_wheel_vel, _right_wheel_vel;

        bool _foot_on_ground[2] = {false, false};

        void get_imu_data();
        void read_wheel_sensors();
        float get_wheel_pos();
        float get_wheel_vel();
        float get_yaw_angle();
        float get_yaw_rate();
        float get_extension_position(int legIndex);
        float get_balance_torque (float robot_state[4], float des_state[4]);
        float get_yaw_torque(float des_yaw);
        float get_impedence_command(int motor_idx, float desired_pos);

        void set_motor_comms(float left_leg_torque, float right_leg_torque, float left_wheel_torque, float right_wheel_torque);
        float complimentaryFilter();

        bool feet_on_ground();
        void transition_to_idle();
        void transition_to_balancing();

        float filter(float signal);

    public:

        float accel_gyro_values[6] = {0};

        //Impedence Gains for Leg Motors
        float _impedence_stiffness = 50000;
        float _impedence_damping = 1000;

        void test_impedence_hold(float position);

        void homing_sequence();

        int get_state();
        float get_pitch(bool in_degrees);
        void control_step(float des_state[4]);
        void PollCAN();

        HopperRobot();
        ~HopperRobot();
};