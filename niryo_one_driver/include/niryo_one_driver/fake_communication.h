#ifndef FAKE_COMMUNICATION_H
#define FAKE_COMMUNICATION_H

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <vector>
#include <stdint.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include "niryo_one_driver/stepper_motor_state.h"
#include "niryo_one_driver/communication_base.h"

//#include "niryo_one_driver/dxl_motor_state.h" // for gripper enums

class FakeCommunication : public CommunicationBase {

    public:
        FakeCommunication(int hardware_version);
        int init();

        void servo_state_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
        void stepper_state_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
        void gripper_goal_callback(const std_msgs::Int16& msg);

        void manageHardwareConnection();
        bool isConnectionOk();

        void startHardwareControlLoop();
        void stopHardwareControlLoop();
        void resumeHardwareControlLoop();

        void getCurrentPosition(double pos[6]);

        void getHardwareStatus(bool *is_connection_ok, std::string &error_message,
                int *calibration_needed, bool *calibration_in_progress,
                std::vector<std::string> &motor_names, std::vector<std::string> &motor_types,
                std::vector<int32_t> &temperatures,
                std::vector<double> &voltages, std::vector<int32_t> &hw_errors);

        void getFirmwareVersions(std::vector<std::string> &motor_names,
                std::vector<std::string> &firmware_versions);

        void sendPositionToRobot(const double cmd[6]);
        void activateLearningMode(bool activate);
        bool setLeds(std::vector<int> &leds, std::string &message);

        int allowMotorsCalibrationToStart(int mode, std::string &result_message);
        void requestNewCalibration();
        bool isCalibrationInProgress();

        // tools
        int pingAndSetDxlTool(uint8_t id, std::string name);

        int openGripper(uint8_t id, uint16_t open_position, uint16_t open_speed, uint16_t open_hold_torque);
        int closeGripper(uint8_t id, uint16_t close_position, uint16_t close_speed, uint16_t close_hold_torque, uint16_t close_max_torque);

        int pullAirVacuumPump(uint8_t id, uint16_t pull_air_position, uint16_t pull_air_hold_torque);
        int pushAirVacuumPump(uint8_t id, uint16_t push_air_position);

        // steppers
        void synchronizeMotors(bool begin_traj);

        void addCustomDxlCommand(int motor_type, uint8_t id, uint32_t value,
                uint32_t reg_address, uint32_t byte_number);

    private:
        StepperMotorState m_m1;
        StepperMotorState m_m2;
        StepperMotorState m_m3;

        bool m_fake_communication;

        int hardware_version;

        int m_gripperPos;

        uint64_t m_timeServos;
        uint64_t m_timeSteppers;

        bool    m_motorsOnline;

        void manageLoop();
        boost::shared_ptr<std::thread> m_connection_loop_thread;

        ros::NodeHandle m_nh;
        ros::Publisher m_servo_goal_pub;
        ros::Publisher m_stepper_goal_pub;
        ros::Publisher m_servo_state_pub;

        double curr_pos[6]; // just store cmd in this array

};

#endif
