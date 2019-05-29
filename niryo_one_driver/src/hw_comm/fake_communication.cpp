#include "niryo_one_driver/fake_communication.h"

// we stop at 1022 instead of 1023, to get an odd number of positions (1023)
// --> so we can get a middle point (511)
#define XL320_TOTAL_ANGLE          296.67
#define XL320_MAX_POSITION         1022
#define XL320_MIN_POSITION         0
#define XL320_MIDDLE_POSITION      511
#define XL320_TOTAL_RANGE_POSITION 1023

// we stop at 4094 instead of 4095, to get an odd number of positions (4095)
// --> so we can get a middle point (2047)
#define XL430_TOTAL_ANGLE          360.36
#define XL430_MAX_POSITION         4094
#define XL430_MIN_POSITION         0
#define XL430_MIDDLE_POSITION      2047
#define XL430_TOTAL_RANGE_POSITION 4095

#define RADIAN_TO_DEGREE 57.295779513082320876798154814105

static uint64_t GetTickCountMs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (uint64_t)(ts.tv_nsec / 1000000) + ((uint64_t)ts.tv_sec * 1000ull);
}

uint32_t rad_pos_to_xl320_pos(double position_rad)
{
    return (uint32_t) ((double)XL320_MIDDLE_POSITION + (position_rad * RADIAN_TO_DEGREE * (double)XL320_TOTAL_RANGE_POSITION) / (double) XL320_TOTAL_ANGLE );
}

double xl320_pos_to_rad_pos(uint32_t position_dxl)
{
    return (double) ((((double)position_dxl - XL320_MIDDLE_POSITION) * (double)XL320_TOTAL_ANGLE) / (RADIAN_TO_DEGREE * (double)XL320_TOTAL_RANGE_POSITION));
}


uint32_t rad_pos_to_xl430_pos(double position_rad)
{
    return (uint32_t) ((double)XL430_MIDDLE_POSITION + (position_rad * RADIAN_TO_DEGREE * (double)XL430_TOTAL_RANGE_POSITION) / (double) XL430_TOTAL_ANGLE );
}

double xl430_pos_to_rad_pos(uint32_t position_dxl)
{
    return (double) ((((double)position_dxl - XL430_MIDDLE_POSITION) * (double)XL430_TOTAL_ANGLE) / (RADIAN_TO_DEGREE * (double)XL430_TOTAL_RANGE_POSITION));
}

float kStepsPerRev = 200 * 16;

int32_t rad_pos_to_steps(double position_rad, double gear_ratio, double direction)
{
    return (int32_t) ((kStepsPerRev * gear_ratio * position_rad * RADIAN_TO_DEGREE / 360.0) * direction);
}

double steps_to_rad_pos(int32_t steps, double gear_ratio, double direction)
{
    return (double) ((double)steps * 360.0 / (kStepsPerRev * gear_ratio * RADIAN_TO_DEGREE)) * direction ;
}

void FakeCommunication::servo_state_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    //ROS_INFO("servo_state_callback: %d, %d", msg->data[0], msg->data[1]);

// 0: Axes 4
// 1: Axes 5
// 2: Axes 6

    curr_pos[3] = xl430_pos_to_rad_pos(msg->data[0]);
    curr_pos[4] = xl430_pos_to_rad_pos(XL430_MIDDLE_POSITION * 2 - msg->data[1]);
    curr_pos[5] = xl320_pos_to_rad_pos(msg->data[2]);

    m_timeServos = GetTickCountMs();

    //ROS_INFO("servo_state_callback: %.2f, %.2f, %.2f", curr_pos[3], curr_pos[4], curr_pos[5]);
}

void FakeCommunication::stepper_state_callback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    //ROS_INFO("stepper_state_callback: %d, %d, %d", msg->data[0], msg->data[1], msg->data[2]);

// 0: Axes 1
// 1: Axes 2
// 2: Axes 3

    m_m1.setPositionState(msg->data[0] + m_m1.getOffsetPosition());
    m_m2.setPositionState(msg->data[1] + m_m2.getOffsetPosition());
    m_m3.setPositionState(msg->data[2] + m_m3.getOffsetPosition());

    curr_pos[0] = steps_to_rad_pos(m_m1.getPositionState(), m_m1.getGearRatio(), m_m1.getDirection());
    curr_pos[1] = steps_to_rad_pos(m_m2.getPositionState(), m_m2.getGearRatio(), m_m2.getDirection());
    curr_pos[2] = steps_to_rad_pos(m_m3.getPositionState(), m_m3.getGearRatio(), m_m3.getDirection());

    m_timeSteppers = GetTickCountMs();

    //ROS_INFO("stepper_state_callback: %.2f, %.2f, %.2f", curr_pos[0], curr_pos[1], curr_pos[2]);
}

void FakeCommunication::gripper_goal_callback(const std_msgs::Int16& msg)
{
    if (m_fake_communication)
    {
        std_msgs::Int32MultiArray msgServo;
        msgServo.data.clear();
        msgServo.data.push_back(curr_pos[3]);
        msgServo.data.push_back(curr_pos[4]);
        msgServo.data.push_back(curr_pos[5]);
        msgServo.data.push_back(msg.data);
        m_servo_state_pub.publish(msgServo);            
    }
}

FakeCommunication::FakeCommunication(int hardware_version)
{
    ROS_INFO("Starting Fake Communication... It will just echo cmd into current position");

    this->hardware_version = hardware_version;

    double pos_0, pos_1, pos_2;
    ros::param::get("/niryo_one/motors/stepper_1_home_position", pos_0);
    ros::param::get("/niryo_one/motors/stepper_2_home_position", pos_1);
    ros::param::get("/niryo_one/motors/stepper_3_home_position", pos_2);

    if (hardware_version == 1) {
        double pos_3;
        ros::param::get("/niryo_one/motors/stepper_4_home_position", pos_3);

        curr_pos[0] = pos_0;
        curr_pos[1] = pos_1;
        curr_pos[2] = pos_2;
        curr_pos[3] = pos_3;
        curr_pos[4] = 0.0;
        curr_pos[5] = 0.0;
    }
    else if (hardware_version == 2) {
        curr_pos[0] = pos_0;
        curr_pos[1] = pos_1;
        curr_pos[2] = pos_2;
        curr_pos[3] = 0.0;
        curr_pos[4] = 0.0;
        curr_pos[5] = 0.0;
    }

    m_gripperPos = 0;
    m_motorsOnline = true;
}

int FakeCommunication::init()
{
    ros::param::get("~fake_communication", m_fake_communication);

    double gear_ratio_1, gear_ratio_2, gear_ratio_3, gear_ratio_4;
    ros::param::get("/niryo_one/motors/stepper_1_gear_ratio", gear_ratio_1);
    ros::param::get("/niryo_one/motors/stepper_2_gear_ratio", gear_ratio_2);
    ros::param::get("/niryo_one/motors/stepper_3_gear_ratio", gear_ratio_3);
    ros::param::get("/niryo_one/motors/stepper_4_gear_ratio", gear_ratio_4);
    ROS_INFO("Gear ratios : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf)", gear_ratio_1, gear_ratio_2, gear_ratio_3, gear_ratio_4);

    double home_position_1, home_position_2, home_position_3, home_position_4;
    ros::param::get("/niryo_one/motors/stepper_1_home_position", home_position_1);
    ros::param::get("/niryo_one/motors/stepper_2_home_position", home_position_2);
    ros::param::get("/niryo_one/motors/stepper_3_home_position", home_position_3);
    ros::param::get("/niryo_one/motors/stepper_4_home_position", home_position_4);
    ROS_INFO("Home positions : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf)", home_position_1, home_position_2, home_position_3, home_position_4);

    double offset_position_1, offset_position_2, offset_position_3, offset_position_4;
    ros::param::get("/niryo_one/motors/stepper_1_offset_position", offset_position_1);
    ros::param::get("/niryo_one/motors/stepper_2_offset_position", offset_position_2);
    ros::param::get("/niryo_one/motors/stepper_3_offset_position", offset_position_3);
    ros::param::get("/niryo_one/motors/stepper_4_offset_position", offset_position_4);
    ROS_INFO("Angle offsets : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf)", offset_position_1, offset_position_2, offset_position_3, offset_position_4);

    double direction_1, direction_2, direction_3, direction_4;
    ros::param::get("/niryo_one/motors/stepper_1_direction", direction_1);
    ros::param::get("/niryo_one/motors/stepper_2_direction", direction_2);
    ros::param::get("/niryo_one/motors/stepper_3_direction", direction_3);
    ros::param::get("/niryo_one/motors/stepper_4_direction", direction_4);

    int max_effort_1, max_effort_2, max_effort_3, max_effort_4;
    ros::param::get("/niryo_one/motors/stepper_1_max_effort", max_effort_1);
    ros::param::get("/niryo_one/motors/stepper_2_max_effort", max_effort_2);
    ros::param::get("/niryo_one/motors/stepper_3_max_effort", max_effort_3);
    ros::param::get("/niryo_one/motors/stepper_4_max_effort", max_effort_4);

    // Create motors with previous params
    m_m1 = StepperMotorState("Stepper Axis 1", 1, gear_ratio_1, direction_1,
            rad_pos_to_steps(home_position_1, gear_ratio_1, direction_1),            // home position
            rad_pos_to_steps(offset_position_1, gear_ratio_1, direction_1),          // offset position
            16, max_effort_1);
    m_m2 = StepperMotorState("Stepper Axis 2", 2, gear_ratio_2, direction_2,
            rad_pos_to_steps(home_position_2, gear_ratio_2, direction_2),
            rad_pos_to_steps(offset_position_2, gear_ratio_2, direction_2),
            16, max_effort_2);
    m_m3 = StepperMotorState("Stepper Axis 3", 3, gear_ratio_3, direction_3,
            rad_pos_to_steps(home_position_3, gear_ratio_3, direction_3),
            rad_pos_to_steps(offset_position_3, gear_ratio_3, direction_3),
            16, max_effort_3);

    return 0; // success
}

void FakeCommunication::manageLoop()
{
    ROS_INFO("FakeCommunication::manageLoop");

    m_servo_goal_pub   = m_nh.advertise<std_msgs::Int16MultiArray>("servo_goal", 1);
    m_stepper_goal_pub = m_nh.advertise<std_msgs::Int16MultiArray>("stepper_goal", 1);

    //ros::Publisher enableMotors_pub  = m_nh.advertise<std_msgs::Int16>("enable_motors", 1);

    m_servo_state_pub = m_nh.advertise<std_msgs::Int32MultiArray>("/servo_state", 1);

    ros::Subscriber servo_state_subscriber;
    servo_state_subscriber = m_nh.subscribe("/servo_state", 1, &FakeCommunication::servo_state_callback, this);

    ros::Subscriber stepper_state_subscriber;
    stepper_state_subscriber = m_nh.subscribe("/stepper_state", 1, &FakeCommunication::stepper_state_callback, this);

    ros::Subscriber gripper_goal_subscriber;
    gripper_goal_subscriber = m_nh.subscribe("/gripper_goal", 1, &FakeCommunication::gripper_goal_callback, this);

    sleep(5);   // Give time to init everything

    m_timeServos   = GetTickCountMs();
    m_timeSteppers = GetTickCountMs();

    ros::Rate rate = ros::Rate(10.0);
    while (ros::ok())
    {
        if (!m_fake_communication)
        {
            uint64_t now = GetTickCountMs();
            bool servosOnline   = ((now - m_timeServos  ) < 2000);
            bool steppersOnline = ((now - m_timeSteppers) < 2000);

            if (m_motorsOnline)
            {
                if (!servosOnline) {
                    ROS_ERROR("Servos offline");
                    m_motorsOnline = false;
                }

                if (!steppersOnline) {
                    ROS_ERROR("Stepers offline");
                    m_motorsOnline = false;
                }

                if (!m_motorsOnline)
                {
                    ROS_ERROR("Connection lost");
                    //std_msgs::Int16 msgEnable;
                    //msgEnable.data = 0;
                    //enableMotors_pub.publish(msgEnable);
                }
            }

            if (servosOnline && steppersOnline)
                m_motorsOnline = true;
        }

        rate.sleep();
    }

    ROS_INFO("FakeCommunication::manageLoop(): exit");
}

void FakeCommunication::manageHardwareConnection()
{
    m_connection_loop_thread.reset(new std::thread(boost::bind(&FakeCommunication::manageLoop, this)));
}

void FakeCommunication::startHardwareControlLoop()
{
    ROS_INFO("Start hardware control loop : nothing to do in fake mode.");
}

void FakeCommunication::stopHardwareControlLoop()
{
    // nothing
}

void FakeCommunication::resumeHardwareControlLoop()
{
    // nothing
}

void FakeCommunication::synchronizeMotors(bool begin_traj)
{
    // nothing
}

bool FakeCommunication::isConnectionOk()
{
    return m_motorsOnline || m_fake_communication;
}

int FakeCommunication::allowMotorsCalibrationToStart(int mode, std::string &result_message)
{
    ROS_INFO("Motor calibration with mode : %d", mode);
    return 1;
}

void FakeCommunication::requestNewCalibration()
{
    // nothing
}

bool FakeCommunication::isCalibrationInProgress()
{
    return false;
}

void FakeCommunication::sendPositionToRobot(const double cmd[6])
{
    //ROS_INFO("sendPositionToRobot: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);

// cmd:
// 0: Axes 1
// 1: Axes 2
// 2: Axes 3
// 3: Axes 4
// 4: Axes 5
// 5: Axes 6

    if (m_fake_communication) {
        // Echo all
        for (int i = 0; i < 6; i++)
            curr_pos[i] = cmd[i];
    }

    // Do not send too often to slow OpenCM_9.04
    static int s_sendCounter = 0;
    if (++s_sendCounter >= 4)
    {
        s_sendCounter = 0;

        // 0: Axes 4
        // 1: Axes 5
        // 2: Axes 6
        std_msgs::Int16MultiArray msgServo;
        msgServo.data.clear();
        msgServo.data.push_back(rad_pos_to_xl430_pos(cmd[3]));
        msgServo.data.push_back(XL430_MIDDLE_POSITION * 2 - rad_pos_to_xl430_pos(cmd[4]));
        msgServo.data.push_back(rad_pos_to_xl320_pos(cmd[5]));
        m_servo_goal_pub.publish(msgServo);
    }

    // 0: Axes 1
    // 1: Axes 2
    // 2: Axes 3
    m_m1.setPositionCommand(rad_pos_to_steps(cmd[0], m_m1.getGearRatio(), m_m1.getDirection()));
    m_m2.setPositionCommand(rad_pos_to_steps(cmd[1], m_m2.getGearRatio(), m_m2.getDirection()));
    m_m3.setPositionCommand(rad_pos_to_steps(cmd[2], m_m3.getGearRatio(), m_m3.getDirection()));

    std_msgs::Int16MultiArray msgStepper;
    msgStepper.data.clear();
    msgStepper.data.push_back(m_m1.getPositionCommand() - m_m1.getOffsetPosition());
    msgStepper.data.push_back(m_m2.getPositionCommand() - m_m2.getOffsetPosition());
    msgStepper.data.push_back(m_m3.getPositionCommand() - m_m3.getOffsetPosition());
    m_stepper_goal_pub.publish(msgStepper);
}

void FakeCommunication::getCurrentPosition(double pos[6])
{
    for (int i = 0 ; i < 6 ; i++) {
        pos[i] = curr_pos[i];
    }
}

void FakeCommunication::addCustomDxlCommand(int motor_type, uint8_t id, uint32_t value,
        uint32_t reg_address, uint32_t byte_number)
{
    ROS_INFO("Add custom Dxl command");
}

void FakeCommunication::getHardwareStatus(bool *is_connection_ok, std::string &error_message,
        int *calibration_needed, bool *calibration_in_progress,
        std::vector<std::string> &motor_names, std::vector<std::string> &motor_types,
        std::vector<int32_t> &temperatures, std::vector<double> &voltages,
        std::vector<int32_t> &hw_errors)
{
    //ROS_INFO("Get Hardware Status");
    *(is_connection_ok) = true;
    *(calibration_needed) = false;
    *(calibration_in_progress) = false;
}

void FakeCommunication::getFirmwareVersions(std::vector<std::string> &motor_names,
        std::vector<std::string> &firmware_versions)
{
    // ROS_INFO("Get firmware versions")
}

void FakeCommunication::activateLearningMode(bool activate)
{
    ROS_INFO("Activate learning mode : %d", activate);
}

bool FakeCommunication::setLeds(std::vector<int> &leds, std::string &message)
{
//    ROS_INFO("Set leds");
    return true;
}

int FakeCommunication::pullAirVacuumPump(uint8_t id, uint16_t pull_air_position, uint16_t pull_air_hold_torque)
{
    ROS_INFO("Pull air on vacuum pump with id : %03d", id);
    return 0; //VACUUM_PUMP_STATE_PULLED;
}
int FakeCommunication::pushAirVacuumPump(uint8_t id, uint16_t push_air_position)
{
    ROS_INFO("Push air on vacuum pump with id : %03d", id);
    return 0; //VACUUM_PUMP_STATE_PUSHED;
}

int FakeCommunication::pingAndSetDxlTool(uint8_t id, std::string name)
{
    ROS_INFO("Ping gripper with id : %03d", id);
    return 0; //TOOL_STATE_PING_OK;
}

int FakeCommunication::openGripper(uint8_t id, uint16_t open_position, uint16_t open_speed, uint16_t open_hold_torque)
{
    ROS_INFO("Open gripper with id : %03d", id);

    m_gripperPos = 0;

    return 0; //GRIPPER_STATE_OPEN;
}

int FakeCommunication::closeGripper(uint8_t id, uint16_t close_position, uint16_t close_speed, uint16_t close_hold_torque, uint16_t close_max_torque)
{
    ROS_INFO("Close gripper with id : %03d", id);

    m_gripperPos = 0;

    return 1023;
}
