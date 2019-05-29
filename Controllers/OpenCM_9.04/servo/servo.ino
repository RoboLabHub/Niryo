// ROS stuff
#include <ros_.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;

// ROS logging
char g_logStr[200] = {0};
void LogInfo (const char* str) { nh.loginfo(str); }
void LogInfo1(const char* str) { strcat(g_logStr, str); }

#define POSITIONS_PUBLISH_FREQUENCY    10   //hz

void posCallback(const std_msgs::Int16MultiArray& msg);
ros::Subscriber<std_msgs::Int16MultiArray> g_goalSub("servo_goal", posCallback);

void gripperCallback(const std_msgs::Int16& msg);
ros::Subscriber<std_msgs::Int16> g_gripperSub("gripper_goal", gripperCallback);

void enableCallback(const std_msgs::Int16& msg);
ros::Subscriber<std_msgs::Int16> g_enableSub("enable_motors", enableCallback);

std_msgs::Int32MultiArray g_posState;
ros::Publisher g_posPub("servo_state", &g_posState);

#include "dmx_driver.h"

DMXDriver dmx_driver;

boolean g_motorsEnabled = false;

void enableMotors(bool enable)
{
    dmx_driver.setTorque(1, enable);
    dmx_driver.setTorque(2, enable);
    dmx_driver.setTorque(3, enable, false);
    dmx_driver.setTorque(4, enable, false);

    g_motorsEnabled = enable;
}

void posCallback(const std_msgs::Int16MultiArray& msg)
{
// 0: Axes 4
// 1: Axes 5
// 2: Axes 6

    if (!g_motorsEnabled)
        return;

    if (msg.data_length != 3)
    {
        LogInfo("Received incorrect data length!");
        return; 
    }
  
    dmx_driver.setPosition(1, msg.data[0]);
    dmx_driver.setPosition(2, msg.data[1]);

    dmx_driver.setPosition(3, msg.data[2], false);

    //char log_msg[50];
    //sprintf(log_msg, "vel [v:%0.2f, a:%0.2f]", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
    //nh.loginfo(log_msg);
}

void gripperCallback(const std_msgs::Int16& msg)
{
    dmx_driver.setPosition(4, msg.data, false);
}

void log_dmx_err(char* prefix, int id, bool xl430)
{
    byte err = 0;
    dmx_driver.getHWError(id, err, xl430);

    char err_msg[50] = {0};
    sprintf(err_msg, "%s: 0x%X", prefix, (int)err);
    nh.logerror(err_msg);
}

void publishPositions(void)
{
    int32_t data[4] = {0};

    if (!dmx_driver.getEncoder(1, data[0])) {       // Axes 4
        log_dmx_err("HW1 err", 1, true);
        return;
    }
  
    if (!dmx_driver.getEncoder(2, data[1])) {       // Axes 5
        log_dmx_err("HW2 err", 2, true);
        return;
    }

    if (!dmx_driver.getPositionXL320(3, data[2])) { // Axes 6
        log_dmx_err("HW3 err", 3, false);
        return;
    }

    if (!dmx_driver.getPositionXL320(4, data[3])) { // Gripper
        log_dmx_err("HW4 err", 4, false);
        return;
    }
  
    std_msgs::Int32MultiArray pos;
    pos.data_length = 4;
    pos.data = data;
  
    g_posPub.publish(&pos);
}

void setup()
{
    g_logStr[0] = 0;

    // Initialize ROS node handle, advertise and subscribe the topics
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    nh.subscribe(g_goalSub);
    nh.subscribe(g_enableSub);
    nh.subscribe(g_gripperSub);
    
    nh.advertise(g_posPub);

    dmx_driver.init();
    delay(1000);
    enableMotors(false);

    dmx_driver.setOperatingMode(1, kMode_Position);
    dmx_driver.setOperatingMode(2, kMode_Position);
    dmx_driver.setOperatingMode(3, kMode_Joint);
    dmx_driver.setOperatingMode(4, kMode_Joint);

    dmx_driver.setMaxTorqueXL320(4, 128);
}

void enableCallback(const std_msgs::Int16& msg)
{
    if (msg.data == 0) {
        nh.loginfo("Disable servos");
        enableMotors(false);
    }
    else {
        nh.loginfo("Enable servos");
        enableMotors(true);
    }
}

void loop()
{
    static uint32_t pubTime = 0;

    uint32_t t = millis();
    if ((t-pubTime) >= (1000 / POSITIONS_PUBLISH_FREQUENCY))
    {
        publishPositions();
        pubTime = t;
    }

    // Call all the callbacks waiting to be called at that point in time
    nh.spinOnce();

    static bool rosConnected = false;
    if (nh.connected()) 
    {
        if (false == rosConnected) 
        {
            rosConnected = true;

            if (strlen(g_logStr) > 0) 
            {
                LogInfo(g_logStr);
                g_logStr[0] = 0;
            }
            LogInfo("OpenCM connected");
        }
    }
    else {
        if (rosConnected)
          nh.initNode();  // Try to reinit node in case of failure
          
        rosConnected = false;
    }
}

