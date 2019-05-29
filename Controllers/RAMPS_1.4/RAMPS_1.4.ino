#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include "TimerOne.h"
#include "Stepper.h"

// RAMPS 1.4 pins from https://reprap.org/wiki/RAMPS_1.4

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62

#define POSITIONS_PUBLISH_FREQUENCY    20   //hz

StepperController stepper1(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
StepperController stepper2(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
StepperController stepper3(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);

ros::NodeHandle nh;

void positionCallback(const std_msgs::Int16MultiArray& cmd_vel_msg);
ros::Subscriber<std_msgs::Int16MultiArray> g_posSub("stepper_goal", positionCallback);

void enableCallback(const std_msgs::Int16& msg);
ros::Subscriber<std_msgs::Int16> g_enableSub("enable_motors", enableCallback);

std_msgs::Int16MultiArray g_posState;
ros::Publisher g_posPub("stepper_state", &g_posState);

uint32_t g_recvTime = 0;

void ROSLog(char* str)
{
    nh.loginfo(str);
}

void doOneStep()
{
    if ((millis()-g_recvTime) < 2*1000) // Sanity check that connection is alive
    {
        stepper1.run();
        stepper2.run();
        stepper3.run();
    }
}

void setup()
{
    // Initialize ROS node handle, advertise and subscribe the topics
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    nh.subscribe(g_posSub);
    nh.subscribe(g_enableSub);
    nh.advertise(g_posPub);

    int kMaxStepsPerSec = 2 * 200 * 16;
    stepper1.setMaxSpeed(kMaxStepsPerSec);
    stepper2.setMaxSpeed(kMaxStepsPerSec);
    stepper3.setMaxSpeed(kMaxStepsPerSec);

    Timer1.initialize(100);
    Timer1.attachInterrupt(doOneStep);
}

void publishPositions(void)
{
    int16_t data[3];

    data[0] = stepper1.getCurrentPosition();
    data[1] = stepper2.getCurrentPosition();
    data[2] = stepper3.getCurrentPosition();

    std_msgs::Int16MultiArray pos;
    pos.data_length = 3;
    pos.data        = data;

    g_posPub.publish(&pos);
}

void enableStepper(StepperController& motor, boolean enable)
{
    motor.enableMotor(enable);
}

void enableMotors(bool enableMotors)
{
    enableStepper(stepper1, enableMotors);
    enableStepper(stepper2, enableMotors);
    enableStepper(stepper3, enableMotors);
}

void loop()
{
    static uint32_t s_pubTime = 0;
    uint32_t t = millis();
    if ((t-s_pubTime) >= (1000 / POSITIONS_PUBLISH_FREQUENCY))
    {
        publishPositions();
        s_pubTime = t;
    }

    static bool s_rosConnected = false;
    if (nh.connected()) 
    {
        if (false == s_rosConnected) 
        {
            s_rosConnected = true;
            delay(10);  // Delay after connection for serial link establishing
            nh.loginfo("RAMPS connected");
        }
    }
    else 
    {
        s_rosConnected = false;
    }

    // Call all the callbacks waiting to be called at that point in time
    nh.spinOnce();
}

void positionCallback(const std_msgs::Int16MultiArray& posArray)
{
    if (posArray.data_length != 3)
    {
        nh.loginfo("Received incorrect data length!");
        return; 
    }

    g_recvTime = millis();

    stepper1.setTargetPos(posArray.data[0]);
    stepper2.setTargetPos(posArray.data[1]);
    stepper3.setTargetPos(posArray.data[2]);
}

void enableCallback(const std_msgs::Int16& msg)
{
    if (msg.data == 0) {
        nh.loginfo("Disable steppers");
        enableMotors(false);
    }
    else {
        nh.loginfo("Enable steppers");
        enableMotors(true);
    }
}

