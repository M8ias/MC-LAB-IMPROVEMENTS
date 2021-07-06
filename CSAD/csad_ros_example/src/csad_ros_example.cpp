
#include <ros/ros.h>
#include "csad_actuator_driver/csad_actuator.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "math.h"

#define pi 3.1415


float servoPos[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float prevLeftStickAngle = 0;
int revolutions = 0;
float motorPower = 0.0f;

void prosessControllerInput(const sensor_msgs::Joy msg){
    float leftStickAngle = atan2(msg.axes[0], msg.axes[1]);
    float leftTriggerValue = msg.axes[4];
    float rightTriggerValue = msg.axes[5];
    servoPos[0] = leftStickAngle;
    servoPos[1] = leftStickAngle;
    servoPos[2] = leftStickAngle;
    servoPos[3] = leftStickAngle;
    servoPos[4] = leftStickAngle;
    servoPos[5] = leftStickAngle;
    motorPower = rightTriggerValue - leftTriggerValue;

    ROS_INFO("%f, %f, %f", leftStickAngle, leftTriggerValue, rightTriggerValue);



}


int main(int argc, char* argv[])
{
    CSAD_Actuator servos;
    int size;

    //init
    ros::init(argc, argv, "Servo_controller");
    //message code
    ros::NodeHandle node_handle_;
    ros::Rate loop_rate(60);
    ros::Subscriber values_sub = node_handle_.subscribe("joy",1, prosessControllerInput);
    


    float counter = 0.0;

    uint8_t servoIds[] = {1, 2, 3, 4, 5, 6};
    servos.initServos();
    servos.openI2CPort("/dev/i2c-1", 100);
    
    while (ros::ok())
    {   

        counter = counter + (3.1415f/90.0f);

        ros::spinOnce();
        servos.setMotorPower(6, motorPower);
        servos.setMotorPower(4, motorPower);
        servos.setMotorPower(5, motorPower);
        servos.setMotorPower(1, motorPower);
        servos.setMotorPower(2, motorPower);
        servos.setMotorPower(3, motorPower);
        servos.setServoPositions(servoPos, servoIds);
        //servos.setServoPosition(servoPos[0],1);
        loop_rate.sleep();
    }
}