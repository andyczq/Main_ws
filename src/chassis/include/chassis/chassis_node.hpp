#ifndef __CHASSIS_NODE_HPP_
#define __CHASSIS_NODE_HPP_

#include "ros/ros.h"
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
// #include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <string.h>

#define PI      3.1415926f

// #define FRAME_HEAD  0x5A
// #define FRAME_TAIL  0x00
typedef struct __vel_tgrobot
{
    float X,Y,Z,rad_yaw;
}Vel_Tgrobot;

typedef struct __pose_tgrobot
{
    float X,Y;
}Pose_Tgrobot;

typedef struct __odom_tgrobot {
    Vel_Tgrobot vel;
    Pose_Tgrobot pose;
}Odom_Tgrobot;

class turn_on_tgrobot 
{
public:
    turn_on_tgrobot();
    ~turn_on_tgrobot();
    void tgrobot_controller();

private:
    serial::Serial tgrobot_serial_port;
    std::string serial_port_name, odom_frame_id, base_frame_id;
    int serial_baud_rate;
    ros::Subscriber twist_cmd_vel;
    void CMD_Vel_Callback(const geometry_msgs::Twist &twist_aux);
    uint8_t Check_CRC(uint8_t *data, uint8_t len);

    ros::Publisher battery_pub, odometer_pub;
    ros::Timer battery_timer, odometer_timer;

    void BatteryPub_TimerCallback(const ros::TimerEvent &event);

    bool GetOdometer_toSensor(Odom_Tgrobot &odom);
    void OdomPub_TimerCallback(const ros::TimerEvent &event);
};

#endif