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
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
// #include <string.h>
#include <chassis_msgs/Ultrasonic.h>
// #include <math.h>

#define PI      3.1415926f

// #define FRAME_HEAD  0x5A
// #define FRAME_TAIL  0x00
typedef struct __vel_chassis {
    float x, y, z,rad_yaw;
}Vel_Chassis;

typedef struct __pose_chassis {
    float x, y;
}Pose_Chassis;

typedef struct __odom_chassis {
    Vel_Chassis vel;
    Pose_Chassis pose;
}Odom_Chassis;

class tgrobot_chassis 
{
public:
    tgrobot_chassis();
    ~tgrobot_chassis();
    void tgrobot_controller();

private:
    serial::Serial tgrobot_serial_port;
    std::string serial_port_name, odom_frame_id, base_frame_id;
    int serial_baud_rate;
    ros::Subscriber twist_cmd_vel;
    void CMD_Vel_Callback(const geometry_msgs::Twist &twist_aux);
    void Serial_SendCMD_waitRD(const uint8_t* data);
    uint8_t Check_CRC(uint8_t *data, uint8_t len);

    ros::Publisher battery_pub, odometer_pub, ultrasonic_pub, imu_pub;
    ros::Timer battery_timer, odometer_timer, ultrasonic_timer, imu_timer;

    void BatteryPub_TimerCallback(const ros::TimerEvent &event);

    bool GetOdometer_toSensor(Odom_Chassis &odom);
    void OdomPub_TimerCallback(const ros::TimerEvent &event);

    void UltrasonicPub_TimerCallback(const ros::TimerEvent &event);

    void IMUdataPub_TimerCallback(const ros::TimerEvent &event);
};

#endif