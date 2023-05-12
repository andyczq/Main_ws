#ifndef __BASE_CONTROL_NODE_H_
#define __BASE_CONTROL_NODE_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <signal.h>

#define PI      3.1415926f

typedef struct __vel_odom {
    float x, z, euler_yaw;
}Vel_Odom;

typedef struct __pose_odom {
    float x, y;
}Pose_Odom;

typedef struct __odom_data {
    Vel_Odom vel;
    Pose_Odom pose;
}OdomData;

class BaseControl {
    public:
        BaseControl();
        ~BaseControl();
        void SubTwist_Callback(const geometry_msgs::Twist &twist);
        void PubOdom_TimerCallback(const ros::TimerEvent &event);
        uint8_t Check_CRC(const uint8_t *data, uint8_t len);
        bool GetOdometer_toSensor(OdomData &odom);
        bool Write_SerialPort(uint8_t *data);

    private:
        ros::NodeHandle nh;
        serial::Serial serial_port;
        ros::Subscriber twist_sub;
        ros::Publisher odom_pub;
        ros::Timer odom_timer;
        OdomData odometer;
        ros::Time now_time, last_time;
        tf2_ros::TransformBroadcaster tf_broadcaster;
};

#endif