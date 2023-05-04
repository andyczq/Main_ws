#ifndef __CHASSIS_NODE_HPP_
#define __CHASSIS_NODE_HPP_

#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/BatteryState.h"
#include "ros/timer.h"

// #define FRAME_HEAD  0x5A
// #define FRAME_TAIL  0x00

class turn_on_tgrobot 
{
public:
    turn_on_tgrobot();
    ~turn_on_tgrobot();
    serial::Serial tgrobot_serial_port;

private:
    std::string serial_port_name;
    int serial_baud_rate;
    ros::Subscriber twist_cmd_vel;
    void CMD_Vel_Callback(const geometry_msgs::Twist &twist_aux);
    uint8_t CMD_Check_CRC(uint8_t *data, uint8_t len);

    ros::Publisher battery_pub;
    ros::Timer battery_timer;
    void Battery_Timer_Callback(const ros::TimerEvent &event);
};

#endif