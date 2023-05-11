#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define PI      3.1415926f

typedef struct __vel_odom {
    float x, euler_yaw;
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
        uint8_t Check_CRC(const uint8_t *data);
        bool GetOdometer_toSensor(OdomData &odom);
        bool SendCMD_WaitResponse(const uint8_t* cmd, uint8_t &data);
        bool Write_SerialPort(uint8_t *data);

    private:
        ros::NodeHandle nh;
        serial::Serial serial_port;
        ros::Subscriber twist_sub;
        ros::Publisher odom_pub;
        ros::Timer odom_timer;
        OdomData odom;
};

BaseControl::BaseControl()
{
    serial_port.setPort("/dev/chassis_serial");
    serial_port.setBaudrate(115200);
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(2000);
    serial_port.setTimeout(serial_timeout);

    try{
        serial_port.open();
    }
    catch(const serial::IOException& e) {
        ROS_ERROR_STREAM("Tgrobot can not open serial port,Please check the serial port cable! "); 
        return;
    }
    ROS_INFO_STREAM("Chassis serial port enabled successfully.");

    twist_sub = nh.subscribe("cmd_vel", 100, &BaseControl::SubTwist_Callback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    odom_timer = nh.createTimer(ros::Duration(1.0/50), &BaseControl::PubOdom_TimerCallback, this);
}

BaseControl::~BaseControl()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    SubTwist_Callback(twist);

    try
    {
        serial_port.close();
        ROS_INFO_STREAM("Tgrobot serial port close succeed!");
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR("%s \n", e.what());
        ROS_ERROR_STREAM("Unable to close the serial port.");
    }
    ros::shutdown();
}

inline bool BaseControl::Write_SerialPort(uint8_t *data)
{
    try {
        serial_port.write(data, sizeof(data));
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR("%s \n", e.what());
        return false;
    }
    return true;
}

bool BaseControl::GetOdometer_toSensor(OdomData &odom)
{
    uint8_t cmd[6] = {0x5A, 0x06, 0x01, 0x11, 0x00, 0xA2};
    uint8_t serial_buf[14] = {0};

    if(Write_SerialPort(cmd))
    {
        while ((serial_port.waitReadable() == false) || (serial_port.available() != 14))
        {
            uint8_t times = 0;
            ros::Time::sleepUntil(ros::Time(0.001));
            if (times++ > 10) // 10ms
            {
                ROS_ERROR_STREAM("Wait serial feedback timeout!");
                serial_port.flush();
                return false;
            }
        }
        try {
            serial_port.read(serial_buf, 14);
        }
        catch (const serial::IOException &e)
        {
            ROS_ERROR("%s \n", e.what());
            return false;
        }
    }

    
    
}

void BaseControl::PubOdom_TimerCallback(const ros::TimerEvent &event)
{
    GetOdometer_toSensor(odom);
    
}

void BaseControl::SubTwist_Callback(const geometry_msgs::Twist &_twist)
{
    uint8_t cmd_data[12] = {0};
    short trans_temp = 0;
    cmd_data[0] = 0x5A;
    cmd_data[1] = 0x0C;
    cmd_data[2] = 0x01;
    cmd_data[3] = 0x01;
    
    trans_temp = _twist.linear.x*1000;
    cmd_data[4] = trans_temp>>8;
    cmd_data[5] = trans_temp&0xFF;

    trans_temp = _twist.linear.y*1000;
    cmd_data[6] = trans_temp>>8;
    cmd_data[7] = trans_temp&0xFF;

    trans_temp = _twist.angular.z*1000;
    cmd_data[8] = trans_temp>>8;
    cmd_data[9] = trans_temp&0xFF;

    cmd_data[10] = 0x00;
    cmd_data[11] = Check_CRC(cmd_data);

    try {
        serial_port.write(cmd_data, sizeof(cmd_data));
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR("%s \n", e.what());
        ROS_ERROR_STREAM("Unable to write data through serial_port!");
    }
}

uint8_t BaseControl::Check_CRC(const uint8_t *data)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < sizeof(data)-1; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc ^= 0x18;
                crc = (crc >> 1)|0x80;
            }
            else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

bool BaseControl::SendCMD_WaitResponse(const uint8_t* cmd, uint8_t &data)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_control");

    BaseControl tgrobot;
    
    ros::spin();
    
    return 0;
}