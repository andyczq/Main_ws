#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

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
    odometer = {0};
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    odom_timer = nh.createTimer(ros::Duration(1.0/50), &BaseControl::PubOdom_TimerCallback, this);
    now_time = ros::Time::now();
    last_time = now_time;
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

    if(Write_SerialPort(cmd) == true)
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

    now_time = ros::Time::now();
    float sampling_time = (now_time - last_time).toSec();
    last_time = now_time;
    short transition;

    if ((serial_buf[13] == Check_CRC(serial_buf, 13)) && (serial_buf[3] == 0x12))
    {
        transition = short((serial_buf[4] << 8) | serial_buf[5]);
        odom.vel.x = transition / 1000.0;

        transition = short((serial_buf[8] << 8) | serial_buf[9]);
        odom.vel.euler_yaw = (transition / 100.0) * PI / 180.0; // Angle to radian

        transition = short((serial_buf[10] << 8)|serial_buf[11]);
        odom.vel.z = transition / 1000.0;

        odom.pose.x += (odom.vel.x * cos(odom.vel.euler_yaw)) * sampling_time;
        odom.pose.y += (odom.vel.x * sin(odom.vel.euler_yaw)) * sampling_time;

        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Check data error!");
        ROS_INFO("Receive DATA: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", serial_buf[0],
          serial_buf[1], serial_buf[2], serial_buf[3], serial_buf[4], serial_buf[5], serial_buf[6], serial_buf[7],\
          serial_buf[8], serial_buf[9], serial_buf[10], serial_buf[11], serial_buf[12], serial_buf[13]);
    }
        
    return false;
}

void BaseControl::PubOdom_TimerCallback(const ros::TimerEvent &event)
{
    if(GetOdometer_toSensor(odometer) == true)
    {
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, odometer.vel.euler_yaw);
        geometry_msgs::Quaternion quat;
        quat.x = qtn.getX();
        quat.y = qtn.getY();
        quat.z = qtn.getZ();
        quat.w = qtn.getW();

        nav_msgs::Odometry odom_msgs;
        odom_msgs.header.stamp = now_time;
        odom_msgs.header.frame_id = "odom";
        odom_msgs.child_frame_id = "move_base";
        odom_msgs.pose.pose.position.x = odometer.pose.x;
        odom_msgs.pose.pose.position.y = odometer.pose.y;
        odom_msgs.pose.pose.position.z = 0;
        odom_msgs.pose.pose.orientation = quat;
        odom_msgs.twist.twist.linear.x = odometer.vel.x;
        odom_msgs.twist.twist.linear.y = 0;
        odom_msgs.twist.twist.angular.z = odometer.vel.z;
        odom_pub.publish(odom_msgs);

        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = now_time;
        tfs.header.frame_id = "odom";
        tfs.child_frame_id = "move_base";
        tfs.transform.translation.x = odometer.pose.x;
        tfs.transform.translation.y = odometer.pose.y;
        tfs.transform.translation.z = 0;
        tfs.transform.rotation = quat;
        tf_broadcaster.sendTransform(tfs); 
    }
    else {
        ROS_ERROR_STREAM("Get odometer data error.");
    }  
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
    cmd_data[11] = Check_CRC(cmd_data, 11);

    try {
        serial_port.write(cmd_data, sizeof(cmd_data));
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR("%s \n", e.what());
        ROS_ERROR_STREAM("Unable to write data through serial_port!");
    }
}

uint8_t BaseControl::Check_CRC(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_control");

    BaseControl tgrobot;
    
    ros::spin();
    
    return 0;
}