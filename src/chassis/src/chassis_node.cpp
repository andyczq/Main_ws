#include "chassis/chassis_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis");
    
    Chassis chassis(ros::NodeHandle(), ros::NodeHandle("~"));
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);
    ros::spin();
    
    return 0;
}

Chassis::Chassis(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) : node(_comm_nh), pnode(_param_nh)
{
    pnode.param<std::string>("serial_port_name", serial_port_name, "/dev/chassis_serial");
    pnode.param<int>("serial_baud_rate", serial_baud_rate, 115200);
    pnode.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    pnode.param<std::string>("base_frame_id", base_frame_id, "base_link");

    pnode.param<bool>("pub_odometer", pub_odometer, true);
    pnode.param<bool>("pub_battery", pub_battery, false);
    pnode.param<bool>("pub_ultrasonic", pub_ultrasonic, false);
    pnode.param<bool>("pub_imu", pub_imu, false);

    try
    {
        serial_port.setPort(serial_port_name);
        serial_port.setBaudrate(serial_baud_rate);
        serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(2000);
        serial_port.setTimeout(serial_timeout);
        serial_port.open();
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("Tgrobot can not open serial port,Please check the serial port cable! "); 
        return;
    }
    ROS_INFO_STREAM("Chassis serial port enabled successfully.");

    twist_cmd_vel = node.subscribe("cmd_vel", 100, &Chassis::CMD_Vel_Callback, this);

    if(pub_odometer)
    {
        odometer_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
        odometer_timer = node.createTimer(ros::Duration(1.0/50), &Chassis::OdomPub_TimerCallback, this);
        ROS_INFO_STREAM("Odometer topic is published.");
    }
    
    if(pub_battery)
    {
        battery_pub = _comm_nh.advertise<sensor_msgs::BatteryState>("battery", 10);
        battery_timer = _comm_nh.createTimer(ros::Duration(1.0/2), &Chassis::BatteryPub_TimerCallback, this);
        ROS_INFO_STREAM("Battery topic is published.");
    }
    
    if(pub_ultrasonic)
    {
        ultrasonic_pub = _comm_nh.advertise<chassis_msgs::Ultrasonic>("sonar", 10);
        ultrasonic_timer = _comm_nh.createTimer(ros::Duration(1.0/10), &Chassis::UltrasonicPub_TimerCallback, this);
        ROS_INFO_STREAM("Ultrasonic topic is published.");
    }

    if(pub_imu)
    {
        imu_pub = _comm_nh.advertise<sensor_msgs::Imu>("imu", 100);
        imu_timer = _comm_nh.createTimer(ros::Duration(1.0/100), &Chassis::IMUdataPub_TimerCallback, this);
        ROS_INFO_STREAM("IMU topic is published.");
    }
}

void Chassis::IMUdataPub_TimerCallback(const ros::TimerEvent &event)
{
    uint8_t cmd_imu[6] = {0x5A, 0x06, 0x01, 0x13, 0x00, 0x33};
    uint8_t serial_buf[38] = {0}, count = 38;

    if(SendCMD_WaitResponse(cmd_imu, serial_buf, count))
    {
        if ((serial_buf[37] == Check_CRC(serial_buf, 37)) && (serial_buf[3] == 0x14))
        {
            sensor_msgs::Imu imu;
            imu.header.stamp = ros::Time::now();
            imu.header.frame_id = odom_frame_id;

            int32_t transition;

            transition = int32_t((serial_buf[4] << 24) | (serial_buf[5] << 16) | (serial_buf[6] << 8) | serial_buf[7]);
            imu.angular_velocity.x = transition / 100000.0;

            transition = int32_t((serial_buf[8] << 24) | (serial_buf[9] << 16) | (serial_buf[10] << 8) | serial_buf[11]);
            imu.angular_velocity.y = transition / 100000.0;

            transition = int32_t((serial_buf[12] << 24) | (serial_buf[13] << 16) | (serial_buf[14] << 8) | serial_buf[15]);
            imu.angular_velocity.z = transition / 100000.0;

            transition = int32_t((serial_buf[16] << 24) | (serial_buf[17] << 16) | (serial_buf[18] << 8) | serial_buf[19]);
            imu.linear_acceleration.x = transition / 100000.0;

            transition = int32_t((serial_buf[20] << 24) | (serial_buf[21] << 16) | (serial_buf[22] << 8) | serial_buf[23]);
            imu.linear_acceleration.y = transition / 100000.0;

            transition = int32_t((serial_buf[24] << 24) | (serial_buf[25] << 16) | (serial_buf[26] << 8) | serial_buf[27]);
            imu.linear_acceleration.z = transition / 100000.0;

            transition = int32_t((serial_buf[28] << 8) | serial_buf[29]);
            imu.orientation.w = transition / 10000.0;

            transition = int32_t((serial_buf[30] << 8) | serial_buf[31]);
            imu.orientation.x = transition / 10000.0;

            transition = int32_t((serial_buf[32] << 8) | serial_buf[33]);
            imu.orientation.y = transition / 10000.0;

            transition = int32_t((serial_buf[34] << 8) | serial_buf[35]);
            imu.orientation.z = transition / 10000.0;

            imu_pub.publish(imu);
        }
        else {
            ROS_ERROR_STREAM("[CMD IMU]:Serial data checked error.");
        }
    }
}

void Chassis::UltrasonicPub_TimerCallback(const ros::TimerEvent &event)
{
    uint8_t cmd_sonic[6] = {0x5A, 0x06, 0x01, 0x19, 0x00, 0xD4};
    uint8_t serial_buf[10] = {0}, count = 10;

    if(SendCMD_WaitResponse(cmd_sonic, serial_buf, count))
    {
        uint8_t sonic_data[4] = {0};
        if((serial_buf[9] == Check_CRC(serial_buf, 9)) && (serial_buf[3] == 0x1A))
        {
            chassis_msgs::Ultrasonic sonar;
            sonar.header.stamp = ros::Time::now();
            sonar.header.frame_id = odom_frame_id;
            sonar.field_of_view = 0.15;
            sonar.min_range = 0.01;
            sonar.max_range = 0.40;

            serial_buf[4] > 40 ? sonar.front_range = INFINITY : sonar.front_range = serial_buf[4] / 100.0;
            serial_buf[5] > 40 ? sonar.right_range = INFINITY : sonar.right_range = serial_buf[5] / 100.0;
            serial_buf[6] > 40 ? sonar.back_range = INFINITY : sonar.back_range = serial_buf[6] / 100.0;
            serial_buf[7] > 40 ? sonar.left_range = INFINITY : sonar.left_range = serial_buf[7] / 100.0;

            ultrasonic_pub.publish(sonar);
        }
        else {
            ROS_ERROR_STREAM("[CMD Sonic]:Serial data checked error.");
        }
    }
}

bool Chassis::GetOdometer_toSensor(Odom_Chassis *odom)
{
    uint8_t cmd_odom[6] = {0x5A, 0x06, 0x01, 0x11, 0x00, 0xA2};
    uint8_t serial_buf[14] = {0}, count = 14;

    if(SendCMD_WaitResponse(cmd_odom, serial_buf, count))
    {
        static ros::Time current_time = ros::Time::now();
        static ros::Time previous_time = current_time;
        short transition = 0;
        
        current_time = ros::Time::now();
        float sampling_time = (current_time - previous_time).toSec();
        previous_time = current_time;

        if((serial_buf[13] == Check_CRC(serial_buf, 13)) && (serial_buf[3] == 0x12))
        {
            transition = short((serial_buf[4] << 8)|serial_buf[5]);
            odom->vel.x = transition / 1000.0;

            transition = short((serial_buf[6] << 8)|serial_buf[7]);
            odom->vel.y = transition / 1000.0;

            transition = short((serial_buf[8] << 8)|serial_buf[9]);
            odom->vel.theta = (transition / 100.0) * PI / 180.0; // Angle to radian

            transition = short((serial_buf[10] << 8)|serial_buf[11]);
            odom->vel.z = transition / 1000.0;

            odom->pose.x += (odom->vel.x * cos(odom->vel.theta) - odom->vel.y * sin(odom->vel.theta)) * sampling_time;
            odom->pose.y += (odom->vel.x * sin(odom->vel.theta) + odom->vel.y * cos(odom->vel.theta)) * sampling_time;
            
            // ROS_INFO("[Odometer] Rad_yam: %.4f  Pose.x: %.4f  Pose.y:%.4f   Twist.x:%.4f   Twist.y:%.4f   Twist.z:%.4f", odom->vel.rad_yaw,odom->pose.x,odom->pose.y,odom->vel.x,odom->vel.y,odom->vel.z);
            return true;
        }
        else {
            ROS_ERROR_STREAM("[CMD Odom]:Serial data checked error.");
        }
    }
    return false;
}

void Chassis::OdomPub_TimerCallback(const ros::TimerEvent &event)
{
    static Odom_Chassis odom_data = {0};

    if(GetOdometer_toSensor(&odom_data) == true)
    {
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, odom_data.vel.theta);
        geometry_msgs::Quaternion quat_odom;
        quat_odom.x = qtn.getX();
        quat_odom.y = qtn.getY();
        quat_odom.z = qtn.getZ();
        quat_odom.w = qtn.getW();

        ros::Time current_time;
        current_time = ros::Time::now();
        
        nav_msgs::Odometry odom_msgs;
        odom_msgs.header.stamp = current_time;
        odom_msgs.header.frame_id = odom_frame_id;
        odom_msgs.child_frame_id = base_frame_id;
        odom_msgs.pose.pose.position.x = odom_data.pose.x;
        odom_msgs.pose.pose.position.y = odom_data.pose.y;
        odom_msgs.pose.pose.position.z = 0;
        odom_msgs.pose.pose.orientation = quat_odom;

        odom_msgs.twist.twist.linear.x = odom_data.vel.x;
        odom_msgs.twist.twist.linear.y = odom_data.vel.y;
        odom_msgs.twist.twist.angular.z = odom_data.vel.z;

        odometer_pub.publish(odom_msgs);
        // ROS_INFO("[Odometer] Odom_Pose: %.4f  %.4f   Odom_Twist:%.4f  %.4f  %.4f",odom_data.pose.x, odom_data.pose.y, odom_data.vel.x, odom_data.vel.y, odom_data.vel.z);

        // Function: show driving track, will produce a large delay and need to be rewritten
        // geometry_msgs::PoseStamped pose_stamped;
        // pose_stamped.header.stamp = current_time;
        // pose_stamped.header.frame_id = odom_frame_id;
        // pose_stamped.pose = odom_msgs.pose.pose;
        
        // static nav_msgs::Path path;
        // path.header = pose_stamped.header;
        // path.poses.push_back(pose_stamped);
        // path_pub.publish(path);

        static tf2_ros::TransformBroadcaster tf_broadcaster;
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = current_time;
        tfs.header.frame_id = odom_frame_id;
        tfs.child_frame_id = base_frame_id;
        tfs.transform.translation.x = odom_data.pose.x;
        tfs.transform.translation.y = odom_data.pose.y;
        tfs.transform.translation.z = 0.0;
        tfs.transform.rotation = quat_odom;
        tf_broadcaster.sendTransform(tfs);
    }
    else {
        ROS_ERROR_STREAM("Get odometer data error.");
    }  
}

void Chassis::CMD_Vel_Callback(const geometry_msgs::Twist &twist_aux)
{
    uint8_t cmd_data[12] = {0};
    short trans_temp = 0;
    cmd_data[0] = 0x5A;
    cmd_data[1] = 0x0C;
    cmd_data[2] = 0x01;
    cmd_data[3] = 0x01;
    
    trans_temp = twist_aux.linear.x*1000;
    cmd_data[4] = trans_temp>>8;
    cmd_data[5] = trans_temp&0xFF;

    trans_temp = twist_aux.linear.y*1000;
    cmd_data[6] = trans_temp>>8;
    cmd_data[7] = trans_temp&0xFF;

    trans_temp = twist_aux.angular.z*1000;
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
        ROS_ERROR_STREAM("Unable to write data through tgrobot_serial_port!");
    }
}

void Chassis::BatteryPub_TimerCallback(const ros::TimerEvent &event)
{
    uint8_t cmd_battery[6] = {0x5A, 0x06, 0x01, 0x07, 0x00, 0xe4};
    uint8_t serial_buf[10] = {0}, count = 10;
    

    if(SendCMD_WaitResponse(cmd_battery, serial_buf, count))
    {
        if((serial_buf[9] == Check_CRC(serial_buf, 9)) && (serial_buf[3] == 0x08))
        {
            short transition;
            sensor_msgs::BatteryState battery;
            battery.header.stamp = ros::Time::now();
            battery.header.frame_id = base_frame_id;

            transition = short((serial_buf[4] << 8) | serial_buf[5]);
            battery.voltage = transition / 1000.0;

            transition = short((serial_buf[6] << 8) | serial_buf[7]);
            battery.current = transition / 1000.0;

            battery_pub.publish(battery);
        }
        else {
            ROS_ERROR_STREAM("[CMD Battery]:Serial data checked error.");
        }
    }
}

bool Chassis::SendCMD_WaitResponse(const uint8_t* w_data, uint8_t *r_data, uint8_t num)
{
    try {
        serial_port.write(w_data, sizeof(w_data));
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR("%s \n", e.what());
        ROS_ERROR_STREAM("Unable to write CMD[Battery].");
        return false;
    }

    while ((serial_port.waitReadable() == false) || (serial_port.available() != num))
    {
        uint8_t times = 0;
        ros::Time::sleepUntil(ros::Time(0.001));
        if (times++ > 10) // 10ms
        {
            ROS_INFO("[CMD Switch]: 0x%02x", w_data[3]);
            ROS_ERROR_STREAM("[Serial CMD]: Wait serial feedback timeout!");
            serial_port.flush();
            return false;
        }
    }

    try
    {
        serial_port.read(r_data, num);
        return true;
    }
    catch(const serial::IOException &e) {
        serial_port.flush();
        ROS_ERROR("%s \n", e.what());
    }
    
    return false;
}

uint8_t Chassis::Check_CRC(const uint8_t *data, uint8_t len)
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

inline void mySigintHandler(int sig)
{
    ROS_INFO("ROS node shutting down.");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

Chassis::~Chassis()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    CMD_Vel_Callback(twist);

    try
    {
        serial_port.close();
        ROS_INFO_STREAM("Serial port closed!");
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR("%s \n", e.what());
        ROS_ERROR_STREAM("Unable to close the serial port.");
    }
}
