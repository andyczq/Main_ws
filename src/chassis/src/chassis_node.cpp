#include "chassis/chassis_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis");
    tgrobot_chassis tg;
    tg.tgrobot_controller();
    
    return 0;
}

tgrobot_chassis::tgrobot_chassis()
{
    ros::NodeHandle nh;
    nh.param<std::string>("Serial_port_name", serial_port_name, "/dev/chassis_serial");
    nh.param<int>("Serial_baud_rate", serial_baud_rate, 115200);
    nh.param<std::string>("odom_frame_id", odom_frame_id, "odom_combined");
    nh.param<std::string>("base_frame_id", base_frame_id, "base_footprint");

    try
    {
        tgrobot_serial_port.setPort(serial_port_name);
        tgrobot_serial_port.setBaudrate(serial_baud_rate);
        serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(2000);
        tgrobot_serial_port.setTimeout(serial_timeout);
        tgrobot_serial_port.open();
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("Tgrobot can not open serial port,Please check the serial port cable! "); 
        return;
    }
    ROS_INFO_STREAM("Tgrobot serial port open succeed!");

    twist_cmd_vel = nh.subscribe("cmd_vel", 100, &tgrobot_chassis::CMD_Vel_Callback, this);

    battery_pub = nh.advertise<sensor_msgs::BatteryState>("Battery", 10);
    battery_timer = nh.createTimer(ros::Duration(1.0/2), &tgrobot_chassis::BatteryPub_TimerCallback, this);
    
    odometer_pub = nh.advertise<nav_msgs::Odometry>("Odometer", 50);
    odometer_timer = nh.createTimer(ros::Duration(1.0/50), &tgrobot_chassis::OdomPub_TimerCallback, this);

    ultrasonic_pub = nh.advertise<chassis_msgs::Ultrasonic>("Sonar", 10);
    ultrasonic_timer = nh.createTimer(ros::Duration(1.0/10), &tgrobot_chassis::UltrasonicPub_TimerCallback, this);

    imu_pub = nh.advertise<sensor_msgs::Imu>("IMU", 100);
    imu_timer = nh.createTimer(ros::Duration(1.0/100), &tgrobot_chassis::IMUdataPub_TimerCallback, this);
}

void tgrobot_chassis::tgrobot_controller()
{
    while(ros::ok())
    {
        ros::spinOnce();
    }
}

void tgrobot_chassis::IMUdataPub_TimerCallback(const ros::TimerEvent &event)
{
    uint8_t cmd_imu[6] = {0x5A, 0x06, 0x01, 0x13, 0x00, 0x33};
    Serial_SendCMD_waitRD(cmd_imu);

    uint8_t serial_buf[38] = {0}, count = 0;
    if((count = tgrobot_serial_port.available()) == 10) {
        tgrobot_serial_port.read(serial_buf, sizeof(serial_buf));
    }
    else return;
    
    int32_t transition;
    if((serial_buf[37] == Check_CRC(serial_buf, 37)) && (serial_buf[3] == 0x14))
    {
        sensor_msgs::Imu imu;
        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "Imu";

        transition = int32_t((serial_buf[4]<<24)|(serial_buf[5]<<16)|(serial_buf[6]<<8)|serial_buf[7]);
        imu.angular_velocity.x = transition / 100000.0;

        transition = int32_t((serial_buf[8]<<24)|(serial_buf[9]<<16)|(serial_buf[10]<<8)|serial_buf[11]);
        imu.angular_velocity.y = transition / 100000.0;

        transition = int32_t((serial_buf[12]<<24)|(serial_buf[13]<<16)|(serial_buf[14]<<8)|serial_buf[15]);
        imu.angular_velocity.z = transition / 100000.0;

        transition = int32_t((serial_buf[16]<<24)|(serial_buf[17]<<16)|(serial_buf[18]<<8)|serial_buf[19]);
        imu.linear_acceleration.x = transition / 100000.0;

        transition = int32_t((serial_buf[20]<<24)|(serial_buf[21]<<16)|(serial_buf[22]<<8)|serial_buf[23]);
        imu.linear_acceleration.y = transition / 100000.0;

        transition = int32_t((serial_buf[24]<<24)|(serial_buf[25]<<16)|(serial_buf[26]<<8)|serial_buf[27]);
        imu.linear_acceleration.z = transition / 100000.0;

        transition = int32_t((serial_buf[28]<<8)|serial_buf[29]);
        imu.orientation.w = transition / 10000.0;

        transition = int32_t((serial_buf[30]<<8)|serial_buf[31]);
        imu.orientation.x = transition / 10000.0;

        transition = int32_t((serial_buf[32]<<8)|serial_buf[33]);
        imu.orientation.y = transition / 10000.0;

        transition = int32_t((serial_buf[34]<<8)|serial_buf[35]);
        imu.orientation.z = transition / 10000.0;
        
        imu_pub.publish(imu);
    }  
}

void tgrobot_chassis::UltrasonicPub_TimerCallback(const ros::TimerEvent &event)
{
    uint8_t cmd_sonic[6] = {0x5A, 0x06, 0x01, 0x19, 0x00, 0xD4};
    Serial_SendCMD_waitRD(cmd_sonic);

    uint8_t serial_buf[10] = {0}, count = 0;
    if((count = tgrobot_serial_port.available()) == 10) {
        tgrobot_serial_port.read(serial_buf, sizeof(serial_buf));
    }
    else return;

    uint8_t sonic_data[4] = {0};
    if((serial_buf[9] == Check_CRC(serial_buf, 9)) && (serial_buf[3] == 0x1A))
    {
        sonic_data[0] = serial_buf[4];
        sonic_data[1] = serial_buf[5];
        sonic_data[2] = serial_buf[6];
        sonic_data[3] = serial_buf[7];
    }
    else return;

    chassis_msgs::Ultrasonic sonar;
    sonar.header.stamp = ros::Time::now();
    sonar.header.frame_id = "sonar";
    sonar.field_of_view = 0.15;
    sonar.min_range = 0.01;
    sonar.max_range = 0.40;

    sonic_data[0] > 40 ? sonar.front_range = INFINITY : sonar.front_range = sonic_data[0]/100.0;
    sonic_data[1] > 40 ? sonar.front_range = INFINITY : sonar.front_range = sonic_data[1]/100.0;
    sonic_data[2] > 40 ? sonar.front_range = INFINITY : sonar.front_range = sonic_data[2]/100.0;
    sonic_data[3] > 40 ? sonar.front_range = INFINITY : sonar.front_range = sonic_data[3]/100.0;

    ultrasonic_pub.publish(sonar);

}

bool tgrobot_chassis::GetOdometer_toSensor(Odom_Chassis &odom)
{
    uint8_t cmd_odom[6] = {0x5A, 0x06, 0x01, 0x11, 0x00, 0xA2};

    Serial_SendCMD_waitRD(cmd_odom);

    uint8_t serial_buf[14] = {0}, count = 0;
    if((count = tgrobot_serial_port.available()) == 14)
    {
        tgrobot_serial_port.read(serial_buf, sizeof(serial_buf));
        // ROS_INFO("[CMD:Odometer] Serial port receive %d bytes.", count);
        // ROS_INFO("[CMD:Odometer] %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",serial_buf[0],serial_buf[1],serial_buf[2],serial_buf[3],\
        // serial_buf[4],serial_buf[5],serial_buf[6],serial_buf[7],serial_buf[8],serial_buf[9],serial_buf[10],serial_buf[11],serial_buf[12],serial_buf[13]);
    }
    else 
        return false;

    short transition = 0;
    static ros::Time current_time = ros::Time::now();
    static ros::Time previous_time = current_time;
    
    current_time = ros::Time::now();
    float sampling_time = (current_time - previous_time).toSec();

    if((serial_buf[13] == Check_CRC(serial_buf, 13)) && (serial_buf[3] == 0x12))
    {
        transition = 0;
        transition = short((serial_buf[4] << 8)|serial_buf[5]);
        odom.vel.x = transition / 1000.0;

        transition = 0;
        transition = short((serial_buf[6] << 8)|serial_buf[7]);
        odom.vel.y = transition / 1000.0;

        transition = 0;
        transition = short((serial_buf[8] << 8)|serial_buf[9]);
        odom.vel.rad_yaw = (transition / 100.0) * PI / 180.0; // Angle to radian

        transition = 0;
        transition = short((serial_buf[10] << 8)|serial_buf[11]);
        odom.vel.z = transition / 1000.0;

        odom.pose.x += (odom.vel.x * cos(odom.vel.rad_yaw) - odom.vel.y * sin(odom.vel.rad_yaw)) * sampling_time;
        odom.pose.y += (odom.vel.x * sin(odom.vel.rad_yaw) + odom.vel.y * cos(odom.vel.rad_yaw)) * sampling_time;
        previous_time = current_time;

        return true;
    }
    return false;
}

void tgrobot_chassis::OdomPub_TimerCallback(const ros::TimerEvent &event)
{
    Odom_Chassis odom_data;

    if(GetOdometer_toSensor(odom_data) == true)
    {
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, odom_data.vel.rad_yaw);
        geometry_msgs::Quaternion quat_odom;
        quat_odom.x = qtn.getX();
        quat_odom.y = qtn.getY();
        quat_odom.z = qtn.getZ();
        quat_odom.w = qtn.getW();
        
        nav_msgs::Odometry odom_msgs;
        odom_msgs.header.stamp = ros::Time::now();
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
        ROS_INFO("[Odometer] Odom_Pose: %.4f  %.4f   Odom_Twist:%.4f  %.4f  %.4f",odom_data.pose.x, odom_data.pose.y, odom_data.vel.x, odom_data.vel.y, odom_data.vel.z);

        tf2_ros::TransformBroadcaster tf_broadcaster;
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = ros::Time::now();
        tfs.header.frame_id = base_frame_id;
        tfs.child_frame_id = odom_frame_id;
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

void tgrobot_chassis::CMD_Vel_Callback(const geometry_msgs::Twist &twist_aux)
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

    try
    {
        tgrobot_serial_port.write(cmd_data, sizeof(cmd_data));
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR("%s \n", e.what());
        ROS_ERROR_STREAM("Unable to write data through tgrobot_serial_port!");
    }
}

void tgrobot_chassis::BatteryPub_TimerCallback(const ros::TimerEvent &event)
{
    uint8_t cmd_battery[6] = {0x5A, 0x06, 0x01, 0x07, 0x00, 0xe4};

    Serial_SendCMD_waitRD(cmd_battery);

    uint8_t serial_buf[10] = {0}, count = 0;

    if((count = tgrobot_serial_port.available()) == 10)
    {
        tgrobot_serial_port.read(serial_buf, sizeof(serial_buf));
        // ROS_INFO("[CMD:Battery] Serial port receive %d bytes.", count);
        // ROS_INFO("[CMD Battery]:%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x",serial_buf[0],serial_buf[1],serial_buf[2],serial_buf[3],\
        // serial_buf[4],serial_buf[5],serial_buf[6],serial_buf[7],serial_buf[8],serial_buf[9]);
    }
    else return;

    sensor_msgs::BatteryState battery_msgs;
    short transition;
    if(serial_buf[9] == Check_CRC(serial_buf, 9))
    {
        if(serial_buf[3] == 0x08)
        {
            transition = short((serial_buf[4] << 8)|serial_buf[5]);
            battery_msgs.voltage = transition/1000.0;

            transition = short((serial_buf[6] << 8)|serial_buf[7]);
            battery_msgs.current = transition/1000.0;

            battery_pub.publish(battery_msgs);
        }
        else
        ROS_ERROR_STREAM("Not correct CMD_INFO[Battery].");
    }
    else
    ROS_ERROR_STREAM("CMD[Battery]:CRC check error.");
}

void tgrobot_chassis::Serial_SendCMD_waitRD(const uint8_t* data)
{
    try {
        tgrobot_serial_port.write(data, sizeof(data));
    }
    catch(const serial::IOException& e) {
        ROS_ERROR("%s \n", e.what());
        ROS_ERROR_STREAM("Unable to write CMD[Battery].");
    }

    ros::Rate rate(100);
    while(tgrobot_serial_port.waitReadable() == false) {
        rate.sleep();
    }
}

uint8_t tgrobot_chassis::Check_CRC(uint8_t *data, uint8_t len)
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

tgrobot_chassis::~tgrobot_chassis()
{
    try
    {
        tgrobot_serial_port.close();
        ROS_INFO_STREAM("Tgrobot serial port close succeed!");
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR("%s \n", e.what());
        ROS_ERROR_STREAM("Unable to close the serial port.");
    }
}