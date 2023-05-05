#include "chassis/chassis_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis");
    turn_on_tgrobot tgrobot_launch;

    while(ros::ok)
    {
        ros::spinOnce();
    }
    return 0;
}

turn_on_tgrobot::turn_on_tgrobot()
{
    ros::NodeHandle nh;
    nh.param<std::string>("Serial_port_name", serial_port_name, "/dev/chassis_serial");
    nh.param<int>("Serial_baud_rate", serial_baud_rate, 115200);

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

    twist_cmd_vel = nh.subscribe("cmd_vel", 100, &turn_on_tgrobot::CMD_Vel_Callback, this);

    battery_pub = nh.advertise<sensor_msgs::BatteryState>("Battery", 10);
    battery_timer = nh.createTimer(ros::Duration(1.0/50), &turn_on_tgrobot::BatteryPub_Timer_Callback, this);

}

turn_on_tgrobot::~turn_on_tgrobot()
{
    try
    {
        tgrobot_serial_port.close();
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to close the serial port");
    }
    
}

void turn_on_tgrobot::CMD_Vel_Callback(const geometry_msgs::Twist &twist_aux)
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
    cmd_data[11] = CMD_Check_CRC(cmd_data, 11);

    try
    {
        tgrobot_serial_port.write(cmd_data, sizeof(cmd_data));
    }
    catch(const serial::IOException& e)
    {
        std::cerr << e.what() << '\n';
        ROS_ERROR_STREAM("Unable to write data through tgrobot_serial_port!");
    }
}

uint8_t turn_on_tgrobot::CMD_Check_CRC(uint8_t *data, uint8_t len)
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

void turn_on_tgrobot::BatteryPub_Timer_Callback(const ros::TimerEvent &event)
{
    uint8_t cmd_data[6] = {0x5A, 0x06, 0x01, 0x07, 0x00, 0xe4};

    try
    {
        tgrobot_serial_port.write(cmd_data, sizeof(cmd_data));
    }
    catch(const serial::IOException& e)
    {
        std::cerr << e.what() << '\n';
        ROS_ERROR_STREAM("Unable to write CMD[Battery].");
    }

    ros::Rate rate(100);
    while(tgrobot_serial_port.waitReadable() == false)
    {
        rate.sleep();
    }

    uint8_t serial_buf[10] = {0}, count = 0;
    if(count = tgrobot_serial_port.available())
    {
        tgrobot_serial_port.read(serial_buf, sizeof(serial_buf));
        // ROS_INFO("[CMD:Battery] Serial port receive %d bytes.", count);
        // ROS_INFO("[CMD Battery]:%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",serial_buf[0],serial_buf[1],serial_buf[2],serial_buf[3],\
        // serial_buf[4],serial_buf[5],serial_buf[6],serial_buf[7],serial_buf[8],serial_buf[9]);
    }

    sensor_msgs::BatteryState battery_msgs;
    short trans_voltage_temp = 0, trans_current_temp = 0;
    if(serial_buf[9] == CMD_Check_CRC(serial_buf, 9))
    {
        if(serial_buf[3] = 0x08)
        {
            trans_voltage_temp |= serial_buf[4]<<8;
            trans_voltage_temp |= serial_buf[5]; 
            trans_current_temp |= serial_buf[6]<<8;
            trans_current_temp |= serial_buf[7];

            battery_msgs.voltage = trans_voltage_temp/1000.0;
            battery_msgs.current = trans_current_temp/1000.0;
            battery_pub.publish(battery_msgs);
        }
        else
        ROS_ERROR_STREAM("Not correct CMD_INFO[Battery].");
    }
    else
    ROS_ERROR_STREAM("CMD[Battery]:CRC check error.");
}