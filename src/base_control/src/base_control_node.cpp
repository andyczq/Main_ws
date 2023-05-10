#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_control");
    while(ros::ok());
    
    return 0;
}