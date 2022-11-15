/**
 * @file number_pub_node.cpp
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @brief Simple ROS publisher.
 * @version 0.1
 * @date 2022-11-14
 *
 * Publishes to the /number ROS topic at a rate of 1 message per second. Message starts at 0 and
 * increments by 1 every second.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "number_pub_node");
    ros::NodeHandle nh;
    ros::Publisher number_pub = nh.advertise<std_msgs::Int32>("/number", 1);
    ros::Rate loop(1);
    int number_start = 0;
    while (ros::ok())
    {
        std_msgs::Int32 num_msg;
        num_msg.data = number_start;
        number_pub.publish(num_msg);
        number_start += 1;
        loop.sleep();
    }

    return 0;
}