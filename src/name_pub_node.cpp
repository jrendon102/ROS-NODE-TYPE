/**
 * @file name_pub_node.cpp
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @brief Simple ROS publisher.
 * @version 1.0
 * @date 2022-11-12
 *
 * Publishes to /name ROS topic at a rate of 1 message per 5 seconds. Names are cycled once all
 * names have been printed out already.
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "name_pub_node");
    ros::NodeHandle nh;
    int counter = 0;

    // Initialize publishers
    ros::Publisher name_pub = nh.advertise<std_msgs::String>("/name", 1);

    // 1 messages every 5 seconds. (Hz)
    ros::Rate loop(.20);

    // Names in alphabetical order.
    std::vector<std::string> names = {"Abby", "Bob", "Clyde", "Daniel", "Edgar"};

    // Publish names every 5 seconds. Once all names are exhausted, start back from the beginning.
    while (ros::ok())
    {
        if (counter > names.size() - 1)
        {
            counter = 0;
        }
        std_msgs::String name_msg;
        name_msg.data = names[counter];
        name_pub.publish(name_msg);
        loop.sleep();
        counter++;
    }
}