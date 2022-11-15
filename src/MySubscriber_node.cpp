/**
 * @file MySubscriber_node.cpp
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @brief Simple ROS subscriber node that utilizes the ros_node_pkg library.
 * @version 0.1
 * @date 2022-11-11
 *
 * ROS subscriber node that implements MySubscriber class to generate subscriber of any data type.
 * Creates two subscribers and receives their most recent ROS message.
 *
 * @copyright
 *
 */

#include <iostream>
#include <ros/ros.h>
#include <ros_node_type/ros_node_type.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MySubscriber_node");

    // Subscribe to topics.
    MySubscriber<std_msgs::Int32> num_sub("/number", 1);
    MySubscriber<std_msgs::String> str_sub("/name", 1);

    while (ros::ok())
    {
        // Timeout set to -1 so that we keep the last message.
        auto num_msg = num_sub.get_msg(-1);
        auto str_msg = str_sub.get_msg(-1);

        if (str_msg == NULL && num_msg == NULL)
        {
            ROS_INFO("Incoming messages: [NULL] [NULL]");
        }
        else if (num_msg != NULL && str_msg == NULL)
        {
            ROS_INFO("Incoming messages: [NULL], [%d].", num_msg->data);
        }
        else if (str_msg != NULL && num_msg == NULL)
        {
            ROS_INFO("Incoming messages: [%s], [NULL].", str_msg->data.c_str());
        }
        else
        {
            ROS_INFO("Incoming messages: [%s], [%d]", str_msg->data.c_str(), num_msg->data);
        }
        ros::spinOnce();
    }
    return 0;
}