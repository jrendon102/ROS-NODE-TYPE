/**
 * @file MySubscriber_node.cpp
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @brief Simple ROS subscriber node that utilizes the ros_utils library.
 * @version 1.0
 * @date 2022-11-11
 *
 * ROS subscriber node that implements MySubscriber class to generate subscriber of any data type.
 *
 * Creates a subscriber to the ROS topic "/number" of data type std_msgs::Int32. A message is
 * received every 5 seconds however, because the timeout is set to 3 seconds, the message will be
 * considered stale once that timeout is reached and NULL will be returned.
 *
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <iostream>
#include <ros/ros.h>
#include <ros_utils/ros_node_type.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// Message is considered stale after time elapsed exceeds timeout. (s)
#define TIMEOUT 3

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MySubscriber_node");
    // Subscriber.
    MySubscriber<std_msgs::Int32> num_sub("/number", 1);

    while (ros::ok())
    {
        // Get most recent message.
        // Once time elapsed exceeds timeout, NULL will be returned.
        auto num_msg = num_sub.get_msg(TIMEOUT);

        if (num_msg == NULL)
        {
            ROS_INFO("Timestamp[%.0f]: [NULL]", ros::Time::now().toSec());
        }
        else
        {
            ROS_INFO("Timestamp:[%.0f], counter:[%d]", ros::Time::now().toSec(), num_msg->data);
        }
        ros::spinOnce();
    }
    return 0;
}