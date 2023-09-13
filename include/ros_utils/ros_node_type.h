/**
 * @file ros_utils.h
 * @author  Julian Rendon (julianrendon514@gmail.com)
 * @brief ROS Library to help simplify the implementation of ROS subscriber nodes.
 *
 * The ros_utils library can be used to create multiple subscribers of any data type. The main
 * functionality is to return the latest ROS message unless the message is determined to be
 * stale. It is possible to have messages "synchronized".
 *
 * @version 0.1
 * @date 2022-11-11
 * @copyright Copyright (c) 2022
 */
#ifndef ROS_CLASS_H
#define ROS_CLASS_H

#include <memory>
#include <ros/ros.h>

/**
 * @brief Template Class to create ROS subscriber node.
 *
 * This class provides a convenient way to create a ROS subscriber node that can
 * receive messages of a specified data type. The class has a constructor that
 * takes in the topic name and queue size for the subscriber, and a public
 * method called `get_data` that returns the most recent message received by the
 * subscriber.
 *
 * @tparam DataType The data type of the ROS message to be subscribed to.
 */
template <typename DataType>
class MySubscriber
{
  public:
    /**
     * Constructor which creates a subscription to the specific ROS topic. This can be any type
     * depending on what the parameter DATATYPE is set to.
     *
     * @param[in] topic_name The name of the ROS topic.
     * @param[in] queue_size The queue size of the message queue for the subscriber.
     */
    MySubscriber(std::string topic_name, int queue_size)
    {
        // Create subscriber.
        sub_ = nh_.subscribe(topic_name, queue_size, &MySubscriber::subCallback, this);
    }

    /**
     * Returns the most recent message received by the subscriber. If the elapsed time since the
     * last message was received exceeds the specified timeout, a null pointer is returned.
     *
     * @param[in] time_out The maximum elapsed time in seconds since the last
     * message was received. Default time_out = 0.
     *
     */
    DataType *get_msg(int time_out = 0)
    {
        if (time_out < 0)
        {
            ROS_WARN("Invalid timeout value: %d. (timeout < 0)", time_out);
            return NULL;
        }

        float time_diff = (ros::Time::now() - msg_time_stamp).toSec();   // Time elapsed.
        if (time_out != 0 && time_diff > time_out)
        {
            return NULL;
        }

        return (DataType *) newRosMsg;
    }

    /**
     * Destroy the My Subscriber object. Deallocate void pointer to prevent memory leak once out of
     * scope.
     */
    ~MySubscriber()
    {   // Deallocate DataType pointer.
        free(newRosMsg);
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Time msg_time_stamp;
    void *newRosMsg = NULL;

    /**
     * Callback function for subscriber. Called when new ROS message is published to the specific
     * ROS topic.
     *
     * @param[in] msg Incoming ROS message.
     */
    void subCallback(const typename DataType::ConstPtr &msg)
    {
        // Assign void pointer to incoming ROS message by allocating dynamic memory.
        newRosMsg = new DataType;
        *(DataType *) newRosMsg = *msg;      // Store incoming ROS message.
        msg_time_stamp = ros::Time::now();   // Get timestamp.
    }
};
#endif