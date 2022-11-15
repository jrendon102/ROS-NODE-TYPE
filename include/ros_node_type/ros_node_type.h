/**
 * @file ros_node_type.h
 * @author  Julian Rendon (julianrendon514@gmail.com)
 * @brief ROS Library to help simplify the implementation of ROS subscriber nodes.
 *
 * The ros_node_type library can be used to create multiple subscribers of any data type. The main
 * functionality is to return the latest ROS message unless the message is determined to be
 * stale. It is possible to have messages "synchronized".
 *
 * @version 0.1
 * @date 2022-11-11
 * @copyright Copyright (c) 2022
 */
#ifndef ROS_CLASS_H
#define ROS_CLASS_H

#include <chrono>
#include <memory>
#include <ros/ros.h>

/**
 * @brief Class to simplify creation of ROS subscriber nodes.
 *
 * This template class simplifies the implementation of subscriber nodes by created callback
 * functions when instantiated and returning the latest ROS message. If a ROS message is taking
 * long to arrive, or does not arrive at all, a time out can be set to determine if a message is
 * stale.
 *
 * @tparam DataType
 */
template <typename DataType>
class MySubscriber
{
  private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;

    // Time variables used to determine stale messages.
    std::chrono::system_clock::time_point msg_time_stamp;
    std::chrono::system_clock::time_point current_time;

    // Pointer to store ROS msg.
    void *newRosMsg = NULL;

    /**
     * Callback function for subscriber. Called when new ROS message is published to the specific
     * ROS topic. The ROS message value is stored using a void pointer by first type casting the
     * void pointer to the specific data type and then dereferenced.
     *
     * @tparam DataType ROS message data type.
     * @param msg Incoming ROS message.
     */
    void subCallback(const typename DataType::ConstPtr &msg);

  public:
    /**
     * Constructor which creates a subscription to the specific ROS topic. This can be any type
     * depending on what the parameter DATATYPE is set to.
     *
     * @tparam DataType ROS message data type.
     * @param topic_name ROS topic name.
     * @param queue_size Size of out going ROS message.
     */
    MySubscriber(std::string topic_name, int queue_size);

    /**
     * Returns the latest ROS message or if message is stale, returns NULL. The time_out
     * parameter can be set to determine if a message is stale.
     *
     * @param time_out Time that determines if a message is stale. If set to -1, the last message
     * received will continue to be returned until a new message is published.
     * @return DataType* ROS data type.
     */
    DataType *get_msg(int time_out);

    /**
     * Destroy the My Subscriber object. Deallocate void pointer to prevent memory leak once out of
     * scope.
     */
    ~MySubscriber()
    {
        // Deallocate void pointer.
        free(newRosMsg);
    };
};

// Constructor
template <typename DataType>
MySubscriber<DataType>::MySubscriber(std::string topic_name, int queue_size)
{
    _sub = _nh.subscribe(topic_name, queue_size, &MySubscriber::subCallback, this);
}

// Callback function.
template <typename DataType>
void MySubscriber<DataType>::subCallback(const typename DataType::ConstPtr &msg)
{

    // Assign void pointer to incoming ROS message by allocating dynamic memory.
    newRosMsg = new DataType;
    *(DataType *) newRosMsg = *msg;

    msg_time_stamp = std::chrono::system_clock::now();
}

// Return the latest ROS message or, if message is stale (exceeds time_out), returns NULL.
// If time_out = -1, time_out is essentially off and returns the last received message until a  new
// one arrives.
// TODO: Handle integers less than -1.
template <typename DataType>
DataType *MySubscriber<DataType>::get_msg(int time_out)
{
    current_time = std::chrono::system_clock::now();
    auto time_diff =
        std::chrono::duration_cast<std::chrono::seconds>(current_time - msg_time_stamp).count();
    if (time_out != -1 && time_diff > time_out)
    {
        newRosMsg = NULL;
    }
    return (DataType *) newRosMsg;
}
#endif