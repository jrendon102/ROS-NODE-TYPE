/**
 * @file RecentMessageSubscriber.h
 * @brief This file defines the RecentMessageSubscriber class for subscribing to and retrieving
 *        the most recent message from a specified ROS topic.
 *
 * @details This class template provides a convenient way to subscribe to a ROS topic and
 *          obtain the most recent message received on that topic. It is designed to work with
 *          different message types specified by tempalte parameter.
 *
 * @note This code is released under the MIT License.
 *       Copyright (c) 2023 Julian Rendon (julianrendon514@gmail.com)
 *
 * @version 1.0.1
 * @date 2023-09-12
 * @author Julian Rendon
 */

#ifndef RECENT_MESSAGE_SUBSCRIBER
#define RECENT_MESSAGE_SUBSCRIBER

#include <ros/ros.h>
#include <ros_utils/ros_exception.h>
#include <ros_utils/ros_utils.h>

using namespace RosUtils;

/**
 * @brief A class for subscribing to and retrieving the most recent message of a specified ROS topic.
 *
 * This class allows you to subscribe to a ROS topic and retrieve the most recent message received on that topic.
 * It is templated to work with different message types.
 *
 * @tparam T The message type to subscribe to.
 */
template <typename T>
class RecentMessageSubscriber
{
  private:
    ros::Subscriber subscriber; /**< The ROS subscriber for the specified topic. */
    ros::Time timeStamp;        /**< The timestamp of the most recent received message. */
    std::shared_ptr<T> rosMsg;  /**< A shared pointer to store the most recent received message. */
    std::string topicName;      /**< The name of the ROS topic being subscribed to. */
    int queueSize;              /**< The size of the message queue for the subscriber. */

  public:
    /**
     * @brief Constructor for RecentMessageSubscriber.
     *
     * Initializes the subscriber and sets up the necessary callbacks.
     *
     * @param nodeHandle A pointer to the ROS NodeHandle.
     * @param topicName The name of the ROS topic to subscribe to.
     * @param queueSize The size of the message queue for the subscriber.
     */
    RecentMessageSubscriber(ros::NodeHandle *nodeHandle, std::string topicName, int queueSize)
        : topicName(topicName), queueSize(queueSize), rosMsg(nullptr)
    {
        subscriber = nodeHandle->subscribe(topicName, queueSize, &RecentMessageSubscriber::SubscriberCallBack, this);
    }

    /**
     * @brief Callback function for handling incoming messages.
     *
     * This function is called whenever a new message is received on the subscribed topic.
     * It updates the stored message and timestamp with the most recent data.
     *
     * @param msg The incoming ROS message.
     */
    void SubscriberCallBack(const typename T::ConstPtr &msg)
    {
        if (!rosMsg)
        {
            rosMsg = std::make_shared<T>();
        }
        *rosMsg = *msg;
        timeStamp = ros::Time::now();
    }

    /**
     * @brief Get the most recent received message.
     *
     * Retrieves the most recent received message, if available, and checks if it is within a specified timeout.
     * If the message is not available or if the specified timeout is exceeded, the message is considered stale.
     *
     * @param timeout The maximum allowed time difference (in seconds) between the current time and the message
     * timestamp. A value of 0.0 means no timeout check, while a non-negative value represents the maximum allowable
     * age of the message in seconds.
     *
     * @return A shared pointer to the most recent received message, or nullptr if the message is stale or if an
     * invalid timeout value is provided.
     *
     * @throws RosUtils::RosException::InvalidParameter if an invalid timeout value is provided (timeout < 0.0).
     */
    std::shared_ptr<T> GetRecentMessage(float timeout = 3.0)
    {
        if (timeout < 0.0)
        {
            throw RosException::InvalidParameter();
        }

        float timeDiff = (ros::Time::now() - timeStamp).toSec();
        if (timeout != 0.0 && timeDiff > timeout)
        {
            return nullptr;   // Message is considered stale.
        }
        return rosMsg;
    }
};
#endif
