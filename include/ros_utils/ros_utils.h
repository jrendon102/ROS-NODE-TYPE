/**
 * @file RosUtils.h
 * @brief Provides utility functions for working with ROS (Robot Operating System).
 *
 * This header file defines a set of utility functions designed to facilitate ROS
 * (Robot Operating System) development.
 *
 * @note This code is released under the MIT License.
 *       (c) 2023 Julian Rendon (julianrendon514@gmail.com)
 *
 * @version 1.0.0
 * @date 2023-09-12
 * @author Julian Rendon
 */

#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include <ros/ros.h>

namespace RosUtils
{

/**
 * @brief Template function to retrieve a parameter's() value from the ROS parameter server.
 *
 * This function attempts to retrieve a parameter's() value from the ROS parameter server.
 * If the parameter exists, its value is returned; otherwise, appropriate exceptions are thrown.
 *
 * @tparam T The data type of the parameter.
 * @param paramName The name of the ROS parameter to retrieve.
 * @return The value of the specified ROS parameter.
 *
 * @throws `ros::InvalidParameterException` - if the parameter exists but is of an incompatible type
 *         or if the parameter does not exist.
 */
template <typename T>
T GetParam(const std::string &paramName)
{
    ROS_DEBUG("%s()::Fetching camera parameter:[%s]", __func__, paramName.c_str());
    T value;
    if (!ros::param::get(paramName, value))
    {
        if (ros::param::has(paramName))
        {
            ROS_ERROR("%s()::Failed to retrieve parameter:[%s]", __func__, paramName.c_str());
            throw ros::InvalidParameterException("invalid parameter type");
        }
        ROS_ERROR("%s()::Failed to retrieve parameter:[%s]", __func__, paramName.c_str());
        throw ros::InvalidParameterException("parameter not found");
    }
    ROS_DEBUG("%s()::Success.", __func__);
    return value;
}

/**
 * @brief Configures the ROS logging level based on command line arguments.
 *
 * This function configures the ROS logging level based on command line arguments.
 * It allows you to specify the verbosity of ROS log messages when starting a ROS node.
 *
 * @param arg The argument count.
 * @param argv The argument vector.
 */
void ConfigureLogLevel(int arg, char **argv, ros::console::Level &logLevel);

}   // namespace RosUtils

#endif
