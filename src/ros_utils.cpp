/**
 * @file RosUtils.cpp
 * @brief Provides utility functions for working with ROS (Robot Operating System).
 *
 * @note This code is released under the MIT License.
 *       (c) 2023 Julian Rendon (julianrendon514@gmail.com)
 *
 * @version 1.0.0
 * @date 2023-09-12
 * @author Julian Rendon
 */
#include <ros_utils/ros_utils.h>

void RosUtils::ConfigureLogLevel(int argc, char **argv)
{
    ros::console::Level logLevel = ros::console::levels::Info;
    if (argc > 1)
    {
        std::string logLevelArg = argv[1];
        if (logLevelArg == "--debg")
        {
            logLevel = ros::console::levels::Debug;
        }
    }

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, logLevel);
}