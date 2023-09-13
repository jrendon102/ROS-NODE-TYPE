# ROS UTILS

## Overview

The ros_utils package which contains a cpp library intended to simplify the implementation of ROS subscribers. The goal was to automate the mundane task of creating callback functions for every ROS topic that needs subscribing to. The MySubscriber class takes care of creating callback functions and returns the last ROS message published. The class also has the ability to determine and discard a message that is stale. This makes "synchronizing" messages possible.
<br>
<br>
<br>
For some additional documentation regarding the package, you can check out this [README.md](./docs/README.md) file to get started.

## Dependencies

- Doxygen: - Used to generate helpful documentation about library. - For some info on how to get started with Doxygen, you can read this [README.md](./docs/README.md) file to get started. - Full documentation can be found at the official website https://doxygen.nl/
  <br>

**\*Note**: You can modify the [CMakeLists.txt](./CMakeLists.txt), located at the top level of this repo, to remove Doxygen.

## Example

The [MySubscriber_node.cpp](./src/MySubscriber_node.cpp) utilizes the library to subscribe to two ROS topics and returns the most recent messages.
<br>

To run the example use the following steps:

1. Build the ROS package either using either catkin build or catkin make command. **_Note_**: If using catkin_make, you must run the command at the top level directory of the ROS workspace.

   ```
   # From any directory inside the ROS workspace.
   catkin build ros_utils

   # From top level directory of ROS workspace.
   catkin_make
   ```

2. Navigate to the top level directory and source setup.bash file.
   ```
   source devel/setup.bash
   ```
3. Run the simple example:
   ```
   roslaunch ros_utils MySubscriber_example.launch
   ```

## Author & Maintaner

Julian Rendon
