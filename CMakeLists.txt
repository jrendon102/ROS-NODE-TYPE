cmake_minimum_required(VERSION 3.0.2)
project(ros_node_type)

# # Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# ######################
# # Setting up Doxygen##
# ######################
# Generate Doxygen documentation. Instructions listed here https://vicrucann.github.io/tutorials/quick-cmake-doxygen/
option(BUILD_DOC "Build documentation" ON)

find_package(Doxygen)

if(DOXYGEN_FOUND)
  # set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/docs/Doxyfile)

  # request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target(doc_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)
else(DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif(DOXYGEN_FOUND)

# # Add this to debug ros node in VScode.
set(CMAKE_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_node_pkg
  CATKIN_DEPENDS roscpp std_msgs
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/ros_node_type.cpp
)

# # Declare a C++ executable
add_executable(name_pub src/name_pub_node.cpp)
add_executable(number_pub src/number_pub_node.cpp)
add_executable(MySubscriber src/MySubscriber_node.cpp)

# # Specify libraries to link a library or executable target against
target_link_libraries(name_pub ${catkin_LIBRARIES})
target_link_libraries(number_pub ${catkin_LIBRARIES})
target_link_libraries(MySubscriber ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# # Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
