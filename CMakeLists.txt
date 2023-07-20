cmake_minimum_required(VERSION 3.5)
project(artslam_wrapper)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ARTSLAM_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -fopenmp -DBOOST_LOG_DYN_LINK")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(geodesy REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(std_srvs REQUIRED)

#find_package(GTSAM 4 REQUIRED)
#include_directories(${GTSAM_INCLUDE_DIR})

#find_package(Boost REQUIRED COMPONENTS program_options log log_setup)
#include_directories(${Boost_INCLUDE_DIRS})

# find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# message("${PROJECT_NAME} - PCL version ${PCL_VERSION}")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

find_library(LOTS_IO_FOUND NAMES lots_core_io_lib PATHS "${ARTSLAM_LOCATION}/LOTS-CORE/lib" REQUIRED NO_DEFAULT_PATH)
find_library(LOTS_UTILS_FOUND NAMES lots_core_utils_lib PATHS "${ARTSLAM_LOCATION}/LOTS-CORE/lib" REQUIRED NO_DEFAULT_PATH)
include_directories("${ARTSLAM_LOCATION}/LOTS-CORE/src")
include_directories("${ARTSLAM_LOCATION}/LOTS-CORE/include")

find_library(LOTS_LIB_FOUND NAMES lotslam_lib PATHS "${ARTSLAM_LOCATION}/LOTS-SLAM/lib" REQUIRED NO_DEFAULT_PATH)
include_directories("${ARTSLAM_LOCATION}/LOTS-SLAM/src")
include_directories("${ARTSLAM_LOCATION}/LOTS-SLAM/include")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)

add_executable(artslam_wrapper_node src/artslam_wrapper_node.cpp
    src/controller/controller.cpp
#    src/controller/bridge_visualizer.cpp
    src/kernel/skeleton.cpp
    src/kernel/frontend.cpp
    src/kernel/backend.cpp
    src/kernel/loop_detector.cpp
)

install(TARGETS
    artslam_wrapper_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(artslam_wrapper_node
    # ${PCL_LIBRARIES}
#    gtsam
    ${LOTS_IO_FOUND}
    ${LOTS_UTILS_FOUND}
    ${LOTS_LIB_FOUND}
)

ament_target_dependencies(artslam_wrapper_node
    rclcpp
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    tf2_eigen
    nav_msgs
    sensor_msgs
    visualization_msgs
    pcl_msgs
    pcl_conversions
    std_srvs
)

ament_export_libraries(${PROJECT_NAME})
ament_package()
