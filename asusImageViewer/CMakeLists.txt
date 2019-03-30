cmake_minimum_required(VERSION 2.8.3)
project(asusImageViewer)

##Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(cv_bridge_DIR /opt/ros/melodic/include/cv_bridge/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

if(${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse.CDT4")
    set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
    set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
    add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    openni2_camera
)

find_package(OpenCV REQUIRED)
find_package(OpenNI2 REQUIRED)

catkin_package(
    INCLUDE_DIRS include 
    CATKIN_DEPENDS
        roscpp
        sensor_msgs
        cv_bridge
        std_msgs
        image_transport
        openni2_camera
    DEPENDS
        OpenNI2
        OpenCV
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${cv_bridge_DIR}
    ${OPENNI2_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}  
    src/Publisher.cpp
    src/VStream.cpp
)

target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${OPENNI2_LIBRARY}
)
