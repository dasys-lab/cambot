#pragma once 

#include <ros/ros.h>
#include <OpenNI.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

class VStream {
    private:
openni::Device* device;
openni::VideoStream* depth;
openni::VideoStream* color; 
const char* deviceURI;

openni::VideoMode vm;
openni::VideoFrameRef frame;
int cols, rows;
    public:
VStream();
~VStream();

void setDevice(openni::Device* device);
openni::Device* getDevice();

void setColorVideoStream(openni::VideoStream* color);
openni::VideoStream* getColorVideoStream();

void setDepthVideoStream(openni::VideoStream* depth);
openni::VideoStream* getDepthVideoStream();

void setDeviceURI(const char* deviceURI);
const char* getDeviceURI();

void setVideoMode(openni::VideoMode vm);
openni::VideoMode getVideoMode();

void setVideoFrameRef(openni::VideoFrameRef frame);
openni::VideoFrameRef getVideoFrameRef();

void setCols(int cols);
int getCols();

void setRows(int rows);
int getRows();

void testDevice();
void testDepthStream();
void testColorStream();

void startColorStream();
void startDepthStream();

void endColorStream();
void endDepthStream();
sensor_msgs::ImagePtr colorStream(int counter, ros::NodeHandle nh);
sensor_msgs::ImagePtr depthStream(int counter, ros::NodeHandle nh);
sensor_msgs::Image depthColorStream(int counter, ros::NodeHandle nh);
};
