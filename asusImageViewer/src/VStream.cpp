#include "../include/VStream.h"
#include <iostream>

#define CV_COLOR_ENCODINGS     CV_8UC3
#define CV_DEPTH_ENCODINGS     CV_16SC1

#define ROS_IMAGE_COLOR_ENCODINGS   sensor_msgs::image_encodings::RGB8
#define ROS_IMAGE_DEPTH_ENCODINGS   sensor_msgs::image_encodings::TYPE_16UC1

VStream::VStream() {
    this->color = new openni::VideoStream;
    this->depth = new openni::VideoStream;
    this->device = new openni::Device;
    this->deviceURI = openni::ANY_DEVICE;
}
VStream::~VStream() {
    delete device;
    delete depth;
    delete color;
}

void VStream::setDevice(openni::Device* device) {
    this->device = device;
}

openni::Device* VStream::getDevice() {
    return device;
}

void VStream::setColorVideoStream(openni::VideoStream* color) {
    this->color = color;
}
openni::VideoStream* VStream::getColorVideoStream() {
    return color;
}

void VStream::setDepthVideoStream(openni::VideoStream* depth) {
    this->depth = depth;
}
openni::VideoStream* VStream::getDepthVideoStream() {
    return depth;
}

void VStream::setDeviceURI(const char* deviceURI) {
    this->deviceURI = deviceURI;
}
const char* VStream::getDeviceURI() {
    return deviceURI;
}

void VStream::setVideoMode(openni::VideoMode vm) {
    this->vm = vm;
}
openni::VideoMode VStream::getVideoMode() {
    return vm;
}

void VStream::setVideoFrameRef(openni::VideoFrameRef frame) {
    this->frame = frame;
}
openni::VideoFrameRef VStream::getVideoFrameRef() {
    return frame; 
}

void VStream::setCols(int cols) {
    this->cols = cols;
}
int VStream::getCols() {
    return cols;
}

void VStream::setRows(int rows) {
    this->rows = rows;
}
int VStream::getRows() {
    return rows;
}

void VStream::testDevice() {
    openni::Status rc = openni::STATUS_OK;
    rc = openni::OpenNI::initialize();
    std::cout <<"After initialization:\n"<< openni::OpenNI::getExtendedError() << std::endl;
    rc = this->getDevice()->open(this->getDeviceURI());
    if(rc != openni::STATUS_OK) {
        std::cout <<"asusImageViewer: Device open failed: \n"<< openni::OpenNI::getExtendedError() << std::endl;
        openni::OpenNI::shutdown();
    }        
}

void VStream::testDepthStream() {
    openni::Status rc = this->getDepthVideoStream()->create(*(this->getDevice()), openni::SENSOR_DEPTH);
    if(rc == openni::STATUS_OK) {
        rc = this->getDepthVideoStream()->start();
        if(rc != openni::STATUS_OK) {
            std::cout <<"asusImageViewer: Couldnt find depth stream:\n"<< openni::OpenNI::getExtendedError() << std::endl;
            this->getDepthVideoStream()->destroy();
        }   
    } else {
        std::cout <<"asusImageViewer: Couldnt find depth stream:\n"<< openni::OpenNI::getExtendedError() << std::endl;
    }
}

void VStream::testColorStream() {
    openni::Status rc = this->getColorVideoStream()->create(*(this->getDevice()), openni::SENSOR_COLOR);
    if(rc == openni::STATUS_OK) {
        rc = this->getColorVideoStream()->start();
        if(rc != openni::STATUS_OK) {
            std::cout <<"asusImageViewer: Couldnt find depth stream:\n"<< openni::OpenNI::getExtendedError() << std::endl;
            this->getColorVideoStream()->destroy();
        }   
    } else {
        std::cout <<"asusImageViewer: Couldnt find depth stream:\n"<< openni::OpenNI::getExtendedError() << std::endl;
    }
}

sensor_msgs::ImagePtr VStream::colorStream(int counter, ros::NodeHandle nh) {
        this->getColorVideoStream()->readFrame(&frame);
            
        openni::RGB888Pixel *dData = (openni::RGB888Pixel *) frame.getData();
        cv::Mat colorImage(this->getRows(), this->getCols(), CV_COLOR_ENCODINGS, dData);  
            
        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
            
        header.seq = counter;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, ROS_IMAGE_COLOR_ENCODINGS, colorImage);
        
        return img_bridge.toImageMsg();   
}
sensor_msgs::ImagePtr VStream::depthStream(int counter, ros::NodeHandle nh) {
        this->getDepthVideoStream()->readFrame(&frame);
            
        openni::RGB888Pixel *dData = (openni::RGB888Pixel *) frame.getData();
        cv::Mat depthImage(this->getRows(), this->getCols(), CV_DEPTH_ENCODINGS, dData);
            
        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
            
        header.seq = counter;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, ROS_IMAGE_DEPTH_ENCODINGS, depthImage);
        
        return img_bridge.toImageMsg();
}

sensor_msgs::Image VStream::depthColorStream(int counter, ros::NodeHandle nh) {
    sensor_msgs::ImagePtr colorImage_msg = this->colorStream(counter, nh);
    sensor_msgs::ImagePtr depthImage_msg = this->depthStream(counter, nh);
    sensor_msgs::Image colorDepthImage_msg;

    uint8_t colorDepthData [colorImage_msg->height * colorImage_msg->width * 4] = {};

    for (int i = 0; i < colorImage_msg->height * colorImage_msg->width; i++) {
        if (depthImage_msg->data[i * 2] == 0){
            colorDepthData[4 * i + 0] = 0;
            colorDepthData[4 * i + 1] = 0;
            colorDepthData[4 * i + 2] = 0;
            colorDepthData[4 * i + 3] = 0;
        }else {
            colorDepthData[4 * i + 0] = colorImage_msg->data[3 * i + 2]; 
            colorDepthData[4 * i + 1] = colorImage_msg->data[3 * i + 1];
            colorDepthData[4 * i + 2] = colorImage_msg->data[3 * i + 0];
            colorDepthData[4 * i + 3] = depthImage_msg->data[i * 2];
        }
    }

    sensor_msgs::fillImage( colorDepthImage_msg, "8UC4",colorImage_msg->height, colorImage_msg->width,
                            colorImage_msg->width * 4, &colorDepthData);
    
    return colorDepthImage_msg;
}

void VStream::startDepthStream() {
    this->setVideoMode(this->getDepthVideoStream()->getVideoMode());
    this->setCols(this->getVideoMode().getResolutionX());
    this->setRows(this->getVideoMode().getResolutionY());

    this->getDepthVideoStream()->start();
}

void VStream::startColorStream() {
    this->setVideoMode(this->getColorVideoStream()->getVideoMode());
    this->setCols(this->getVideoMode().getResolutionX());
    this->setRows(this->getVideoMode().getResolutionY());

    this->getColorVideoStream()->start();
}

void VStream::endColorStream() {
    this->getDepthVideoStream()->stop();
}

void VStream::endDepthStream() {
    this->getColorVideoStream()->stop();
}
