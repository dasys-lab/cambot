#include "../include/VStream.h"
#include <ros/ros.h>
#include <OpenNI.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <iostream>
#include <string>
#include <signal.h>

VStream vStream;

void handlerColor(int signum) {
    std::cout << "\nClose Programm" << std::endl;
    vStream.endColorStream();
    openni::OpenNI::shutdown();    
    exit(1);
}

void handlerDepth(int signum) {
    std::cout << "\nClose Programm" << std::endl;
    vStream.endDepthStream();
    openni::OpenNI::shutdown();    
    exit(1);
}

void handlerColorDepth(int signum) {
    std::cout << "\nClose Programm" << std::endl;
    vStream.endColorStream();
    vStream.endDepthStream();
    openni::OpenNI::shutdown();    
    exit(1);
}

int main(int argc, char **argv, const char *node_name) {
    if (argc > 2) {
        ros::init(argc, argv, argv[3]);
    }
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher imagePub = it.advertise("camera/image", 1);

    vStream.testDevice();
    vStream.testDepthStream();
    vStream.testColorStream();
    vStream.getColorVideoStream()->setMirroringEnabled(false);
    vStream.getDepthVideoStream()->setMirroringEnabled(false);
    int i = 0;
    if (argc > 2) {
        i = atoi(argv[2]);
    }
   
    int counter = 0; 
    if(i == 1) {
        vStream.startColorStream();
        while(true) {
            signal(SIGINT, handlerColor);
            if(nh.ok()) {
                imagePub.publish(vStream.colorStream(counter, nh));
                counter++;
            }
        }
    } else if(i == 2) {        
        vStream.startDepthStream();
        while(true) {
            signal(SIGINT, handlerDepth);
            if(nh.ok()) {
                imagePub.publish(vStream.depthStream(counter, nh));
                counter++;
            }  
        }
    } else if(i == 3) {
        vStream.startDepthStream();
        vStream.startColorStream();
        while(true) {
            signal(SIGINT, handlerColorDepth);
            if(nh.ok()) {
                imagePub.publish(vStream.depthColorStream(counter, nh));
                counter++;
            }   
        }
    } else {
        std::cout <<"asusImageViewer: Wrong argument!" << std::endl;
        return 1;
    }
    return 0;
}
