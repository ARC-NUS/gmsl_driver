
#ifndef OPEN_CV_CONNECTOR
#define OPEN_CV_CONNECTOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>

//to create image message in main
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class OpenCVConnector {

public:
   OpenCVConnector(std::string topic_name);

   void WriteToOpenCV(unsigned char*, int, int);
   // void WriteToOpenCV(cudaArray_t*, int, int);
   // void WriteToOpenCV(sensor_msgs::ImagePtr, int, int);
   void WriteToOpenCV(sensor_msgs::ImagePtr*, int, int);
   ros::Publisher * getPublisher();
   void Cuda2Gpumat(unsigned char* src, int width, int height);
   ros::NodeHandle nh;
//   image_transport::ImageTransport it;
//   image_transport::Publisher pub;
   ros::Publisher pub;
   std::string topic_name;

   unsigned int counter;
};

#endif

