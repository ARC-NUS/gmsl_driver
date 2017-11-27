#include "cv_connection.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
	
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// this call is not avail for opencv 3 #include "opencv2/gpu/gpu.hpp"  // --YLC--

OpenCVConnector::OpenCVConnector(std::string topic_name) : it(nh), counter(0)	{
   pub = it.advertise(topic_name, 1);
    

}


void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width, int height) {

    
    // create a cv::Mat from a dwImageNvMedia rgbaImage
    cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);

    cv::Mat converted;//=new cv::Mat();
    
    // --YLC--
    cv::gpu::GpuMat dst, src;
    src.upload(mat_img);
    cv::cuda::cvtColor(src, dst, CV_RGB2HSV, 3);
    dst.download(converted);

    // cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_msg.header = header;
    
    // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, converted);
    // img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

    cudaMemcpy(mat_img, img_msg.data, mat_img.step[0] * mat_img.rows[0] * mat_img.rows[1], cudaMemcpyDeviceToHost);
    img_msg.height = height;
    img_msg.width = width;
    img_msg.encoding = "RGBA8";
    pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

}


