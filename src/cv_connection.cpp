#include "cv_connection.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include "upload.h"
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
OpenCVConnector::OpenCVConnector(std::string topic_name) : counter(0)	{
    pub = nh.advertise<sensor_msgs::CompressedImage>(topic_name + "/compressed", 1);
}


void OpenCVConnector::WriteToOpenCV(sensor_msgs::ImagePtr * img_msg, int width, int height) {
// void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width, int height) {
//void OpenCVConnector::WriteToOpenCV(cudaArray_t* buffer, int width, int height) {


	// create a cv::Mat from a dwImageNvMedia rgbaImage
/**	cv::Mat mat_img(cv::Size(width, height), CV_8UC3, buffer);

	cv::Mat converted;//=new cv::Mat();

	// cv::cvtColor(mat_img,converted,cv::COLOR_RGBA2RGB);   //=COLOR_BGRA2BGR

	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg; // >> message to be sent
	//	sensor_msgs::ImagePtr img_msg; // >> message to be sent

	std_msgs::Header header; // empty header
	header.seq = counter; // user defined counter
	header.stamp = ros::Time::now(); // time

	//	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGBA8, mat_img);
	//	img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

	//	img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGBA8, mat_img).toImageMsg();

	img_msg.header = header;
	img_msg.height = height;
	img_msg.width = width;
	img_msg.encoding = "rgb8";	
	int step = width * 3 * sizeof(uint8_t); //  image.cols * number_of_channels * sizeof(datatype_used)
	img_msg.step = step;
	size_t size = step * height;
	img_msg.data.resize(size);	
	memcpy(&img_msg.data[0], buffer, size);
	//cudaCopy(&img_msg.data[0], buffer, size);
**/
	pub.publish(*img_msg);
	//pub.publish(img_msg);
	// pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
}


ros::Publisher * OpenCVConnector::getPublisher()
{
	return &pub;
}

void OpenCVConnector::Cuda2Gpumat(unsigned char* src, int width, int height)
{	
	cv::cuda::GpuMat d_test(cv::Size(width, height), CV_8UC3, src);
	cv::cuda::GpuMat d_dst;
	cv::cuda::cvtColor(d_test, d_dst, cv::COLOR_RGB2BGR);
	cv::Mat h_test; // = d_test;
	d_dst.download(h_test);
	//std::cout << h_test << std::endl;
	std::cerr << "in" << std::endl;
	bool isWritten = cv::imwrite("/home/nvidia/Desktop/test.jpg", h_test);  
	std::cerr << isWritten << std::endl;
	return;
}
