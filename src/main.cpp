/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>
#include <thread>
#include <vector>

#include <chrono>
#include <mutex>
#include <condition_variable>
//#include <lodepng.h>


#include <ros/ros.h>


// SAMPLE COMMON
#include <Checks.hpp>
#include <WindowGLFW.hpp>
#include <WindowEGL.hpp>
#include <ProgramArguments.hpp>
#include <ConsoleColor.hpp>
#include <ResourceManager.hpp>


// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// RENDERER
#include <dw/renderer/Renderer.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

// IMAGE
#include <dw/image/FormatConverter.h>
#include <dw/image/ImageStreamer.h>
#include <SampleFramework.hpp>


#include "cv_connection.hpp"
#include "upload.h"
#include "Camera.hpp"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

#include "libgpujpeg/gpujpeg.h"
#include <thread>
#include <chrono>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static volatile bool g_run = true;
static bool gTakeScreenshot = false;
static int gScreenshotCount = 0;

// 1KB should be plenty for data lines from any sensor
// Actual size is returned during runtime
#define MAX_EMBED_DATA_SIZE (1024 * 1024)
NvMediaISCEmbeddedData sensorData;

// Resource Manager
ResourceManager gResources;
uint32_t g_numCameras;

std::mutex cam_res_mutex;
std::mutex steamer_mutex;


//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
void initGL(WindowBase **window);
void initSensors(dwSALHandle_t sal, std::vector<Camera> &cameras);

void runNvMedia_pipeline(WindowBase *window, dwRendererHandle_t renderer, dwContextHandle_t sdk,
		std::vector<Camera> &cameras);
void _threadEncodeNPublish(int camIdx, Camera* camera, gpujpeg_encoder* encoder, 
		ros::Publisher * publisher, dwCameraProperties cameraProperties, 
		dwImageStreamerHandle_t * nvm2CUDA, dwImageFormatConverterHandle_t * yuv2rgb);

void sig_int_handler(int sig);
void sig_handler(int sig);
void userKeyPressCallback(int key);

//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{

	// Program arguments
	ProgramArguments arguments(
			{
			ProgramArguments::Option_t("type-ab", "ar0231-rccb-ss3322"),
			ProgramArguments::Option_t("type-cd", "ar0231-rccb-ss3322"),
			ProgramArguments::Option_t("type-ef", "ar0231-rccb-ss3322"),
			ProgramArguments::Option_t("selector-mask", "0001"),
			ProgramArguments::Option_t("csi-port", "ab"),
			ProgramArguments::Option_t("cross-csi-sync", "0"),
			ProgramArguments::Option_t("offscreen", "0"),
			ProgramArguments::Option_t("write-file", ""),
			ProgramArguments::Option_t("serialize-type", "h264"),
			ProgramArguments::Option_t("serialize-bitrate", "8000000"),
			ProgramArguments::Option_t("serialize-framerate", "30"),
			ProgramArguments::Option_t("slave", "0"),
			ProgramArguments::Option_t("fifo-size", "3"),

			});

	gResources.initializeResources(argc, argv, &arguments, userKeyPressCallback);
	std::vector<Camera> cameras;

	// Set up linux signal handlers
	struct sigaction action;
	memset(&action, 0, sizeof(action));
	action.sa_handler = sig_handler;

	sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
	sigaction(SIGINT, &action, NULL);  // Ctrl-C
	sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
	sigaction(SIGABRT, &action, NULL); // abort() called.
	sigaction(SIGTERM, &action, NULL); // kill command

	//Init
	g_run = true;

	// create HAL and camera
	uint32_t imageWidth;
	uint32_t imageHeight;
	dwImageType cameraImageType;

	initSensors(gResources.getSAL(), cameras);


	for (auto &camera : cameras) {
		if(camera.imageType != DW_IMAGE_NVMEDIA)
		{
			std::cerr << "Error: Expected nvmedia image type, received "
				<< cameraImageType << " instead." << std::endl;
			exit(-1);
		}
	}

	// Allocate buffer for parsed embedded data
	sensorData.top.data    = new uint8_t[MAX_EMBED_DATA_SIZE];
	sensorData.bottom.data = new uint8_t[MAX_EMBED_DATA_SIZE];
	sensorData.top.bufferSize    = MAX_EMBED_DATA_SIZE;
	sensorData.bottom.bufferSize = MAX_EMBED_DATA_SIZE;


	runNvMedia_pipeline(gResources.getWindow(), gResources.getRenderer(), gResources.getSDK(), cameras);

	// release used objects in correct order
	for (auto &camera : cameras)
		dwSAL_releaseSensor(&camera.sensor);

	// todo - render release code has been commented out, since that one results in a stall
	//        of the GMSL (nvmedia) pipeline. The issue is known and will be fixed in the future.
	//dwRenderer_release(&renderer);


	delete[] sensorData.top.data;
	delete[] sensorData.bottom.data;

	return 0;
}


//------------------------------------------------------------------------------
void initGL(WindowBase **window)
{
	bool offscreen = atoi(gArguments.get("offscreen").c_str()) != 0;
	(void)offscreen;

	if(!gResources.getWindow())
		gResources.window = new WindowGLFW(1280, 800);

	(gResources.window)->makeCurrent();
	(gResources.window)->setOnKeypressCallback(userKeyPressCallback);
}


//------------------------------------------------------------------------------
void initSensors(dwSALHandle_t sal, std::vector<Camera> &cameras)
{

	bool recordCamera = !gArguments.get("write-file").empty();
	std::string selector = gArguments.get("selector-mask");

	dwStatus result;

	// identify active ports
	int idx             = 0;
	int cnt[3]          = {0, 0, 0};
	std::string port[3] = {"ab", "cd", "ef"};
	for (size_t i = 0; i < selector.length() && i < 12; i++, idx++) {
		const char s = selector[i];
		if (s == '1') {
			cnt[idx / 4]++;
		}
	}



	// how many cameras selected in a port
	g_numCameras = 0;
	for (size_t p = 0; p < 3; p++) {
		if (cnt[p] > 0) {
			std::string params;

			params += std::string("csi-port=") + port[p];
			params += ",camera-type=" + gArguments.get((std::string("type-") + port[p]).c_str());
			params += ",camera-count=4"; // when using the mask, just ask for a;ll cameras, mask will select properly

			if (selector.size() >= p*4) {
				params += ",camera-mask="+ selector.substr(p*4, std::min(selector.size() - p*4, size_t{4}));
			}

			params += ",slave="  + gArguments.get("slave");
			params += ",cross-csi-sync="  + gArguments.get("cross-csi-sync");
			params += ",fifo-size="  + gArguments.get("fifo-size");
			params += ",output-format=yuv";

			dwSensorHandle_t salSensor = DW_NULL_HANDLE;
			dwSensorParams salParams;
			salParams.parameters = params.c_str();
			salParams.protocol = "camera.gmsl";

			Camera *cam = new Camera(salSensor, salParams, gResources.getSAL(), gResources.getSDK(), gArguments, recordCamera);
			cameras.push_back(*cam);
			(g_numCameras) += cam->numSiblings;
		}    
	}
}



//------------------------------------------------------------------------------


void runNvMedia_pipeline(WindowBase *window, dwRendererHandle_t renderer, dwContextHandle_t sdk, std::vector<Camera> &cameras)
{
	bool recordCamera = !gArguments.get("write-file").empty();

	// init jpeg encoder
	struct gpujpeg_parameters param;
	gpujpeg_set_default_parameters(&param);  // quality:75, restart int:8, interleaved:1

	struct gpujpeg_image_parameters param_image;
	gpujpeg_image_set_default_parameters(&param_image);
	param_image.width = 1920;
	param_image.height = 1208;
	param_image.comp_count = 3;
	// (for now, it must be 3)
	param_image.color_space = GPUJPEG_RGB;
	param_image.sampling_factor = GPUJPEG_4_4_4;
	std::vector<gpujpeg_encoder*> encoders;
	// Start all the cameras 
	for (auto &camera : cameras) {	
		g_run &= camera.start();
	}

	int argc = 0; char** argv = nullptr;
	ros::init(argc, argv, "image_publisher");

	std::vector<OpenCVConnector *> cv_connectors;

	// Create a topic for each camera attached to each CSI port
	// Topic naming scheme is port/neighbor_idx/image
	for (int i = 0; i < cameras.size(); i++) {
		for (int neighbor = 0; neighbor < cameras[i].numSiblings; neighbor++) {
			const std::string topic = std::string("camera/") + std::to_string(i) + std::string("/") + std::to_string(neighbor) + std::string("/image"); 
			cv_connectors.push_back(new OpenCVConnector(topic));
			std::cout << "num of camera: " << neighbor << std::endl;
			// init jpeg encoder
			encoders.push_back(gpujpeg_encoder_create(&param,
						&param_image));
			if ( encoders.back() == NULL )
				std::cerr << "cannot create encoder\n";

		}
	}

	// ---- init streamer to send to cuda ---- //
	dwImageStreamerHandle_t nvm2CUDA = DW_NULL_HANDLE;

	dwImageProperties cameraImageProperties;
	dwStatus status = dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, cameras[0].sensor); // assuming the properties of both cameras are the same

	if(status != DW_SUCCESS)
	{
		std::cerr << "error in setting image properties\n";
	}

	else
	{
		std::cerr << "got image properties from camera\n";

		status = dwImageStreamer_initialize(&nvm2CUDA, &cameraImageProperties, DW_IMAGE_CUDA, sdk);
		if (status != DW_SUCCESS) 
		{
			std::cerr << "\n ERROR Initialising stream: "  << dwGetStatusName(status) << std::endl;
		}
		else
		{
			std::cerr << "   stream initialised\n" ;

		}

		cameraImageProperties.type = DW_IMAGE_CUDA;
		cameraImageProperties.pxlFormat = DW_IMAGE_YUV420;
		dwImageFormatConverterHandle_t yuv2rgb = DW_NULL_HANDLE;
		dwImageProperties displayImageProperties = cameraImageProperties;
		displayImageProperties.pxlFormat = DW_IMAGE_RGB;
		displayImageProperties.planeCount = 1;
		status = dwImageFormatConverter_initialize(&yuv2rgb, displayImageProperties.type, sdk);
		if (status != DW_SUCCESS) 
		{
			std::cerr << "\n ERROR Initialising converter: "  << dwGetStatusName(status) << std::endl;
		}


		// Message msg;
		while (g_run && ros::ok()) {
			int numOfCamPassed = 0;

			for (int i = 0; i < cameras.size(); i++) {
				Camera camera = cameras[i];
				std::vector<std::thread> t_vector;

				//Get Camera properties
				dwCameraProperties cameraProperties;
				status = dwSensorCamera_getSensorProperties(&cameraProperties, camera.sensor);
				if(status != DW_SUCCESS)
				{
					std::cerr << " cannot get sensor properties: " << dwGetStatusName(status) << std::endl;
				}

				for (int camIdx = 0; camIdx < camera.numSiblings; camIdx++){
					// call thread and move on to next sibling
					t_vector.emplace_back(&_threadEncodeNPublish, camIdx, &camera, encoders[numOfCamPassed], 
							cv_connectors[numOfCamPassed]->getPublisher(), cameraProperties, &nvm2CUDA, &yuv2rgb);
					numOfCamPassed++;
				}

				//destroy threads
				for (std::thread & t : t_vector) {
					t.join();
				}
				t_vector.clear();

				/**if (window)
				  {
				  std::cout << "window!\n";
				  window->swapBuffers();
				  } **/
			}

		}
		dwImageFormatConverter_release(&yuv2rgb);
	}

	// Clean up and release camera assets
	for (auto camera : cameras) {
		camera.stop_camera();
	}
	while(!encoders.empty())
	{
		gpujpeg_encoder_destroy(encoders.back());
		encoders.pop_back();
	}
	dwImageStreamer_release(&nvm2CUDA);
}

////------------------------------------------------------------------------------
void sig_handler(int sig)
{
	(void)sig;

	g_run = false;
}

////------------------------------------------------------------------------------
void userKeyPressCallback(int key)
{
	// stop application
	if (key == GLFW_KEY_ESCAPE)
		gRun = false;
}


////-------------------------------------------------------------------------------
void _threadEncodeNPublish(int camIdx, Camera* camera, gpujpeg_encoder* encoder, 
		ros::Publisher * publisher, dwCameraProperties cameraProperties, dwImageStreamerHandle_t * nvm2CUDA,
		dwImageFormatConverterHandle_t * yuv2rgb)
{
	auto start = std::chrono::high_resolution_clock::now();

	bool isOk = true;
	dwCameraFrameHandle_t frameHandle;
	dwImageNvMedia *frame = nullptr;


	//TODO lock camera, sensorData   
	cam_res_mutex.lock(); // ---------------------------------------------------------------------locked
	auto locked = std::chrono::high_resolution_clock::now();
	// std::cout << "entered mutex on" << camIdx << " lock\n";
	if(!camera->sensor)
	{
		std::cerr << "camera has no sensor\n";
	}

	dwStatus status = dwSensorCamera_readFrame(&frameHandle, camIdx, 1000000, camera->sensor);
	if (status != DW_SUCCESS) {
		std::cout << "\n ERROR readFrame: " << dwGetStatusName(status) << std::endl;
		// continue;
		isOk = false;
	}
	else
	{

		if( cameraProperties.outputTypes & DW_CAMERA_PROCESSED_IMAGE) {
			status = dwSensorCamera_getImageNvMedia(&frame, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
			if( status != DW_SUCCESS ) {
				std::cout << "\n ERROR getImageNvMedia " << dwGetStatusName(status) << std::endl;
			}
		}

		// get embedded lines
		if( cameraProperties.outputTypes & DW_CAMERA_DATALINES) {
			const dwImageDataLines* dataLines = nullptr;
			status = dwSensorCamera_getDataLines(&dataLines, frameHandle);
			// parse the data
			if( status == DW_SUCCESS ) {
				status = dwSensorCamera_parseDataNvMedia(&sensorData, dataLines, camera->sensor);
				if( status == DW_SUCCESS ) {
					std::cout << "Exposure Time (s): " << sensorData.exposureControl.exposureTime << "\r";// std::endl;
				} else {
					std::cout << "Could not parse embedded data: " << dwGetStatusName(status) << "\r"; //std::endl;
				}
			} else {
				std::cout << "Error getting datalines: " << dwGetStatusName(status) << "\r"; //std::endl;
			}
		}

	}
	// Convert from YUV to RGBA
	// bool isOk = false;
	// dwImageNvMedia *rgbaImage;
	// if (frame && camera->rgbaImagePool.size() > 0) {
	// 	rgbaImage = camera->rgbaImagePool.back();
	// 	camera->rgbaImagePool.pop_back();
	// 	isOk= true;
	// }
	cam_res_mutex.unlock(); //--------------------------------------------------------------------unlocked

	auto unlocked = std::chrono::high_resolution_clock::now();

	// std::cout << "exited mutex on " << camIdx << ": unlocked\n";
	dwImageNvMedia *rgbaImage;

	if(isOk)
	{
		rgbaImage = frame;
		steamer_mutex.lock(); //------------------------------------------------------------------lock
		status = dwImageStreamer_postNvMedia(rgbaImage, *nvm2CUDA);
		if (status != DW_SUCCESS) 
		{
			std::cerr << "\n ERROR postNvMedia: " << dwGetStatusName(status) << std::endl;
		} 
		else 
		{
			dwImageCUDA * d_frame = nullptr;
			status = dwImageStreamer_receiveCUDA(&d_frame, 10000, *nvm2CUDA);

			if (status == DW_SUCCESS && d_frame) 
			{
				// ---- convert yuv420 to rgb ---- //
				dwImageCUDA d_rgb;
				void *dptr   = nullptr;
				size_t pitch;
				cudaMallocPitch(&dptr, &pitch, d_frame->prop.width * 3, d_frame->prop.height);
				pitch = d_frame->prop.width * 3; // 5760
				status = dwImageCUDA_setFromPitch(&d_rgb, dptr, d_frame->prop.width, d_frame->prop.height, pitch, DW_IMAGE_RGB);
				d_rgb.prop.pxlType = DW_TYPE_UINT8;
				if (status != DW_SUCCESS) 
				{
					std::cerr << "error creating dw cuda img from pitch: " << dwGetStatusName(status) << std::endl;
				} 

				status = dwImageFormatConverter_copyConvertCUDA(&d_rgb, d_frame, *yuv2rgb, 0);
				if (status != DW_SUCCESS) 
				{
					std::cerr << "error converting cuda format: " << dwGetStatusName(status) << std::endl;
				} 
				// TODO record post cv frame for logging here 
				int height =  d_frame->prop.height;
				int width = d_frame->prop.width;

				status = dwImageStreamer_returnReceivedCUDA(d_frame, *nvm2CUDA);
				if(status != DW_SUCCESS)
				{
					std::cerr << "ERROR cannot return CUDA: " <<  dwGetStatusName(status) << std::endl;
				}
			//	steamer_mutex.unlock(); //---------------------------------------------------------------unlock

				int numChannels = 3;
				// uint8_t * h_rgb;
				// h_rgb = (uint8_t*) malloc(width*height*3);
				auto copy_start = std::chrono::high_resolution_clock::now();
				// cudaCopy(h_rgb,(uint8_t*) d_rgb.dptr[0], width*height*numChannels*sizeof(uint8_t));
				auto copy_end = std::chrono::high_resolution_clock::now();
				struct gpujpeg_encoder_input encoder_input;
				auto encode_start = std::chrono::high_resolution_clock::now();
				// gpujpeg_encoder_input_set_image(&encoder_input, (uint8_t*) h_rgb);
				gpujpeg_encoder_input_set_image(&encoder_input, (uint8_t*) d_rgb.dptr[0]);
				uint8_t* image_compressed = NULL;
				int image_compressed_size = 0;
				if ( gpujpeg_encoder_encode(encoder, &encoder_input, &image_compressed,
							&image_compressed_size, true) != 0 )
					std::cerr << "cannot encode image\n";
				auto encode_end = std::chrono::high_resolution_clock::now();
				sensor_msgs::CompressedImage c_img_msg;
				std_msgs::Header header; // empty header
				header.stamp = ros::Time::now(); // time
				c_img_msg.header = header;
				c_img_msg.format = "jpeg";
				c_img_msg.data.resize(image_compressed_size);
				auto cpy_to_pub_start = std::chrono::high_resolution_clock::now();
				memcpy(&c_img_msg.data[0], image_compressed, image_compressed_size);                                
				auto cpy_to_pub_end = std::chrono::high_resolution_clock::now();
				// publish to ros
				publisher->publish(c_img_msg);
				auto pub_end = std::chrono::high_resolution_clock::now();

				cudaFree(d_rgb.dptr[0]);
				// free(h_rgb);

				std::cout << "thread cam " << camIdx << " took "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(pub_end - cpy_to_pub_end).count()
					<< " milliseconds to copy to publish ros msg\n";

				std::cout << "thread cam " << camIdx << " took "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(cpy_to_pub_end - cpy_to_pub_start).count()
					<< " milliseconds to copy to ros msg\n";

				std::cout << "thread cam " << camIdx << " took "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(copy_end-copy_start).count()
					<< " milliseconds to copy to CPU memory\n";

				std::cout << "thread cam " << camIdx << " took "
					<< std::chrono::duration_cast<std::chrono::milliseconds>(encode_end-encode_start).count()
					<< " milliseconds to encode\n";
			}
			else
			{
				std::cerr << "ERROR cannot receive on CUDA: " <<  dwGetStatusName(status) << std::endl;
			//	steamer_mutex.unlock();
			}
		}

		// any image returned back, we put back into the pool
		dwImageNvMedia *retimg = nullptr;
		auto stream_start = std::chrono::high_resolution_clock::now();
//		steamer_mutex.lock();
		dwImageStreamer_waitPostedNvMedia(&retimg, 33000, *nvm2CUDA);
		steamer_mutex.unlock();
		auto stream_end = std::chrono::high_resolution_clock::now();
		std::cout << "thread cam " << camIdx << " took "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(stream_end-stream_start).count()
			<< " milliseconds to lock and unlock for retimg\n";

		// if (retimg)
		// {
		// 	cam_res_mutex.lock(); 
		// 	std::cout << "entered second  " << camIdx << " mutex: locked\n";
		// 	camera->rgbaImagePool.push_back(retimg);
		// 	cam_res_mutex.unlock(); 
		// 	std::cout << "exited second " << camIdx << " mutex: unlocked\n";
		// }

		dwSensorCamera_returnFrame(&frameHandle);
	}
	else
		std::cerr << "No image in rgbaImagePool!\n";

	auto end = std::chrono::high_resolution_clock::now();

	std::cout << "thread cam " << camIdx << " took "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()
		<< " milliseconds in total\n";

	std::cout << "thread cam " << camIdx << " took "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(locked-start).count()
		<< " milliseconds to lock\n";

	std::cout << "thread cam " << camIdx << " took "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(unlocked-locked).count()
		<< " milliseconds to unlock\n";
}

