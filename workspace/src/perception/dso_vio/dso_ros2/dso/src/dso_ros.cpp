#include <cstdio>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <iostream>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

std::string calib = "./calibration/camera.txt";
std::string vignetteFile = "./calibration/resized_vignette.png";
std::string gammaFile = "./calibration/pcalib.txt";

using namespace dso;

int
encoding_to_mat(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat frame(
      msg->height, msg->width, encoding_to_mat(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step);

  if (msg->encoding == "rgb8") {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
  }
  cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);

  cv::Mat image = frame;

	if(setting_fullResetRequested)
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		delete fullSystem;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = new FullSystem();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
    if(undistorter->photometricUndist != 0)
      fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

	MinimalImageB minImg((int)image.cols, (int)image.rows,(unsigned char*)image.data);
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
  undistImg->timestamp = static_cast<double>(msg->header.stamp.sec);
	fullSystem->addActiveFrame(undistImg, frameID);
	frameID++;
	delete undistImg;

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // std::string topic("image");
  std::string topic("image");

	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 1200;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;

  bool useSampleOutput = false;


	printf("MODE WITH CALIBRATION, but without exposure times!\n");
	setting_photometricCalibration = 2;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;

  undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

  setGlobalCalib(
          (int)undistorter->getSize()[0],
          (int)undistorter->getSize()[1],
          undistorter->getK().cast<float>());


  fullSystem = new FullSystem();
  fullSystem->linearizeOperation=false;


  if(!disableAllDisplay)
    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
         (int)undistorter->getSize()[0],
         (int)undistorter->getSize()[1]));


  if(useSampleOutput)
      fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


  if(undistorter->photometricUndist != 0)
    fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

  auto node = rclcpp::Node::make_shared("dsoimage");
  auto callback = [&node](const sensor_msgs::msg::Image::SharedPtr msg)
  {
    vidCb(msg);
  };

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
      topic, callback, rmw_qos_profile_default);

  rclcpp::spin(node);
  rclcpp::shutdown();
  for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
  {
      ow->join();
      delete ow;
  }

  delete undistorter;
  delete fullSystem;
  return 0;
}
