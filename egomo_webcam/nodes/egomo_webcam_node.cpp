#include <ros/ros.h>
#include <ros/master.h>
#include <ros/callback_queue.h>
#include <egomo_webcam/web_cam.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>

#include <sys/time.h>
#include <unistd.h>

#include <boost/thread/thread.hpp>


#include "egomo_webcam/SetV4lParameter.h"
#include "egomo_webcam/GetNewImage.h"
#include "egomo_webcam_node.h"


using namespace egomo_webcam;


WebCamNode::WebCamNode() :
	     reconfigServerHandle("~"),
	     nodeHandle("~"),
	     seqNum(0),
	     nSubscribers(0)
{
  std::string deviceName;
  int image_width_still;
  int image_height_still;
  int image_width_stream;
  int image_height_stream;

  nodeHandle.param("video_device", deviceName, std::string("/dev/video0"));
  nodeHandle.param("image_width_still", image_width_still, 960);
  nodeHandle.param("image_height_still", image_height_still, 720);
  nodeHandle.param("image_width_stream", image_width_stream, 320);
  nodeHandle.param("image_height_stream", image_height_stream, 240);

  camDriver.SetImageFormatStill(ImageData::YUYV, image_width_still, image_height_still);
  camDriver.SetImageFormatStream(ImageData::MJPG, image_width_stream, image_height_stream);

  std::cout << "starting device " << deviceName << " with img res. " << image_width_still << "x" << image_height_still
	    << " and stream res. " << image_width_stream << "x" << image_height_stream << std::endl;

  if(camDriver.StartDevice(deviceName) != 0) {
    std::vector<std::string> errorMsgs=camDriver.GetErrorMessages();
    for (unsigned i=0; i<errorMsgs.size(); i++)
      ROS_ERROR("%s", errorMsgs[i].c_str());
    exit(1);
  }
  AdvertiseService();
}


WebCamNode::~WebCamNode() {
  camDriver.StopDevice();
}


void WebCamNode::AdvertiseService()
{
  std::cout << "Advertising WebCam service ..." << std::endl;

  serverGetNewImage = nodeHandle.advertiseService("get_new_image",
						  &WebCamNode::RecordSendImage, this);
  serverSetCameraFocus = nodeHandle.advertiseService("set_focus", &WebCamNode::SetCameraFocus, this);

  // Halt connection callbacks until all publishers are set up
  boost::lock_guard<boost::mutex> lock(connectionMutex);
  ros::SubscriberStatusCallback subscriberCb = boost::bind(&WebCamNode::ConnectCb, this);

  // calls subscriberCb both for connect and disconnect events
  serverImgStream = nodeHandle.advertise<sensor_msgs::CompressedImage>("stream/compressed", 2, subscriberCb, subscriberCb);
}


void WebCamNode::ConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connectionMutex);
  nSubscribers = serverImgStream.getNumSubscribers(); // update the number of connected clients
  //std::cout << "number of connected clients: " << nSubscribers << std::endl;

  // optional: do something if someone connects or nSubscribers==0 (=noone connected)
}


bool WebCamNode::SetCameraFocus(egomo_webcam::SetCameraFocusRequest &req, egomo_webcam::SetCameraFocusResponse &res)
{
  bool autofocus = (bool)req.autofocus;
  unsigned int focus = (unsigned int)req.focusValue;

  int result=camDriver.SetManualFocus(focus, autofocus);
  if(result<0) {
    std::vector<std::string> errorMsgs=camDriver.GetErrorMessages();
    for (unsigned i=0; i<errorMsgs.size(); i++)
      ROS_WARN("%s", errorMsgs[i].c_str());
    return false;
  }

  res.result = result;
  return true;
}


bool WebCamNode::RecordSendImage(egomo_webcam::GetNewImageRequest& req, egomo_webcam::GetNewImageResponse& res)
{
  int imgType = (int)req.imgType;
  timespec timeStart, timeEnd;

  clock_gettime(CLOCK_MONOTONIC, &timeStart);
  res.img.header.stamp = ros::Time::now();
  const ImageData *imgData=camDriver.GetStillImage();
  clock_gettime(CLOCK_MONOTONIC, &timeEnd);
  // std::cout<< "Overall img capture time: " << camDriver.CalcTimeDiff(timeStart, timeEnd) << " ms" << std::endl;


  clock_gettime(CLOCK_MONOTONIC, &timeStart);
  if(imgData==NULL) {
    std::vector<std::string> errorMsgs=camDriver.GetErrorMessages();
    for (unsigned i=0; i<errorMsgs.size(); i++)
      ROS_WARN("%s", errorMsgs[i].c_str());
    return false;
  }

  res.img.header.seq = seqNum;
  seqNum++;
  res.img.header.frame_id = "";  // TODO: lookup the required value

  res.img.height = imgData->height;
  res.img.width = imgData->width;
  res.img.encoding = "yuyv422";
  res.img.is_bigendian=false;
  res.img.step=imgData->width*2;

  size_t st0 = (imgData->imgSizeInByte);
  res.img.data.resize(st0);
  memcpy(&res.img.data[0], imgData->data, st0);

  clock_gettime(CLOCK_MONOTONIC, &timeEnd);
  // std::cout << "done, time " << camDriver.CalcTimeDiff(timeStart, timeEnd) << " ms" << std::endl;
  return true;
}


bool WebCamNode::UpdateStreamImage()
{
  if(nSubscribers>0) {  // update image only if there are any subscribers
    streamImg.header.stamp = ros::Time::now();

    const ImageData *imgData=camDriver.GetStreamImage();

    if(imgData==NULL) {
      std::vector<std::string> errorMsgs=camDriver.GetErrorMessages();
      for (unsigned i=0; i<errorMsgs.size(); i++)
	ROS_WARN("%s", errorMsgs[i].c_str());
      return false;
    }

    streamImg.header.seq = seqNumStream;
    seqNumStream++;
    streamImg.header.frame_id = "";  // TODO: lookup the required value
    streamImg.format = "jpeg";

    size_t st0 = (imgData->imgSizeInByte);
    streamImg.data.resize(st0);
    memcpy(&streamImg.data[0], imgData->data, st0);

    serverImgStream.publish(streamImg);
  }

  return true;
}


bool WebCamNode::Spin()
{
  ros::Rate loopRate(this->frameRate);
  static const float freqCheckROScore=2; // check connection to master with 2 Hz

  float counter=0;
  while (ros::ok())
    {
      UpdateStreamImage();
      ros::spinOnce();

      if(counter > this->frameRate/freqCheckROScore) {
	counter=0;
	if(ros::master::check() == false) {
	  TryReconnect();
	}
      }
      counter++;

      loopRate.sleep();
    }
  return true;
}


void WebCamNode::TryReconnect()
{
  nodeHandle.shutdown(); // Shutdown all handles created through the NodeHandle.
  std::cout << "Connection to master lost. Trying to reconnect ..." << std::endl;
  do {
    usleep(1*1000*1000); // sleep for one second
  } while(ros::master::check() == false);

  std::cout << "Connection re-established." << std::endl;
  AdvertiseService();
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "webcam");
  egomo_webcam::WebCamNode a;
  a.SetFrameRate(15);
  a.Spin();
  return EXIT_SUCCESS;
}
