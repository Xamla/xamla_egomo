#ifndef __OPENNI2_CAM_NODE__
#define __OPENNI2_CAM_NODE__

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

#include <boost/thread/thread.hpp>

#include "egomo_depthcam/LZ4compressedImg.h"
#include "egomo_depthcam/GetNewImage.h"
#include "egomo_depthcam/SetParameters.h"
#include "egomo_depthcam/openni2_device.h"

#include <png.h>

#include <sys/time.h>  // for timing measurements when debugging
#include <stdint.h>


namespace egomo_depthcam
{
class OpenNI2CamNode
{
 public:
  OpenNI2CamNode();
  ~OpenNI2CamNode();

  bool RecordSendImage(egomo_depthcam::GetNewImageRequest& req, egomo_depthcam::GetNewImageResponse& res);
  bool SetParameters(egomo_depthcam::SetParametersRequest& req, egomo_depthcam::SetParametersResponse& res);
  void TryReconnect();

 private:
  char *ReorderBits(const char *inData, int lengthByte, char *out=NULL, int outLenghtBytes=0);
  int CompressLZ4(char *inputBuffer, int inputSize, char *outputBuffer);
  void AdvertiseService();
  bool NewDepthImgCallback(openni::VideoFrameRef &currFrame, struct timeval timestamp);
  void ConnectDepthCb();

  // required to encode the depth image as png via libPNG
  struct pngMemEncode {
    char *buffer;
    size_t size;
  };

  bool CompressPNG(const openni::VideoFrameRef *imgData, std::vector<unsigned char> &imgCompressed);
  static void LocalPNGWriteData(png_structp png_ptr, png_bytep data, png_size_t length);
  static void LocalPNGFlush(png_structp png_ptr);

  long TimeDiffSinceStart(timespec compareTime);

  // private ROS node handle
  ros::NodeHandle nodeHandle;

  ros::ServiceServer serverGetNewImage;
  ros::ServiceServer serverSetParameters;

  ros::Publisher serverDepthStream;
  egomo_depthcam::LZ4compressedImg depthStreamImg;

  unsigned long seqNum;
  unsigned long depthStreamSeqNum;

  boost::mutex connectionMutex;
  int nSubscribers;

  timespec nodeStartTime;  // start time in arbitrary units, use only for time differece calculations
  int callbackCounter;

  OpenNI2Device device;

  char *bitReorderBuffer;
  int bitReorderBufferSize;
  char *lz4ResultBuffer;
  int lz4ResultBufferSize;
};


}
#endif
