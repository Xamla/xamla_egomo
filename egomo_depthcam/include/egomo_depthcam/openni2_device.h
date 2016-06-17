#ifndef __OPENNI2_DEVICE__
#define __OPENNI2_DEVICE__

#include <sys/time.h>
#include <OpenNI.h>
#include <boost/function.hpp>
#include <vector>
#include <stdint.h>

#include "egomo_depthcam/openni2_exception.h"


namespace egomo_depthcam
{

typedef boost::function<void(openni::VideoFrameRef &currFrame, struct timeval timestamp)> ImgCallbackFunction;
class OpenNI2FrameListener;

class OpenNI2Device
{
 public:
  enum ImageType {kIRraw, kIRcalib, kIRrawNoSpeckle, kIRcalibNoSpackle, kDepthRaw, kDepthMM, kDepthMMcalib} ;
  enum VideoDeviceType {kDepthVideo, kIRVideo, kRGBVideo, kVideoTypeEnd};

  enum FSMstateGlobal {kClosed, kInactive, kInitialized, kIRprojectorActive, kIRprojectorRequired};
  enum FSMstateIR {kIRdisabled, kIRinactive, kIRconfigured, kIRactive, kIRstreamNoSpeckle, kIRstreamSpeckle};
  enum FSMstateDepth {kDepthDisabled, kDepthInactive, kDepthConfigured, kDepthStreaming};
  // kDepthActive is identical to kIRprojectorActive and therefore not listed as a separate entry

  // configured: device is configured to operate
  // active: device opened and frame listener registered

  enum FSMaction {kOpenDevice, kCloseDevice, kInitDevice, kUninitDevice, kInitIRdevice, kUninitIRdevice, kInitDepthDevice,
                  kUninitDepthDevice,
		  kStartIRdevice, kStopIRdevice, kStartIRstream, kStopIRstream,
		  kStartDepthDevice, kStopDepthDevice,
		  kGrabIRnoSpeckle, kStartIRprojector, kStopIRprojector, kGrabIRSpeckle, kGrabDepthImg,
		  kStartDepthStream, kStopDepthStream};

  enum VideoModeResolutions {kIRstream, kIRstill, kDepthStream, kDepthStill, kRGBstream, kRGBstill, kVideoModeResulutionsEnd};

  OpenNI2Device() throw (OpenNI2Exception);
  ~OpenNI2Device();

  int SwitchState(FSMaction action);
  bool SetResolution(int camStream, int width, int height);

  const openni::VideoFrameRef *GetSingleImage(ImageType type);
  int StartDepthStream();
  int StopDepthStream();

  void RegisterDepthFrameCallback(ImgCallbackFunction callback);
  int SetFrameRate(int frameRate);

  double CalcTimeDiff(timespec start, timespec end);
  openni::Device device;
  static const int maxFrameRate=30;

 private:
  FSMstateGlobal currFSMstate;
  FSMstateIR currIRFSMstate;
  FSMstateDepth currDepthFSMstate;

  int currVideoMode[kVideoModeResulutionsEnd];

  openni::VideoStream videoStreams[kVideoTypeEnd];

  OpenNI2FrameListener *frameListeners[kVideoTypeEnd];

  openni::VideoFrameRef currDepthFrame;
  openni::VideoFrameRef currIRFrame;

  void SetDeviceMode(VideoDeviceType videoType, int mode) throw (OpenNI2Exception);
  void ReadSupportedVideoModes(VideoDeviceType videoType);
  const char* PixelFormatToStr(openni::PixelFormat pixelformat);
};

}

#endif
