#include <iostream>
#include <vector>

#include <OpenNI.h>
#include "egomo_depthcam/openni2_exception.h"
#include "egomo_depthcam/openni2_device.h"
#include "egomo_depthcam/openni2_listener.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <sys/time.h>
#include <unistd.h>


namespace egomo_depthcam
{

OpenNI2Device::OpenNI2Device() throw (OpenNI2Exception) :
  currFSMstate(kClosed),
  currIRFSMstate(kIRdisabled),
  currDepthFSMstate(kDepthDisabled)
{
  for(int i=0; i<kVideoModeResulutionsEnd; i++)
    currVideoMode[i]=-1;

  for(int i=0; i<kVideoTypeEnd; i++)
    frameListeners[i]=NULL;

  if(openni::OpenNI::initialize() != openni::STATUS_OK) {
    THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
  }

  frameListeners[kIRVideo] = new OpenNI2FrameListener("ir");
  frameListeners[kDepthVideo] = new OpenNI2FrameListener("depth");
}


OpenNI2Device::~OpenNI2Device()
{
}


const openni::VideoFrameRef *OpenNI2Device::GetSingleImage(ImageType imgType)
{
  if(currFSMstate == kClosed)
    SwitchState(kOpenDevice);
  if(currFSMstate == kInactive)
    SwitchState(kInitDevice);

  switch(imgType) {
  case kIRraw:
    if(currIRFSMstate == kIRinactive)
      SwitchState(kInitIRdevice);
    if(currIRFSMstate == kIRconfigured)
      SwitchState(kStartIRdevice);

    if(currDepthFSMstate == kDepthInactive)
      SwitchState(kInitDepthDevice);
    if(currFSMstate == kInitialized && currDepthFSMstate == kDepthConfigured)
      SwitchState(kStartIRprojector);

    if(currFSMstate == kInitialized && currDepthFSMstate == kDepthConfigured)
      SwitchState(kStartIRprojector);

    if(currFSMstate == kIRprojectorActive || currDepthFSMstate == kDepthStreaming) {
      try {
        SwitchState(kGrabIRSpeckle);
      }
      catch (GrabbingTimeoutException &err) {
        //std::cout << "========================= Depth frame grabbing error, try again ..." << std::endl;
        SwitchState(kStopIRprojector);  // restart the depth projector with less agressive timing
        usleep(1000*1000);               // 1 sec
        SwitchState(kGrabIRnoSpeckle);
        usleep(500*1000);
        SwitchState(kStartIRprojector);
        usleep(100 * 1000);       // 0.1 sec
        SwitchState(kGrabIRSpeckle); // if frame grabbing fails again, we don't catch the exception again.
        // Failing again indicates a major problem which has to be handled at higher level.
      }
      return &currIRFrame;
    }
    else
      return NULL;
    break;

  case kIRrawNoSpeckle:
    try {
      if(currIRFSMstate == kIRinactive)
        SwitchState(kInitIRdevice);
      if(currIRFSMstate == kIRconfigured)
        SwitchState(kStartIRdevice);

      if(currFSMstate == kIRprojectorActive)
        SwitchState(kStopIRprojector);
      SwitchState(kGrabIRnoSpeckle);
    }
    catch (GrabbingTimeoutException &err) {
      throw;
    }
    return &currIRFrame;
    break;

  case kDepthMM:
    if(currDepthFSMstate == kDepthInactive)
      SwitchState(kInitDepthDevice);
    if(currFSMstate == kInitialized && currDepthFSMstate == kDepthConfigured)
      SwitchState(kStartIRprojector);

    if(currFSMstate == kIRprojectorActive || currDepthFSMstate == kDepthStreaming) {
      try {
        SwitchState(kGrabDepthImg);
      }
      catch (GrabbingTimeoutException &err) {
        //std::cout << "========================= Depth frame grabbing error, try again ..." << std::endl;
        SwitchState(kStopIRprojector);  // restart the depth projector with less agressive timing
        usleep(1000*1000);               // 1 sec
        SwitchState(kGrabIRnoSpeckle);
        usleep(500*1000);
        SwitchState(kStartIRprojector);
        usleep(100 * 1000);       // 0.1 sec
        SwitchState(kGrabDepthImg); // if frame grabbing fails again, we don't catch the exception again.
        // Failing again indicates a major problem which has to be handled at higher level.
      }
      return &currDepthFrame;
    }
    else
      return NULL;
    break;

  default:
    std::cout << __FILE__ << ", " << __LINE__ << " FAIL: invalid ImageType: " << imgType << std::endl;
    break;
  }

  return NULL;
}


int OpenNI2Device::StartDepthStream()
{
  if(currFSMstate == kClosed)
    SwitchState(kOpenDevice);
  if(currFSMstate == kInactive)
    SwitchState(kInitDevice);
  if(currDepthFSMstate == kDepthInactive)
    SwitchState(kInitDepthDevice);
  if(currFSMstate == kInitialized && currDepthFSMstate == kDepthConfigured)
    SwitchState(kStartIRprojector);
  if(currFSMstate == kIRprojectorActive && currDepthFSMstate == kDepthConfigured)
    SwitchState(kStartDepthStream);

  return 0;
}


int OpenNI2Device::StopDepthStream()
{
  SwitchState(kStopDepthStream);
  return 0;
}



int OpenNI2Device::SwitchState(FSMaction action)
{
  int errorCode=0;

  // std::cout << "current state: " << currFSMstate << ", IR=" << currIRFSMstate << ", depth=" << currDepthFSMstate << std::endl;

  switch(action) {
  case kOpenDevice:
    if(currFSMstate == kClosed) {
      if(device.open(openni::ANY_DEVICE) != openni::STATUS_OK)
	THROW_OPENNI_EXCEPTION("Can't open device \n%s\n", openni::OpenNI::getExtendedError());
      currFSMstate=kInactive;
    }
    else
      errorCode=1;
    break;

  case kCloseDevice:
    if(currFSMstate == kInactive) {
      device.close();
      currFSMstate = kClosed;
    }
    break;

  case kInitDevice:
    if(currFSMstate == kInactive) {
      if(device.hasSensor(openni::SENSOR_IR)) {
	currIRFSMstate = kIRinactive;
      }
      if(device.hasSensor(openni::SENSOR_DEPTH)) {
	currDepthFSMstate = kDepthInactive;
      }
      currFSMstate=kInitialized;
    }
    else
      errorCode=1;
    break;

  case kUninitDevice:
    if(currFSMstate == kInitialized) {
      currFSMstate = kInactive; // no further actions required at the moment
    }
    else
      errorCode=1;
    break;

  case kInitIRdevice:
    if(currIRFSMstate == kIRinactive) {
      const openni::Status sensorStatus = videoStreams[kIRVideo].create(device, openni::SENSOR_IR);
      if (sensorStatus != openni::STATUS_OK)
	THROW_OPENNI_EXCEPTION("Couldn't create IR video stream: \n%s\n", openni::OpenNI::getExtendedError());

      ReadSupportedVideoModes(kIRVideo);
      SetDeviceMode(kIRVideo, 4);
      videoStreams[kIRVideo].setMirroringEnabled(false);

      currIRFSMstate = kIRconfigured;
    }
    else
      errorCode=1;
    break;

  case kUninitIRdevice:
    if(currIRFSMstate == kIRconfigured) {
      videoStreams[kIRVideo].destroy();
      currIRFSMstate == kIRinactive;
    }
    else
      errorCode=1;
    break;

  case kInitDepthDevice:
    if(currDepthFSMstate == kDepthInactive) {
      const openni::Status sensorStatus = videoStreams[kDepthVideo].create(device, openni::SENSOR_DEPTH);
      if (sensorStatus != openni::STATUS_OK)
	THROW_OPENNI_EXCEPTION("Couldn't create DEPTH video stream: \n%s\n", openni::OpenNI::getExtendedError());

      ReadSupportedVideoModes(kDepthVideo);
      SetDeviceMode(kDepthVideo, 4);
      videoStreams[kDepthVideo].setMirroringEnabled(false);

      currDepthFSMstate = kDepthConfigured;
      videoStreams[kDepthVideo].addNewFrameListener(frameListeners[kDepthVideo]);
    }
    else
      errorCode=1;
    break;

  case kUninitDepthDevice:
    if(currDepthFSMstate == kDepthConfigured) {
      videoStreams[kDepthVideo].destroy();
      currDepthFSMstate = kDepthInactive;
      // std::cout << __FILE__ << ", " << __LINE__ << "remove depth frame listener" << std::endl;
      videoStreams[kDepthVideo].removeNewFrameListener(frameListeners[kDepthVideo]);
    }
    else
      errorCode=1;
    break;

  case kStartIRdevice:
    if(currIRFSMstate == kIRconfigured) {
      videoStreams[kIRVideo].addNewFrameListener(frameListeners[kIRVideo]);
      videoStreams[kIRVideo].start();
      currIRFSMstate = kIRactive;
    }
    break;

   case kStopIRdevice:
    if(currIRFSMstate == kIRactive) {
      videoStreams[kIRVideo].stop();
      // std::cout << __FILE__ << ", " << __LINE__ << "remove IR frame listener" << std::endl;
      videoStreams[kIRVideo].removeNewFrameListener(frameListeners[kIRVideo]);
      currIRFSMstate = kIRconfigured;
    }
    break;

  case kStartIRstream:
    if(currFSMstate == kIRprojectorActive || currFSMstate == kIRprojectorRequired) {
      if(currIRFSMstate == kIRactive) {
	currIRFSMstate = kIRstreamSpeckle;
	currFSMstate = kIRprojectorRequired;
	frameListeners[kIRVideo]->ActivateCbFunction(true);
      }

      if(currFSMstate == kInitialized) {
	 if(currIRFSMstate == kIRactive) {
	   currIRFSMstate = kIRstreamNoSpeckle;
	   frameListeners[kIRVideo]->ActivateCbFunction(true);
	 }
      }
    }
    else
      errorCode=1;
    break;

  case kStopIRstream:
    if(currIRFSMstate == kIRstreamNoSpeckle || currIRFSMstate == kIRstreamSpeckle) {
      // std::cout << __FILE__ << ", " << __LINE__ << "remove IR listener " << std::endl;
      videoStreams[kIRVideo].removeNewFrameListener(frameListeners[kIRVideo]);
      videoStreams[kIRVideo].stop();
      currIRFSMstate = kIRconfigured;

      if(currDepthFSMstate != kDepthStreaming) {
	if(currFSMstate == kIRprojectorRequired)
	  currFSMstate = kIRprojectorActive;
      }
    }
    else
      errorCode=1;
    break;


  case kGrabIRnoSpeckle:
    if(currFSMstate == kInitialized) {
      if(currIRFSMstate == kIRstreamNoSpeckle) {
	frameListeners[kIRVideo]->ActivateCbFunction(false);
        try {
          currIRFrame=frameListeners[kIRVideo]->GetNextFrame();
        }
        catch  (GrabbingTimeoutException &err) {
          frameListeners[kIRVideo]->ActivateCbFunction(true);
          throw;
        }

	frameListeners[kIRVideo]->ActivateCbFunction(true);
      }
      else if(currIRFSMstate == kIRactive) {
	currIRFrame=frameListeners[kIRVideo]->GetNextFrame();
      }
      else
	errorCode=2;
    }
    else if(currFSMstate == kIRprojectorActive && currIRFSMstate == kIRactive) {
      SwitchState(kStopIRprojector);
      currIRFrame=frameListeners[kIRVideo]->GetNextFrame();
    }
    else
      errorCode=1;
    break;

  case kStartIRprojector:
    if(currFSMstate == kInitialized) {
      if(currIRFSMstate==kIRstreamNoSpeckle) {
	errorCode=2;
	std::cout << "Can't start depth projector with running IR no spackle stream" << std::endl;
      }

      openni::Status status=videoStreams[kDepthVideo].start();
      currFSMstate = kIRprojectorActive;
    }
    else
      errorCode=1;
    break;

  case kStopIRprojector:
    if(currFSMstate == kIRprojectorActive) {
      videoStreams[kDepthVideo].stop();
      currDepthFSMstate = kDepthConfigured;
      currFSMstate = kInitialized;
    }
    else
      errorCode=1;
    break;

  case kGrabIRSpeckle:
    if(currFSMstate == kIRprojectorActive || currFSMstate == kIRprojectorRequired) {
      currIRFrame=frameListeners[kIRVideo]->GetNextFrame();
    }
    else
      errorCode=1;
    break;

  case kGrabDepthImg:
    if(currFSMstate == kIRprojectorActive || currFSMstate == kIRprojectorRequired) {
      if(currDepthFSMstate == kDepthStreaming) {
	frameListeners[kDepthVideo]->ActivateCbFunction(false);
        try {
          currDepthFrame = frameListeners[kDepthVideo]->GetNextFrame();
        }
        catch  (GrabbingTimeoutException &err) {
          frameListeners[kDepthVideo]->ActivateCbFunction(true);
          throw;
        }
	frameListeners[kDepthVideo]->ActivateCbFunction(true);
      }
      else {
        currDepthFrame = frameListeners[kDepthVideo]->GetNextFrame();
      }
    }
    else
      errorCode=1;
    break;

  case kStartDepthStream:
    if(currFSMstate == kIRprojectorActive || currFSMstate == kIRprojectorRequired) {
      currFSMstate = kIRprojectorRequired;
      currDepthFSMstate = kDepthStreaming;
      frameListeners[kDepthVideo]->ActivateCbFunction(true);
    }
    else
      errorCode=1;
    break;

  case kStopDepthStream:
    if(currDepthFSMstate == kDepthStreaming) {
      if(currIRFSMstate != kIRstreamSpeckle)
	currFSMstate = kIRprojectorActive;
      currDepthFSMstate = kDepthConfigured;
      frameListeners[kDepthVideo]->ActivateCbFunction(false);
    }
    else
      errorCode=1;
    break;

  };

  if(errorCode==1) {
    std::cout << "Invalid state transition from " << currFSMstate << " via action " << action << std::endl;
  }
  if(errorCode==2) {
    std::cout << "Invalid IR/depth state transition from ir " << currIRFSMstate
	      << " or depth " << currDepthFSMstate << " via action " << action << std::endl;
  }

  // std::cout << "new state: " << currFSMstate << ", IR=" << currIRFSMstate << ", depth=" << currDepthFSMstate << std::endl;
}


bool OpenNI2Device::SetResolution(int camStream, int width, int height)
{
  switch(camStream) {
  case kIRstream:
  case kIRstill:
    if(width==320 && height==240) {
      currVideoMode[kIRstream]=0;
      currVideoMode[kIRstill]=0;
    }
    else if(width==640 && height==480) {
      currVideoMode[kIRstream]=4;
      currVideoMode[kIRstill]=4;
    }
    else if(width==1280 && height==1024) {
      currVideoMode[kIRstream]=6;
      currVideoMode[kIRstill]=6;
    }
    else {
      currVideoMode[kIRstream]=-1;
      currVideoMode[kIRstill]=-1;
      return -1;
    }
    break;

  case kDepthStream:
  case kDepthStill:
    if(width==160 && height==120) {
      currVideoMode[kDepthStream]=6;
      currVideoMode[kDepthStill]=6;
    }
    else if(width==320 && height==240) {
      currVideoMode[kDepthStream]=0;
      currVideoMode[kDepthStill]=0;
    }
    else if(width==640 && height==480) {
      currVideoMode[kDepthStream]=4;
      currVideoMode[kDepthStill]=4;
    }
    else {
      currVideoMode[kDepthStream]=-1;
      currVideoMode[kDepthStill]=-1;
      return -1;
    }
    break;

  case kRGBstream:
  case kRGBstill:
    currVideoMode[kRGBstream]=-1;
    currVideoMode[kRGBstill]=-1;
    return -1;
    break;
  }

  return 0;
}


void OpenNI2Device::RegisterDepthFrameCallback(ImgCallbackFunction callback)
{
  frameListeners[kDepthVideo]->SetCbFunction(callback);
}


int OpenNI2Device::SetFrameRate(int frameRate)
{
  int scaleFactor=ceil(double(maxFrameRate)/frameRate);

  if(frameListeners[kIRVideo]!=NULL)
    frameListeners[kIRVideo]->SetFrameThrotteling(scaleFactor);
  if(frameListeners[kDepthVideo]!=NULL)
    frameListeners[kDepthVideo]->SetFrameThrotteling(scaleFactor);
  if(frameListeners[kRGBVideo]!=NULL)
    frameListeners[kRGBVideo]->SetFrameThrotteling(scaleFactor);

  return maxFrameRate/scaleFactor;
}


void OpenNI2Device::SetDeviceMode(VideoDeviceType type, int mode) throw (OpenNI2Exception)
{
    const openni::SensorInfo &sensorInfo = videoStreams[type].getSensorInfo();
    const openni::Array< openni::VideoMode > &supportedVideoModes = sensorInfo.getSupportedVideoModes();
    if(mode < supportedVideoModes.getSize()) {
      const openni::Status rc = videoStreams[type].setVideoMode(supportedVideoModes[mode]);
      if (rc != openni::STATUS_OK)
	THROW_OPENNI_EXCEPTION("Couldn't set video mode: \n%s\n", openni::OpenNI::getExtendedError());
      else
	std::cout << "Video mode for " << type << " set to " << mode << std::endl;
    }
    else
      THROW_OPENNI_EXCEPTION("Invalid video mode number: %i", mode);
}


void OpenNI2Device::ReadSupportedVideoModes(VideoDeviceType type)
{
  const openni::SensorInfo &sensorInfo = videoStreams[type].getSensorInfo();
  const openni::Array< openni::VideoMode > &supportedVideoModes = sensorInfo.getSupportedVideoModes();

  std::cout << "supported res, video stream type " << type << std::endl;
  for (int i=0; i<supportedVideoModes.getSize(); i++) {
    std::cout << i << ": " << supportedVideoModes[i].getResolutionX()
              << "x" << supportedVideoModes[i].getResolutionY()
              << ": " << PixelFormatToStr(supportedVideoModes[i].getPixelFormat()) << std::endl;
  }
}


const char* OpenNI2Device::PixelFormatToStr(openni::PixelFormat pixelformat)
{
  switch (pixelformat) {
  case openni::PIXEL_FORMAT_DEPTH_100_UM:
    return "PIXEL_FORMAT_DEPTH_100_UM";
    break;
  case openni::PIXEL_FORMAT_DEPTH_1_MM:
    return "PIXEL_FORMAT_DEPTH_1_MM";
    break;
  case openni::PIXEL_FORMAT_GRAY16:
    return "PIXEL_FORMAT_GRAY16";
    break;
  case openni::PIXEL_FORMAT_GRAY8:
    return "PIXEL_FORMAT_GRAY8";
      break;
  case openni::PIXEL_FORMAT_JPEG:
    return "PIXEL_FORMAT_JPEG";
    break;
  case openni::PIXEL_FORMAT_RGB888:
    return "PIXEL_FORMAT_RGB888";
      break;
  case openni::PIXEL_FORMAT_SHIFT_9_2:
    return "PIXEL_FORMAT_SHIFT_9_2";
    break;
  case openni::PIXEL_FORMAT_SHIFT_9_3:
      return "PIXEL_FORMAT_SHIFT_9_3";
      break;
  case openni::PIXEL_FORMAT_YUV422:
    return "PIXEL_FORMAT_YUV422";
    break;
  default:
    return "unknown";
    break;
  }
}


// Debug function to calculate time differences and return the result converted to milliseconds
double OpenNI2Device::CalcTimeDiff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec-start.tv_nsec)<0) {
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
  }

  double timediff=temp.tv_sec*1000 + double(temp.tv_nsec)/(1e6);
  return timediff;
}


}
