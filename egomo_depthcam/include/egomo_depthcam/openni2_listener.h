#ifndef __OPENNI2_LISTENER__
#define __OPENNI2_LISTENER__

#include <string>

#include <sys/time.h>
#include <stdint.h>
#include <OpenNI.h>
#include "egomo_depthcam/openni2_device.h"
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <exception>

namespace egomo_depthcam
{

class OpenNI2FrameListener : public openni::VideoStream::NewFrameListener
{
 public:
  OpenNI2FrameListener(std::string name="noname");
  ~OpenNI2FrameListener();

  void onNewFrame(openni::VideoStream& stream);
  void SetCbFunction(ImgCallbackFunction& cb);
  void SetFrameThrotteling(unsigned int val);
  ImgCallbackFunction GetCbFunction() {return callbackFunction;};
  void ActivateCbFunction(bool val) {cbActive=val;};

  void EnableFramePostProcessing(bool val=true) {framePostProcessingEnabled=val;}
  openni::VideoFrameRef GetNextFrame();
  struct timeval GetTimestamp() {return timestamp;};
  std::string GetListenerName() {return listenerName;};

 private:
  std::string listenerName;
  openni::VideoFrameRef currFrame;
  struct timeval timestamp;
  ImgCallbackFunction callbackFunction;
  void ProcessFrame(const openni::VideoFrameRef& frame);
  bool framePostProcessingEnabled;
  bool cbActive;
  unsigned int frameCounter;
  unsigned int frameThrottel;

  bool grabSingleFrame;
  bool nextFrameAvailable;

  mutable boost::mutex frameGrabbingMutex;
  mutable boost::mutex singleCaptureMutex;
  boost::condition_variable condVarFrameGrabbing;
};

class GrabbingTimeoutException
{
 public:
  GrabbingTimeoutException(std::string message) {msg = message;};
  virtual ~GrabbingTimeoutException() {;};
  virtual const char *what() {return msg.c_str();};
 private:
  std::string msg;
};

}

#endif
