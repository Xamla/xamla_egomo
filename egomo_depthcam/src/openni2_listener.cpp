#include <iostream>
#include <string>

#include <sys/time.h>
#include <OpenNI.h>

#include "egomo_depthcam/openni2_exception.h"
#include "egomo_depthcam/openni2_listener.h"
#include "egomo_depthcam/openni2_device.h"

#include <boost/thread/thread.hpp>


namespace egomo_depthcam
{

OpenNI2FrameListener::OpenNI2FrameListener(std::string name) :
  listenerName(name),
  framePostProcessingEnabled(true),
  cbActive(false),
  frameCounter(0),
  frameThrottel(2)
{
  ;
}

OpenNI2FrameListener::~OpenNI2FrameListener()
{
  ;
}

void OpenNI2FrameListener::onNewFrame(openni::VideoStream& stream)
{
  boost::mutex::scoped_lock lock(frameGrabbingMutex);
  gettimeofday(&timestamp, NULL);

  frameCounter++;
  if( (cbActive && frameCounter>=frameThrottel) || grabSingleFrame) {
    stream.readFrame(&currFrame);
    if(framePostProcessingEnabled)
      ProcessFrame(currFrame);

    if(cbActive) {
      frameCounter=0;
      callbackFunction(currFrame, timestamp);
    }
  }

  if(grabSingleFrame) {
    nextFrameAvailable = true;
    grabSingleFrame = false;
    lock.unlock();  // unlock the scoped_lock manually to ensure the notified function can continue processing immediately
    condVarFrameGrabbing.notify_one();
  }
}



void OpenNI2FrameListener::SetCbFunction(ImgCallbackFunction& cb)
{
  callbackFunction = cb;
  cbActive=false;
}


void OpenNI2FrameListener::SetFrameThrotteling(unsigned int val)
{
  frameThrottel=val;
  std::cout << "frame throtteling set to " << frameThrottel << std::endl;
};


openni::VideoFrameRef OpenNI2FrameListener::GetNextFrame()
{
  boost::mutex::scoped_lock lock(frameGrabbingMutex);

  grabSingleFrame=true;
  nextFrameAvailable=false;

  while(nextFrameAvailable == false) {
    if(!condVarFrameGrabbing.timed_wait(lock,boost::posix_time::milliseconds(500))) {
      std::cout << "!!! TIMEOUT !!!" << std::endl;
      throw(GrabbingTimeoutException(listenerName + " GetNextFrame timed out." ));
    }
  }

  // Test catch-throw mechanism
  // if(((double) rand() / (RAND_MAX))>0.9)
  //   throw(GrabbingTimeoutException(listenerName + " Testthrow" ));

  return currFrame;
}



// Example code for frame processing
// The current implementation just does nothing
void OpenNI2FrameListener::ProcessFrame(const openni::VideoFrameRef& frame)
{
  openni::DepthPixel* pDepth;
  openni::RGB888Pixel* pColor;

  int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;

  switch (frame.getVideoMode().getPixelFormat())    {
  case openni::PIXEL_FORMAT_DEPTH_1_MM:
  case openni::PIXEL_FORMAT_DEPTH_100_UM:
    // pDepth = (openni::DepthPixel*)frame.getData();
    // printf("[%08llu] %8d\n", (long long)frame.getTimestamp(),
    //        pDepth[middleIndex]);
    break;
  case openni::PIXEL_FORMAT_RGB888:
    // pColor = (openni::RGB888Pixel*)frame.getData();
    // printf("[%08llu] 0x%02x%02x%02x\n", (long long)frame.getTimestamp(),
    //        pColor[middleIndex].r&0xff,
    //        pColor[middleIndex].g&0xff,
    //        pColor[middleIndex].b&0xff);
    break;
  case openni::PIXEL_FORMAT_GRAY16:
    // usually used by the IR frame, no processing required
    break;
  case openni::PIXEL_FORMAT_GRAY8:
    // usually used by the IR frame, no processing required
    break;
  default:
    std::cout << "Unknown format" << std::endl;
  }
}

}
