#include <ros/ros.h>
#include <ros/master.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <fstream>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>
#include <stdint.h>
#include <sys/time.h>

#include <OpenNI.h>

#include "egomo_depthcam/DepthImage.h"
#include "egomo_depthcam/GetNewImage.h"
#include "egomo_depthcam/SetParameters.h"
#include "egomo_depthcam_node.h"
#include "egomo_depthcam/openni2_device.h"

#include <sys/time.h>
#include <unistd.h>

#include <boost/thread/thread.hpp>

#include <png.h>  // png compression
#include "lz4.h"  // LZ4 data compression library


using namespace egomo_depthcam;


OpenNI2CamNode::OpenNI2CamNode() :
                 nodeHandle("~"),
                 seqNum(0),
		 depthStreamSeqNum(0),
                 nSubscribers(0),
		 bitReorderBuffer(NULL),
		 bitReorderBufferSize(0),
		 lz4ResultBuffer(NULL),
		 lz4ResultBufferSize(0),
                 useLZ4compression(false)
{
  clock_gettime(CLOCK_MONOTONIC, &nodeStartTime);

  int depthWidthStill;
  int depthHeightStill;
  int depthWidthStream;
  int depthHeightStream;
  int irWidthStill;
  int irHeightStill;
  int irWidthStream;
  int irHeightStream;
  int frameRate;

  nodeHandle.param("depthimage_width_still", depthWidthStill, 640);
  nodeHandle.param("depthimage_height_still", depthHeightStill, 480);
  nodeHandle.param("depthimage_width_stream", depthWidthStream, 640);
  nodeHandle.param("depthimage_height_stream", depthHeightStream, 480);
  nodeHandle.param("irimage_width_still", irWidthStill, 640);
  nodeHandle.param("irimage_height_still", irHeightStill, 480);
  nodeHandle.param("irimage_width_stream", irWidthStream, 640);
  nodeHandle.param("irimage_height_stream", irHeightStream, 480);
  nodeHandle.param("frame_rate", frameRate, 30);
  nodeHandle.param("use_lz4compress", useLZ4compression, false);

  device.SetResolution(OpenNI2Device::kDepthStill, depthWidthStill, depthHeightStill);
  device.SetResolution(OpenNI2Device::kDepthStream, depthWidthStream, depthHeightStream);
  device.SetResolution(OpenNI2Device::kIRstill, irWidthStill, irHeightStill);
  device.SetResolution(OpenNI2Device::kIRstream, irWidthStream, irHeightStream);
  int newFrameRate=device.SetFrameRate(frameRate);
  if(newFrameRate!=frameRate) {
    ROS_WARN("Requested frame rate of %i fps not supported. Set to %i fps.", frameRate, newFrameRate);
  }

  AdvertiseService();
}

OpenNI2CamNode::~OpenNI2CamNode() {

}


void OpenNI2CamNode::AdvertiseService()
{
  std::cout << "Advertising egomo depthcam service..." << std::endl;
  serverGetNewImage = nodeHandle.advertiseService("get_new_image",
						  &OpenNI2CamNode::RecordSendImage, this);
  serverSetParameters = nodeHandle.advertiseService("set_parameters",
						  &OpenNI2CamNode::SetParameters, this);

  boost::lock_guard<boost::mutex> lock(connectionMutex);
  ros::SubscriberStatusCallback subscriberCb = boost::bind(&OpenNI2CamNode::ConnectDepthCb, this);
  serverDepthStream = nodeHandle.advertise<egomo_depthcam::DepthImage>("depthstream/image", 1, subscriberCb, subscriberCb);
  device.RegisterDepthFrameCallback(boost::bind(&OpenNI2CamNode::NewDepthImgCallback, this, _1, _2));
}


bool OpenNI2CamNode::NewDepthImgCallback(openni::VideoFrameRef &currFrame, struct timeval timestamp)
{
  timespec timeCbStart, timeCbEnd;  // debug: time the whole callback routine
  clock_gettime(CLOCK_MONOTONIC, &timeCbStart);


  depthStreamSeqNum++;

  // timespec timeCurr;
  // if(callbackCounter==0)
  //   clock_gettime(CLOCK_MONOTONIC, &timerFramerate);
  // if(callbackCounter>10) {
  //   clock_gettime(CLOCK_MONOTONIC, &timeCurr);
  //   std::cout << "10 callbacks took " << device.CalcTimeDiff(timerFramerate, timeCurr) << " ms" << std::endl;
  //   callbackCounter=0;
  //   clock_gettime(CLOCK_MONOTONIC, &timerFramerate);
  // }

  depthStreamImg.header.stamp = ros::Time::now();
  depthStreamImg.header.seq = depthStreamSeqNum;
  depthStreamImg.header.frame_id = "";

  depthStreamImg.height = (unsigned int) currFrame.getHeight();
  depthStreamImg.width = (unsigned int) currFrame.getWidth();

  depthStreamImg.is_bigendian=false;
  depthStreamImg.step=currFrame.getStrideInBytes();
  depthStreamImg.data_size_uncompressed = currFrame.getDataSize();

  if(useLZ4compression) {
    if(bitReorderBufferSize != currFrame.getDataSize()) {
      //std::cout << "Realloc bit reorder buffer size" << std::endl;
      delete[] bitReorderBuffer;
      delete[] lz4ResultBuffer;

      bitReorderBufferSize=currFrame.getDataSize();
      bitReorderBuffer = new char[bitReorderBufferSize];

      lz4ResultBufferSize = bitReorderBufferSize + 0.1*bitReorderBufferSize;  // add some reserve in case of incompressible data
      lz4ResultBuffer = new char[bitReorderBufferSize];

      //std::cout << "New sizes: bitReorderBufferSize=" << bitReorderBufferSize << ", lz4ResultBufferSize=" << lz4ResultBufferSize << std::endl;
    }


    timespec timeStart, timeEnd;  // debug: time the main compression routine
    clock_gettime(CLOCK_MONOTONIC, &timeStart);
    ReorderBits((char*)currFrame.getData(), currFrame.getDataSize(), bitReorderBuffer, bitReorderBufferSize);
    clock_gettime(CLOCK_MONOTONIC, &timeEnd);
    // std::cout<< "Bit reorder time: " << device.CalcTimeDiff(timeStart, timeEnd) << " ms" << std::endl;

    clock_gettime(CLOCK_MONOTONIC, &timeStart);
    depthStreamImg.data_size=CompressLZ4(bitReorderBuffer,  depthStreamImg.data_size_uncompressed, lz4ResultBuffer);
    clock_gettime(CLOCK_MONOTONIC, &timeEnd);
    // std::cout<< "LZ4 compression time: " << device.CalcTimeDiff(timeStart, timeEnd) << " ms" << std::endl;

    depthStreamImg.data.resize(depthStreamImg.data_size);
    memcpy(&depthStreamImg.data[0], lz4ResultBuffer, depthStreamImg.data_size);
    depthStreamImg.compressionType="lz4";
  }
  else {
    depthStreamImg.data_size = depthStreamImg.data_size_uncompressed;
    depthStreamImg.data.resize(depthStreamImg.data_size);
    memcpy(&depthStreamImg.data[0], (char*)currFrame.getData(), currFrame.getDataSize());

    depthStreamImg.compressionType="none";
  }

  struct timeval tp;
  gettimeofday(&tp, NULL);
  depthStreamImg.timeSend_sec = tp.tv_sec;
  depthStreamImg.timeSend_usec = tp.tv_usec;

  serverDepthStream.publish(depthStreamImg);

  callbackCounter++;
  clock_gettime(CLOCK_MONOTONIC, &timeCbEnd);
  // std::cout<< "Overall Cb time: " << device.CalcTimeDiff(timeCbStart, timeCbEnd) << " ms" << std::endl;
  return true;
}

void OpenNI2CamNode::ConnectDepthCb()
{
  boost::lock_guard<boost::mutex> lock(connectionMutex);
  nSubscribers = serverDepthStream.getNumSubscribers(); // update the number of connected clients
  // std::cout << "number of connected clients: " << nSubscribers << std::endl;

  if(nSubscribers>0) {
    device.StartDepthStream();
  }
  else {
    device.StopDepthStream();
  }
  // optional: do something if someone connects or nSubscribers==0 (=noone connected)
}


bool OpenNI2CamNode::RecordSendImage(egomo_depthcam::GetNewImageRequest& req, egomo_depthcam::GetNewImageResponse& res)
{
  timespec timeTotalStart, timeTotalEnd;
  clock_gettime(CLOCK_MONOTONIC, &timeTotalStart);

  OpenNI2Device::ImageType imgType = (OpenNI2Device::ImageType)req.imgType;
  const openni::VideoFrameRef *imgData;
  timespec timeStart, timeEnd;

  // std::cout << "-------------- Capturing image type " << imgType << std::endl;

  clock_gettime(CLOCK_MONOTONIC, &timeStart);
  imgData = device.GetSingleImage(imgType);
  clock_gettime(CLOCK_MONOTONIC, &timeEnd);
  // std::cout<< "Overall img capture time: " << device.CalcTimeDiff(timeStart, timeEnd) << " ms" << std::endl;
  if(imgData==NULL)  // some error occured and we don't have a valid image
    return false;

  res.img.header.stamp = ros::Time::now();
  res.img.header.seq = seqNum;
  seqNum++;
  res.img.header.frame_id = "";

  res.img.height = (unsigned int) imgData->getHeight();
  res.img.width = (unsigned int) imgData->getWidth();

  res.img.encoding = "";
  res.img.is_bigendian=false;
  res.img.step=imgData->getStrideInBytes();

  size_t st0 = (imgData->getDataSize());
  res.img.data.resize(st0);

  memcpy(&res.img.data[0], imgData->getData(), st0);

  clock_gettime(CLOCK_MONOTONIC, &timeTotalEnd);
  // std::cout << "OpenNI2CamNode::RecordSendImage total time " << device.CalcTimeDiff(timeTotalStart, timeTotalEnd) << " ms" << std::endl;
  return true;
}


// Dynamic reconfiguration is currently not supported
bool OpenNI2CamNode::SetParameters(egomo_depthcam::SetParametersRequest& req, egomo_depthcam::SetParametersResponse& res)
{
  // // std::cout << "This is SetParamters" << std::endl;

  // timespec timeTotalStart, timeTotalEnd;
  // clock_gettime(CLOCK_MONOTONIC, &timeTotalStart);

  // int imgWidth = req.imgWidth;
  // int imgHeight = req.imgHeight;
  // int camStream = req.camStream;
  // int irProjectorStatus = req.irProjector;

  // if(irProjectorStatus<0) {
  //   res.success=device.SetResolution(camStream, imgWidth, imgHeight);
  //   clock_gettime(CLOCK_MONOTONIC, &timeTotalEnd);
  //   std::cout << "Switching resolution took " << device.CalcTimeDiff(timeTotalStart, timeTotalEnd) << " ms" << std::endl;
  // }
  // else if(irProjectorStatus>=0 && camStream<0) {
  //   if(irProjectorStatus==0)
  //     device.StopDepthStream();
  //   else
  //     device.StartDepthStream();
  // }

  return true;
}


void OpenNI2CamNode::TryReconnect()
{
  nodeHandle.shutdown(); // Shutdown every handle created through the NodeHandle.
  std::cout << "Connection to master lost. Trying to reconnect ..." << std::endl;
  do {
    usleep(1*1000*1000); // sleep for one second
  } while(ros::master::check() == false);

  std::cout << "Connection re-established." << std::endl;
  AdvertiseService();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "egomo_depthcam");
  egomo_depthcam::OpenNI2CamNode a;
  static const float loopFreq=60;
  static const float freqCheckROScore=2; // check connection to master with 2 Hz

  float counter=0;
  while (ros::ok()) {
    if(counter > loopFreq/freqCheckROScore) {
      counter=0;
      if(ros::master::check() == false) {
        a.TryReconnect();
      }
    }
    counter++;

    // from documentation:
    // Parameters for callAvailable:
    //  timeout     The amount of time to wait for at least one callback to be available.
    //              If there is already at least one callback available, this parameter does nothing.
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0/loopFreq));
  }
  return EXIT_SUCCESS;
}


// Reorders the bits within the depth image so that the LZ4 algorithm can achive better compression performance
char *OpenNI2CamNode::ReorderBits(const char *inData, int lengthByte, char *out, int outLenghtBytes)
{
  if(out==NULL) {
    if(lengthByte%16 == 0)
      outLenghtBytes=lengthByte;
    else
      outLenghtBytes = ((lengthByte/16)+1)*16;

    out = new char[outLenghtBytes];
  }

  int nPixel=lengthByte/2;
  int stride=lengthByte/16;
  for(int i=0; i<lengthByte; i++)
    out[i]=0;

  int buffer[16];
  for(int i=0; i<16; i++)
    buffer[i]=0;
  long outPos=0;
  int bitpos=0;

  const unsigned short *in=reinterpret_cast<const unsigned short*>(inData);

  for(int i=0; i<nPixel; i++) {
    if(bitpos>=8) {
      for(int j=0; j<16; j++) {
   	//std::cout << "loop " << outPos << ", j=" << j << ": " << std::bitset<8>(buffer[j]) << std::endl;
   	out[j*stride + outPos] = buffer[j];
   	buffer[j]=0;
      }
      outPos++;
      bitpos=0;
    }

    for(int j=0; j<16; j++) {
      buffer[15-j] |= ((in[i] >> j) & 1) << (7-bitpos);
    }

    bitpos++;
  }

  for(int j=0; j<16; j++)
     out[long(j*stride) + outPos] = buffer[j];

  return out;
}


int OpenNI2CamNode::CompressLZ4(char *inputBuffer, int inputSize, char *outputBuffer)
{
  static const int BLOCK_BYTES = 1024 * 8;

  LZ4_stream_t lz4Stream_body;
  LZ4_stream_t* lz4Stream = &lz4Stream_body;

  int inpos=0;
  int outpos=0;
  LZ4_resetStream(lz4Stream);

  int cmpBytes=0;
  while(inputSize>0) {
    int inpBytes;
    if(inputSize>=BLOCK_BYTES)
      inpBytes=BLOCK_BYTES;
    else
      inpBytes=inputSize;

    char cmpBuf[LZ4_COMPRESSBOUND(BLOCK_BYTES)];
    cmpBytes = LZ4_compress_fast_continue(lz4Stream, &inputBuffer[inpos], cmpBuf, inpBytes, sizeof(cmpBuf), 1);
    if(cmpBytes <= 0) {
      std::cout << "Error in compression function" << std::endl;
      break;
    }

    std::memcpy(&outputBuffer[outpos], reinterpret_cast<char *>(&cmpBytes), sizeof(cmpBytes));
    outpos+=sizeof(cmpBytes);
    std::memcpy(&outputBuffer[outpos], cmpBuf, (size_t) cmpBytes);
    outpos+=cmpBytes;
    inpos += inpBytes;
    inputSize -= inpBytes;
  }

  // add a next-block-size = 0 information to indicate end of compressed data
  cmpBytes=0;
  std::memcpy(&outputBuffer[outpos], reinterpret_cast<char *>(&cmpBytes), sizeof(cmpBytes));
  outpos+=sizeof(cmpBytes);

  return outpos;
}



// Convert the raw data contained in VideoFrameRef to an PNG image stored in imgCompressed
// PNG parameters like compression factor, etc. are hard-coded at the moment
// For the raspberry, the PNG compression requires to much time so currently a LZ4 with "manual" bit
// reordering is used.
bool OpenNI2CamNode::CompressPNG(const openni::VideoFrameRef *imgData, std::vector<unsigned char> &imgCompressed)
{
 // convert the image to png format
  // TODO: check if memory allocation/de-allocation can be optimized

  int imgHeight = imgData->getHeight();
  int imgWidth = imgData->getWidth();
  int imgStrideInBytes = imgData->getStrideInBytes();
  const unsigned char *inputData = (const unsigned char *)imgData->getData();

  // data structure which will hold the compressed image
  struct pngMemEncode pngBuffer;
  pngBuffer.buffer=NULL;
  pngBuffer.size=0;

  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png_ptr)
    return false;
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
    return false;
  }
  if (setjmp(png_jmpbuf(png_ptr))) {  // c style error handling
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return false;
  }
  // register callback functions to write the compressed data to memory instead of disk file
  png_set_write_fn(png_ptr, &pngBuffer, LocalPNGWriteData, LocalPNGFlush);

  // set img properties: width, height,
  //                     bit_depth, color_type, interlace_type,
  //                     compression_type, filter_method
  png_set_IHDR(png_ptr, info_ptr, imgWidth, imgHeight,
	       16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
	       PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  png_set_compression_level( png_ptr, 2);  // png compression level 2: compromise of file size and compression speed
  // write the png header data
  png_write_info(png_ptr, info_ptr);

  // perform the actual compression
  png_bytepp rowPointers = (png_bytepp)png_malloc(png_ptr, imgHeight * sizeof(png_bytep));
  for(int y=0; y<imgHeight; y++)
    rowPointers[y] = (png_bytep)(&inputData[y*imgData->getStrideInBytes()]);


  //timespec timeStart, timeEnd;  // debug: time the main png compression routine
  //clock_gettime(CLOCK_MONOTONIC, &timeStart);

  png_write_image(png_ptr, rowPointers);

  //clock_gettime(CLOCK_MONOTONIC, &timeEnd);
  //std::cout << "core of png compression took " << device.CalcTimeDiff(timeStart, timeEnd) << " ms" << std::endl;


  png_write_end(png_ptr, NULL);

  // at this point pngBuffer.buffer contains the PNG image of size pngBuffer.size bytes
  std::cout << "png compressed size: " << pngBuffer.size << std::endl;

  // debug: dump resulting png and the raw data to files
  // std::ofstream bStream("test_my.png", std::fstream::out | std::fstream::binary);
  // if (bStream) {
  //   bStream.write(pngBuffer.buffer, pngBuffer.size);
  //   std::cout << "bstream status: " << bStream.good() << std::endl;
  // }
  // std::ofstream bStreamB("test_my.bin", std::fstream::out | std::fstream::binary);
  // if (bStreamB) {
  //   bStreamB.write(((char *)imgData->getData()), imgData->getDataSize());
  //   std::cout << "bstreamB status: " << bStreamB.good() << std::endl;
  // }

  //size_t st0 = (imgData->getDataSize());
  imgCompressed.resize(pngBuffer.size);
  memcpy(&imgCompressed[0], pngBuffer.buffer, pngBuffer.size);

  // delete the rowPointers array
  png_free(png_ptr, rowPointers);
  // cleanup: delete the internal data structues
  png_destroy_write_struct(&png_ptr, &info_ptr);

  return true;
}



// Utility function for CompressPNG, don't call it from elsewhere
void OpenNI2CamNode::LocalPNGWriteData(png_structp png_ptr, png_bytep data, png_size_t length)
{
  // with libpng15 next line causes pointer deference error; use libpng12
  struct pngMemEncode* p=(struct pngMemEncode*)png_get_io_ptr(png_ptr); // was png_ptr->io_ptr
  size_t nsize = p->size + length;

  // allocate or grow buffer
  if(p->buffer)
    p->buffer = (char*) realloc(p->buffer, nsize);
  else
    p->buffer = (char*) malloc(nsize);

  if(!p->buffer)
    png_error(png_ptr, "Write Error");

  /* copy new bytes to end of buffer */
  memcpy(p->buffer + p->size, data, length);
  p->size += length;
}


// This is optional but included to show how png_set_write_fn() is called
void OpenNI2CamNode::LocalPNGFlush(png_structp png_ptr)
{
  return;
}


long OpenNI2CamNode::TimeDiffSinceStart(timespec compareTime)
{
  long timediff = ( compareTime.tv_sec - (nodeStartTime.tv_sec+1))*1000*1000; // to get microseconds
  timediff += (compareTime.tv_nsec + ((1000*1000*1000) - nodeStartTime.tv_nsec)) / 1000;

  return timediff;
}
