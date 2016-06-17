#ifndef __ROS_WEB_CAM__
#define __ROS_WEB_CAM__


#include <asm/types.h>          /* for videodev2.h */

extern "C"
{
#include <linux/videodev2.h>
}

#include <string>
#include <vector>
#include <sstream>

namespace egomo_webcam {
  class ImageData {
  public:
  ImageData() : data(NULL), bufferSizeInByte(0), imgSizeInByte(0) {};
    ~ImageData() {if(data!=NULL) delete[] data;};
    void CreateBuffer(long size) {
      if(data!=NULL)
	delete[] data;
      data = new unsigned char[size];
      bufferSizeInByte=size;
    };

    enum ImgFormatType {
      YUYV,
      MJPG
    };

    long GetBufferSize() {return bufferSizeInByte;};
    int width;
    int height;
    ImgFormatType type;
    int imgSizeInByte;
    unsigned char *data;
  private:
    long bufferSizeInByte;
  };




  class WebCam {
  public:
    enum io_method {
      IO_METHOD_READ,
      IO_METHOD_MMAP,
      IO_METHOD_USERPTR,
    };

    struct driverInfo {
      unsigned int id;
      std::string name;
      int minVal;
      int maxVal;
      int stepSize;
      int defaultVal;
    };


    WebCam();
    ~WebCam();

    int StartDevice(std::string devName);
    void StopDevice();
    void SetImageFormatStream(ImageData::ImgFormatType type, int width, int height);
    void SetImageFormatStill(ImageData::ImgFormatType type, int width, int height);

    const ImageData *GetStillImage();
    const ImageData *GetStreamImage();
    ImageData GetCurrImgSettings();

    int AutoFocusImage();
    unsigned int SetManualFocus(int val, bool autofocus);

    const std::vector<driverInfo> &GetDeviceFeatures() {return deviceFeatures;};
    int SetV4lParameter(const std::string &paramName, int value);
    int SetV4lParameter(const std::string &paramName, const std::string value);

    std::vector<std::string> GetErrorMessages() {return errorMsgArray;};
    void ResetErrorMessageArray() {errorMsgArray.clear();};

    double CalcTimeDiff(timespec start, timespec end);

  private:
    std::string fDevName;

    int imgWidthCurr;
    int imgHeightCurr;
    ImageData::ImgFormatType imgFormatCurr;

    int imgWidthStream;
    int imgHeightStream;
    ImageData::ImgFormatType imgFormatStream;

    int imgWidthStill;
    int imgHeightStill;
    ImageData::ImgFormatType imgFormatStill;

    io_method io;
    ImageData imgData;
    std::vector<driverInfo> deviceFeatures;

    struct buffer {
      void   *start;
      size_t  length;
    };

    std::vector<std::string> errorMsgArray;
    std::stringstream errorMsgStream;

    int fd;
    struct buffer *buffers;
    unsigned int n_buffers;

    bool deviceOpened;
    bool deviceInitialized;
    bool captureRunning;

    int SwitchImageFormat(ImageData::ImgFormatType type, int width, int height);

    int xioctl(int fh, int request, void *arg);
    void errno_exit(const char *s);

    int open_device(void);
    int close_device(void);

    int init_device(void);
    void uninit_device(void);

    int init_read(unsigned int buffer_size);
    int init_mmap(void);
    int init_userp(unsigned int buffer_size);

    int start_capturing(void);
    void stop_capturing(void);

    int query_device_attributes(void);
    void enumerate_menu(unsigned int ctrlid, unsigned int min, unsigned int max);


    int grab_image(void);
    int read_frame(void);
    int process_image(const void * src, int len);
  };
}

#endif
