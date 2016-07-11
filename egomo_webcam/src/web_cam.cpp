#include <string>
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include "../include/egomo_webcam/web_cam.h" // <web_cam/web_cam.h>

#include <fstream>
#include <iostream>

#include <sys/time.h>
#include <unistd.h>


#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace egomo_webcam {

  WebCam::WebCam() :
    fDevName(""),
    imgWidthCurr(640),
    imgHeightCurr(480),
    imgFormatCurr(ImageData::YUYV),
    imgWidthStream(640),
    imgHeightStream(480),
    imgFormatStream(ImageData::MJPG),
    imgWidthStill(640),
    imgHeightStill(480),
    imgFormatStill(ImageData::YUYV),
    imgWidthStreamRaw(640),
    imgHeightStreamRaw(480),
    imgFormatStreamRaw(ImageData::YUYV),
    io(IO_METHOD_MMAP),
    fd(-1),
    buffers(NULL),
    n_buffers(0),
    deviceOpened(false),
    deviceInitialized(false),
    captureRunning(false)
  {
    return;
  }

  WebCam::~WebCam()
  {
    if(captureRunning)
      stop_capturing();
    if(deviceInitialized)
      uninit_device();
    if(deviceOpened)
      close_device();
  }

  int WebCam::StartDevice(std::string devName)
  {
    // close device if its already running
    if(deviceOpened==true) {
      if(captureRunning)
	stop_capturing();
      if(deviceInitialized)
	uninit_device();
      if(deviceOpened)
	close_device();
    }

    fDevName=devName;

    if(open_device()!=0) {
      errorMsgStream << "Cannot open device " << fDevName << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");

      return -1;
    }
    if(init_device()!=0) {
      errorMsgStream << "Cannot init device " << fDevName << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");

      return -2;
    }

    return 0;
  }

  void WebCam::StopDevice()
  {
    if(captureRunning)
      stop_capturing();
    if(deviceInitialized)
      uninit_device();
    if(deviceOpened)
      close_device();
  }

  void WebCam::SetImageFormatStream(ImageData::ImgFormatType type, int width, int height)
  {
    imgFormatStream = type;
    imgWidthStream = width;
    imgHeightStream = height;
  }

  void WebCam::SetImageFormatStreamRaw(ImageData::ImgFormatType type, int width, int height)
  {
    imgFormatStreamRaw = type;
    imgWidthStreamRaw = width;
    imgHeightStreamRaw = height;
  }

  void WebCam::SetImageFormatStill(ImageData::ImgFormatType type, int width, int height)
  {
    imgFormatStill = type;
    imgWidthStill = width;
    imgHeightStill = height;
  }


  const ImageData *WebCam::GetStreamImage()
  {
    // if(StartDevice(devName, width, height)!=0)
    //   return NULL;

   if(deviceInitialized==false) {
      errorMsgStream << "Device " << fDevName << " is not initialized." << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return NULL;
    }

   if(imgWidthCurr != imgWidthStream || imgHeightCurr != imgHeightStream || imgFormatCurr != imgFormatStream)
     SwitchImageFormat(imgFormatStream, imgWidthStream, imgHeightStream);

   start_capturing();
   if(grab_image()!=0)
     return NULL;

   // If the requested image size is not supported by the camera, an image with closesed supported resolution is recorded.
   // We update the settings to reflect that resolution change
   imgWidthStream = imgData.width;
   imgHeightStream = imgData.height;

   return &imgData;
  }


  const ImageData *WebCam::GetStreamImageRaw()
  {
    // if(StartDevice(devName, width, height)!=0)
    //   return NULL;

   if(deviceInitialized==false) {
      errorMsgStream << "Device " << fDevName << " is not initialized." << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return NULL;
    }

   if(imgWidthCurr != imgWidthStreamRaw || imgHeightCurr != imgHeightStreamRaw || imgFormatCurr != imgFormatStreamRaw)
     SwitchImageFormat(imgFormatStreamRaw, imgWidthStreamRaw, imgHeightStreamRaw);

   start_capturing();
   if(grab_image()!=0)
     return NULL;

   // If the requested image size is not supported by the camera, an image with closesed supported resolution is recorded.
   // We update the settings to reflect that resolution change
   imgWidthStreamRaw = imgData.width;
   imgHeightStreamRaw = imgData.height;

   return &imgData;
  }


  const ImageData *WebCam::GetStillImage()
  {
    timespec timeStart, timeEnd;

   if(deviceInitialized==false) {
      errorMsgStream << "Device " << fDevName << " is not initialized." << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return NULL;
    }

   if(imgWidthCurr == imgWidthStill && imgHeightCurr == imgHeightStill && imgFormatCurr == imgFormatStill) {
     clock_gettime(CLOCK_MONOTONIC, &timeStart);
     stop_capturing();  // an easy and save way to ensure that all buffers are emptied
     start_capturing(); // and an actually new image is recorded when calling grab_image
     clock_gettime(CLOCK_MONOTONIC, &timeEnd);
   }
   else {
     clock_gettime(CLOCK_MONOTONIC, &timeStart);
     SwitchImageFormat(imgFormatStill, imgWidthStill, imgHeightStill);
     clock_gettime(CLOCK_MONOTONIC, &timeEnd);
   }

   // std::cout<< "Cam init/switch format: " << CalcTimeDiff(timeStart, timeEnd) << " ms" << std::endl;

   clock_gettime(CLOCK_MONOTONIC, &timeStart);
   start_capturing();
   if(grab_image()!=0)
      return NULL;
   clock_gettime(CLOCK_MONOTONIC, &timeEnd);

   // std::cout<< "Actual capture operation: " << CalcTimeDiff(timeStart, timeEnd) << " ms" << std::endl;

   // If the requested image size is not supported by the camera, an image with closesed supported resolution is recorded.
   // We update the settings to reflect that resolution change
   imgWidthStill = imgData.width;
   imgHeightStill = imgData.height;

   return &imgData;
  }


  ImageData WebCam::GetCurrImgSettings()
  {
    ImageData data;
    data.width = imgWidthCurr;
    data.height = imgHeightCurr;
    data.type = imgFormatCurr;

    return data;
  }


  int WebCam::AutoFocusImage()
  {
    int prevResWidth=imgWidthCurr;
    int prevResHeight=imgHeightCurr;

    stop_capturing();
    if(deviceInitialized)
      uninit_device();

    if(init_device()!=0)
      return -2;

    if(start_capturing()!=0)
      return -3;

    for(int i=0; i<1; i++) {
      grab_image();

      struct v4l2_ext_control control[2];
      struct v4l2_ext_controls extCtrl;

      memset(&extCtrl, 0, sizeof(extCtrl));

      extCtrl.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
      extCtrl.count=2;
      extCtrl.controls=control;

      control[0].id = V4L2_CID_FOCUS_ABSOLUTE;
      control[0].value=250;
      control[1].id = V4L2_CID_FOCUS_AUTO;
      control[1].value=false;

      if (-1 == xioctl(fd, VIDIOC_S_EXT_CTRLS, &extCtrl)) {
	perror("VIDIOC_S_EXT_CTRLS");
	return -4;
      }
    }

    StopDevice();
    StartDevice(fDevName);

    return 0;
  }


  unsigned int WebCam::SetManualFocus(int val, bool autofocus)
  {
    if(deviceInitialized==false) {
      errorMsgStream << "Device " << fDevName << " is not initialized." << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return -1;
    }

    struct v4l2_ext_controls extCtrl;
    struct v4l2_ext_control control[1];

    memset(&extCtrl, 0, sizeof(extCtrl));
    extCtrl.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
    extCtrl.count=1;
    extCtrl.controls=control;

    // disable or enable autofocus
    // autofocus has to be disabled before settings the focus manually
    if(autofocus == false) {
      control[0].id = V4L2_CID_FOCUS_AUTO;
      control[0].size=0;
      control[0].value=false;
    }
    else {
      control[0].id = V4L2_CID_FOCUS_AUTO;
      control[0].size=0;
      control[0].value=true;
    }

    if (-1 == xioctl(fd, VIDIOC_S_EXT_CTRLS, &extCtrl)) {
      errorMsgStream << "Error disabling autofocus: " << fDevName << ": " << errno << ", " << strerror(errno) << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");

      return -4;
    }

    // After the autofocus is disabled, we can set the focus value
    if(autofocus == false) {
      memset(&extCtrl, 0, sizeof(extCtrl));
      extCtrl.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
      extCtrl.count=1;
      extCtrl.controls=control;
      control[0].id = V4L2_CID_FOCUS_ABSOLUTE;
      control[0].size=0;
      control[0].value=val;

      if (-1 == xioctl(fd, VIDIOC_S_EXT_CTRLS, &extCtrl)) {
	errorMsgStream << "Error setting focus manually: " << fDevName << ": " << errno << ", " << strerror(errno) << std::endl;
	errorMsgArray.push_back(errorMsgStream.str());
	errorMsgStream.str("");

	return -4;
      }
      return control[0].value;  // return the new focus setting
    }

    return 0;
  }


  int WebCam::SetV4lParameter(const std::string &paramName, int value)
  {
    return 0;
  }


  int WebCam::SetV4lParameter(const std::string &paramName, const std::string value)
  {
    return 0;
  }



  // *********** Private functions below this line

  int WebCam::SwitchImageFormat(ImageData::ImgFormatType type, int width, int height)
  {
    // We have to reinit the device to change resolution (new memory mapping, changed buffer size, etc.)
    bool captureWasRunning = captureRunning;
    bool deviceWasInit = deviceInitialized;

    StopDevice();

    imgWidthCurr = width;
    imgHeightCurr = height;
    imgFormatCurr = type;

    int errCode=StartDevice(fDevName);
    if(errCode!=0)
      return errCode;

    if(captureWasRunning && start_capturing()!=0)
      return -3;

    return 0;
  }


  // most of the code below this line has been taken from https://linuxtv.org/downloads/v4l-dvb-apis/capture-example.html

  int WebCam::xioctl(int fh, int request, void *arg)
  {
    int r;

    do {
      r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
  }

  // TODO: replace this function
  void WebCam::errno_exit(const char *s)
  {
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }


  int WebCam::open_device(void)
  {
    struct stat st;

    if (-1 == stat(fDevName.c_str(), &st)) {
      errorMsgStream << "Cannot identify " << fDevName << ": " << errno << ", " << strerror(errno) << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return -1;
    }

    if (!S_ISCHR(st.st_mode)) {
      errorMsgStream << fDevName << " is not a device" << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return -2;
    }

    fd = open(fDevName.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd) {
      errorMsgStream << "Cannot open " << fDevName << ": " << errno << ", " << strerror(errno) << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return -3;
    }

    deviceOpened=true;

    query_device_attributes();

    return 0;
  }


  int WebCam::close_device(void)
  {
    if (-1 == close(fd)) {
      errorMsgStream << "Error closing device " << fDevName << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      fd = -1;
      return -1;
    }

    fd = -1;

    deviceOpened=false;
    return 0;
  }


  int WebCam::init_device(void)
  {
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;

    /* Check if the device is a video device ...  */
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
      if (EINVAL == errno) {
	errorMsgStream << fDevName << " is not a V4L2 device." << std::endl;
	errorMsgArray.push_back(errorMsgStream.str());
	errorMsgStream.str("");
	return -1;
      } else {
	errno_exit("VIDIOC_QUERYCAP");
      }
    }

    /* ... supports Video4Linux 2 at all */
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
	errorMsgStream << fDevName << " is no video capture device." << std::endl;
	errorMsgArray.push_back(errorMsgStream.str());
	errorMsgStream.str("");
	return -1;
    }

    /* Check if the requested data transport method (single frame read or streaming) is supported */
    switch (io) {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
	errorMsgStream << fDevName << " does not support read i/o." << std::endl;
	errorMsgArray.push_back(errorMsgStream.str());
	errorMsgStream.str("");
	return -1;
      }
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
	errorMsgStream << fDevName << " does not support streaming i/o" << std::endl;
	errorMsgArray.push_back(errorMsgStream.str());
	errorMsgStream.str("");
	return -1;
      }
      break;
    }

    /* Set cropping window back to default (full sensor) */
    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
      crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      crop.c = cropcap.defrect; /* reset to default */

      if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
	switch (errno) {
	case EINVAL:
	  /* Cropping not supported. */
	  break;
	default:
	  /* Errors ignored. */
	  break;
	}
      }
    } else {
      /* Errors ignored. */
    }


    /* Select video/image type and resolution */
    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    fmt.fmt.pix.width       = imgWidthCurr;
    fmt.fmt.pix.height      = imgHeightCurr;

    std::cout << "Img init resolution " << imgWidthCurr << ", height " << imgHeightCurr << std::endl;

    switch(imgFormatCurr) {
    case ImageData::YUYV:
      fmt.fmt.pix.pixelformat =  V4L2_PIX_FMT_YUYV; // V4L2_PIX_FMT_MJPEG;
      fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
      break;
    case ImageData::MJPG:
      fmt.fmt.pix.pixelformat =  V4L2_PIX_FMT_MJPEG; // V4L2_PIX_FMT_MJPEG;
      fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
      break;
    default:
      errorMsgStream << "Unkown/unsupported image format: " << imgFormatCurr << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return -2;
      break;
    }

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
      std::cout << "EROROOROOR" << std::endl;
      errno_exit("VIDIOC_S_FMT");
    }

      /* Note VIDIOC_S_FMT may change width and height. */
    // so we update width and height information
    imgWidthCurr=fmt.fmt.pix.width;
    imgHeightCurr=fmt.fmt.pix.height;

    // /* Preserve original settings as set by v4l2-ctl for example */
    // if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
    //  errno_exit("VIDIOC_G_FMT");

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
      fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
      fmt.fmt.pix.sizeimage = min;

    int initMemResult=0;
    switch (io) {
    case IO_METHOD_READ:
      initMemResult=init_read(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_MMAP:
      initMemResult=init_mmap();
      break;

    case IO_METHOD_USERPTR:
      initMemResult=init_userp(fmt.fmt.pix.sizeimage);
      break;
    }

    if(initMemResult==0) {
      imgData.width=imgWidthCurr;
      imgData.height=imgHeightCurr;
      imgData.imgSizeInByte=fmt.fmt.pix.sizeimage;
      imgData.CreateBuffer(buffers[0].length);
    std::cout << "Img resulting size " << imgWidthCurr << ", height " << imgHeightCurr << std::endl;


    }

    if(initMemResult==0)
      deviceInitialized=true;

    return initMemResult;
  }


  void WebCam::uninit_device(void)
  {
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
      free(buffers[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i)
	if (-1 == munmap(buffers[i].start, buffers[i].length))
	  errno_exit("munmap");
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i)
	free(buffers[i].start);
      break;
    }

    free(buffers);
    deviceInitialized=false;
  }


  int WebCam::init_read(unsigned int buffer_size)
  {
    buffers = (buffer*) calloc(1, sizeof(*buffers));

    if (!buffers) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc(buffer_size);

    if (!buffers[0].start) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }

    return 0;
  }

  int WebCam::init_mmap(void)
  {
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 2;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
      if (EINVAL == errno) {
	errorMsgStream << fDevName << " does not support memory mapping" << std::endl;
	errorMsgArray.push_back(errorMsgStream.str());
	errorMsgStream.str("");
	return -1;
      } else {
	errno_exit("VIDIOC_REQBUFS");
      }
    }
    printf("Number of active buffers: %i\n", req.count);

    if (req.count < 2) {
      errorMsgStream << "Insufficient buffer memory on " << fDevName  << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return -1;
    }

    buffers = (buffer*) calloc(req.count, sizeof(*buffers));

    if (!buffers) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
      struct v4l2_buffer buf;

      CLEAR(buf);

      buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory      = V4L2_MEMORY_MMAP;
      buf.index       = n_buffers;

      if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
	errno_exit("VIDIOC_QUERYBUF");

      buffers[n_buffers].length = buf.length;
      buffers[n_buffers].start =
	mmap(NULL /* start anywhere */,
	     buf.length,
	     PROT_READ | PROT_WRITE /* required */,
	     MAP_SHARED /* recommended */,
	     fd, buf.m.offset);

      if (MAP_FAILED == buffers[n_buffers].start)
	errno_exit("mmap");
    }

    return 0;
  }

  int WebCam::init_userp(unsigned int buffer_size)
  {
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count  = 2;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
      if (EINVAL == errno) {
	errorMsgStream << fDevName << " does not support user pointer i/o" << std::endl;
	errorMsgArray.push_back(errorMsgStream.str());
	errorMsgStream.str("");

	return -1;
      } else {
	errno_exit("VIDIOC_REQBUFS");
      }
    }

    buffers = (buffer*) calloc(1, sizeof(*buffers));

    if (!buffers) {
      fprintf(stderr, "Out of memory\n");
      exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < 1; ++n_buffers) {
      buffers[n_buffers].length = buffer_size;
      buffers[n_buffers].start = malloc(buffer_size);

      if (!buffers[n_buffers].start) {
	fprintf(stderr, "Out of memory\n");
	exit(EXIT_FAILURE);
      }
    }

    return 0;
  }


 int WebCam::start_capturing(void)
  {
    if(captureRunning == true)
      return 0;

    unsigned int i;
    enum v4l2_buf_type type;

    switch (io) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i) {
	struct v4l2_buffer buf;

	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = i;

	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
	  errno_exit("VIDIOC_QBUF");
      }
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
	errno_exit("VIDIOC_STREAMON");
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i) {
	struct v4l2_buffer buf;

	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.index = i;
	buf.m.userptr = (unsigned long)buffers[i].start;
	buf.length = buffers[i].length;

	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
	  errno_exit("VIDIOC_QBUF");
      }
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
	errno_exit("VIDIOC_STREAMON");
      break;
    }

    captureRunning=true;
    return 0;
  }


 void WebCam::stop_capturing(void)
  {
    if(captureRunning==false)
      return;

    enum v4l2_buf_type type;

    switch (io) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
	errno_exit("VIDIOC_STREAMOFF");
      break;
    }

    captureRunning=false;
  }


  int WebCam::query_device_attributes(void)
  {
    deviceFeatures.clear();

    struct v4l2_queryctrl queryctrl;
    driverInfo camProperty;

    memset(&queryctrl, 0, sizeof(queryctrl));

    queryctrl.id = V4L2_CTRL_CLASS_USER | V4L2_CTRL_FLAG_NEXT_CTRL;
    while (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
      if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
	continue;

      camProperty.id=queryctrl.id;
      char _tmp[32];
      snprintf(_tmp, 32, "%s", queryctrl.name);
      camProperty.name=_tmp;
      camProperty.minVal=queryctrl.minimum;
      camProperty.maxVal=queryctrl.maximum;
      camProperty.stepSize=queryctrl.step;
      camProperty.defaultVal=queryctrl.default_value;
      deviceFeatures.push_back(camProperty);

      // if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
      // 	enumerate_menu(queryctrl.id, queryctrl.minimum, queryctrl.maximum);

      queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    // if (errno != EINVAL) {
    //   std::cout << "error" << std::endl;
    //   perror("VIDIOC_QUERYCTRL");
    //   exit(EXIT_FAILURE);
    // }
  }

  // results of this function are not used at the moment
  void WebCam::enumerate_menu(unsigned int ctrlid, unsigned int min, unsigned int max)
  {
    std::cout << "SubMenue" << std::endl;
    struct v4l2_querymenu querymenu;
    printf("  Menu items:\n");

    memset(&querymenu, 0, sizeof(querymenu));
    querymenu.id = ctrlid;

    for (querymenu.index = min;
	 querymenu.index <= max;
	 querymenu.index++) {
      if (0 == ioctl(fd, VIDIOC_QUERYMENU, &querymenu)) {
	printf("  %s\n", querymenu.name);
      }
    }
    std::cout << "SubMenue END" << std::endl;
  }



  int WebCam::grab_image(void)
  {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    /* Timeout. */
    tv.tv_sec = 3;
    tv.tv_usec = 0;

    /* From the manual (https://linuxtv.org/downloads/v4l-dvb-apis/func-select.html): */
    /*  With the select() function applications can suspend execution
     *  until the driver has captured data or is ready to accept data
     *   for output.
     *
     * When streaming I/O has been negotiated this function waits until a
     * buffer has been filled or displayed and can be dequeued with the
     * VIDIOC_DQBUF ioctl. When buffers are already in the outgoing queue of
     * the driver the function returns immediately.
     *
     * On success select() returns the total number of bits set in the
     * fd_sets. When the function timed out it returns a value of zero. On
     * failure it returns -1 and the errno variable is set
     * appropriately. When the application did not call VIDIOC_QBUF or
     * VIDIOC_STREAMON yet the select() function succeeds, setting the bit of
     * the file descriptor in readfds or writefds, but subsequent
     * VIDIOC_DQBUF calls will fail.
     *
     * When use of the read() function has been negotiated and the driver
     * does not capture yet, the select() function starts capturing. When
     * that fails, select() returns successful and a subsequent read() call,
     * which also attempts to start capturing, will return an appropriate
     * error code. When the driver captures continuously (as opposed to, for
     * example, still images) and data is already available the select()
     * function returns immediately.  */

    r = select(fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
      if (EINTR == errno)
	return -1;
      errno_exit("select");
    }

    if (0 == r) {
      std::cout << "Timeout :(" << std::endl;
      errorMsgStream << "Image capture on device " << fDevName << " timed out." << std::endl;
      errorMsgArray.push_back(errorMsgStream.str());
      errorMsgStream.str("");
      return -1;
    }

    return read_frame();
  }


  int WebCam::read_frame(void)
  {
    struct v4l2_buffer buf;
    unsigned int i;
    int process_img_result=0;

    switch (io) {
    case IO_METHOD_READ:
      if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
	switch (errno) {
	case EAGAIN:
	  return 0;

	case EIO:
	  /* Could ignore EIO, see spec. */

	  /* fall through */

	default:
	  errno_exit("read");
	}
      }

      process_img_result=process_image(buffers[0].start, buffers[0].length);
      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
	switch (errno) {
	case EAGAIN:
	  return 0;

	case EIO:
	  /* Could ignore EIO, see spec. */

	  /* fall through */

	default:
	  std::cout << "DQBuf error" << std::endl;
	  errno_exit("VIDIOC_DQBUF");
	}
      }

      assert(buf.index < n_buffers);

      process_img_result=process_image(buffers[buf.index].start, buf.bytesused);

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
	errno_exit("VIDIOC_QBUF");
      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
	switch (errno) {
	case EAGAIN:
	  return 0;

	case EIO:
	  /* Could ignore EIO, see spec. */

	  /* fall through */

	default:
	  errno_exit("VIDIOC_DQBUF");
	}
      }

      for (i = 0; i < n_buffers; ++i)
	if (buf.m.userptr == (unsigned long)buffers[i].start
	    && buf.length == buffers[i].length)
	  break;

      assert(i < n_buffers);

      process_img_result=process_image((void *)buf.m.userptr, buf.bytesused);

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
	errno_exit("VIDIOC_QBUF");
      break;
    }

    return process_img_result;
  }




int WebCam::process_image(const void * src, int len)
{
  // at the moment we just copy the bytes from the input to output image
  if(imgData.GetBufferSize() >= len) {
    memcpy(imgData.data, (unsigned char*)src, len);
    imgData.imgSizeInByte=len;
  }
  else {
    errorMsgStream << "Buffer size smaller then image size: " << imgData.GetBufferSize() << " <-> " << len << std::endl;
    errorMsgArray.push_back(errorMsgStream.str());
    errorMsgStream.str("");
    return -1;
  }

  // Format conversions are performed on the receiving site
  // Code kept for reference.
  // if (pixelformat_ == V4L2_PIX_FMT_YUYV)
  // {
  //   if (monochrome_)
  //   { //actually format V4L2_PIX_FMT_Y16, but xioctl gets unhappy if you don't use the advertised type (yuyv)
  //     mono102mono8((char*)src, dest->image, dest->width * dest->height);
  //   }
  //   else
  //   {
  //     yuyv2rgb((char*)src, dest->image, dest->width * dest->height);
  //   }
  // }
  // else if (pixelformat_ == V4L2_PIX_FMT_UYVY)
  //   uyvy2rgb((char*)src, dest->image, dest->width * dest->height);
  // else if (pixelformat_ == V4L2_PIX_FMT_MJPEG)
  //   mjpeg2rgb((char*)src, len, dest->image, dest->width * dest->height);
  // else if (pixelformat_ == V4L2_PIX_FMT_RGB24)
  //   rgb242rgb((char*)src, dest->image, dest->width * dest->height);
  // else if (pixelformat_ == V4L2_PIX_FMT_GREY)
  //   memcpy(dest->image, (char*)src, dest->width * dest->height);

  return 0;
}


const double WebCam::CalcTimeDiff(timespec start, timespec end)
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


// To debug this class stand alone
//
// int main(void)
// {
//   web_cam::WebCam *cam= new web_cam::WebCam;
//   if(cam->StartDevice("/dev/video0", 2350, 1536)==0) {
//     sleep(1);
//     const web_cam::ImageData *img=cam->GetImage();
//     if(img==NULL) {
//       std::vector<std::string> errorMsgs=cam->GetErrorMessages();
//       for (unsigned i=0; i<errorMsgs.size(); i++)
// 	std::cout << "Error: " << errorMsgs[i] << std::endl;
//     }
//     else {
//       std::cout << img->imgSizeInByte << std::endl;
//       std::ofstream outfile ("outfile.jpg",std::ofstream::binary);
//       //     outfile.write(img->data, img->imgSizeInByte);

//       //      char *buffer=new char[img->width*img->height*3];
//       // yuyv2rgb(img->data, buffer, img->width*img->height);
//       outfile.write(img->data, img->imgSizeInByte);
//     }
//   }
//   else {
//     std::vector<std::string> errorMsgs=cam->GetErrorMessages();
//     for (unsigned i=0; i<errorMsgs.size(); i++)
//       std::cout << "Error: " << errorMsgs[i] << std::endl;
//   }
// }
