#include <ros/ros.h>
#include <egomo_webcam/web_cam.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <string>
#include <std_srvs/Empty.h>

#include <boost/thread/thread.hpp>

#include "egomo_webcam/SetV4lParameter.h"
#include "egomo_webcam/GetNewImage.h"
#include "egomo_webcam/SetCameraFocus.h"


namespace egomo_webcam {

  class WebCamNode {
  public:
    WebCamNode();
    ~WebCamNode();

    void SetFrameRate(int rate) {frameRate=rate;};
    bool SetCameraFocus(SetCameraFocusRequest &req, SetCameraFocusResponse &res);
    bool RecordSendImage(GetNewImageRequest& req, GetNewImageResponse& res);
    bool Spin(void);
    void TryReconnect();

  private:
    bool UpdateStreamImage();
    void AdvertiseService();
    void ConnectCb();
    void ConnectCbRaw();

    // private ROS node handle
    ros::NodeHandle nodeHandle;
    ros::NodeHandle reconfigServerHandle;

    ros::ServiceServer serverGetNewImage;
    ros::ServiceServer serverSetCameraFocus;
    ros::Publisher serverImgStream;
    ros::Publisher serverImgStreamRaw;

    unsigned long seqNum;
    unsigned long seqNumStream;
    unsigned long seqNumStreamRaw;

    WebCam camDriver;

    int frameRate;
    sensor_msgs::CompressedImage streamImg;
    sensor_msgs::Image streamImgRaw;
    boost::mutex connectionMutex;
    int nSubscribers;
    int nSubscribersRaw;
  };
}
