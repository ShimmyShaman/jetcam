#include <ros/ros.h>
#include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-utils/gstCamera.h>

bool initImageConverter(imageConverter **image_cvt)
{
  *image_cvt = new imageConverter();

  if (!image_cvt)
  {
    ROS_ERROR("failed to create imageConverter");
    return false;
  }
  return true;
}

bool createAndOpenCamera(gstCamera **pCamera, imageConverter *image_cvt)
{

  std::string resource_str = "csi://0";
  std::string codec_str = "";

  int video_width = 1920;  //video_options.width;
  int video_height = 1080; //video_options.height;
  if (resource_str.size() == 0)
  {
    ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
    return false;
  }

  ROS_INFO("opening video source: %s", resource_str.c_str());

  /*
	 * open video source
	 */
  gstCamera *camera = gstCamera::Create(video_width, video_height);

  if (!camera)
  {
    ROS_ERROR("failed to open video source");
    return false;
  }

  /*
	 * start the camera streaming
  image_transport
	 */
  if (!camera->Open())
  {
    ROS_ERROR("failed to start streaming video source");
    delete camera;
    return false;
  }

  // Ensure correct CPU image size
  if (!image_cvt->Resize(camera->GetWidth(), camera->GetHeight(), imageConverter::ROSOutputFormat))
  {
    ROS_ERROR("failed to resize camera image converter");

    camera->Close();
    delete camera;
    return false;
  }

  ROS_INFO("Camera Opened");
  *pCamera = camera;

  return true;
}

bool captureImage(gstCamera *camera, imageConverter *image_cvt, sensor_msgs::Image *img)
{

  imageConverter::PixelType *capture = NULL;
  if (!camera->Capture(&capture))
  {
    ROS_ERROR("failed to capture image");
    return false;
  }

  // Convert the image to ROS output
  if (!image_cvt->Convert(*img, imageConverter::ROSOutputFormat, capture))
  {
    ROS_ERROR("failed to convert image");
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  // Initialize the image converter
  imageConverter *image_cvt;
  if (!initImageConverter(&image_cvt))
  {
    return 1;
  }

  // Initialize and open the camera
  gstCamera *camera;
  if (!createAndOpenCamera(&camera, image_cvt))
  {
    delete image_cvt;
    return 2;
  }

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("jetcam/image", 1);

  sensor_msgs::ImagePtr msg = sensor_msgs::ImagePtr(new sensor_msgs::Image());

  ros::Rate loop_rate(0.25);
  while (nh.ok())
  {
    if (!captureImage(camera, image_cvt, msg.get()))
    {
      break;
    }

    ROS_INFO("Image Published");
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Cleanup
  ROS_INFO("Closing Camera and deleting resources");
  camera->Close();
  delete camera;
  delete image_cvt;

  return 0;
}
