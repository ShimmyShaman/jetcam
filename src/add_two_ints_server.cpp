#include "ros/ros.h"
#include "jetcam/AddTwoInts.h"

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-utils/gstCamera.h>

// Camera Stuff
gstCamera *camera = NULL;
imageConverter *image_cvt = NULL;
videoOptions video_options;

bool add(jetcam::AddTwoInts::Request &req,
         jetcam::AddTwoInts::Response &res)
{
  // ROS_INFO("request: x=%i, y=%i", (int)req.a, (int)req.b);

  std::string resource_str = "csi://0";
  std::string codec_str = "";

  int video_width = 1920;  //video_options.width;
  int video_height = 1080; //video_options.height;
  if (resource_str.size() == 0)
  {
    ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
    return 0;
  }

  ROS_INFO("opening video source: %s", resource_str.c_str());

  /*
	 * open video source
	 */
  camera = gstCamera::Create(video_width, video_height);

  if (!camera)
  {
    ROS_ERROR("failed to open video source");
    return 0;
  }

  if (!image_cvt)
  {
    ROS_ERROR("failed to create imageConverter");
    return 0;
  }

  /*
	 * start the camera streaming
	 */
  if (!camera->Open())
  {
    ROS_ERROR("failed to start streaming video source");
    return 0;
  }

  imageConverter::PixelType *capture = NULL;
  if (!camera->Capture(&capture))
  {
    ROS_ERROR("failed to capture image");
    return false;
  }

  ROS_INFO("this capture is %zu", sizeof(capture));

  // Ensure correct image size
  if (!image_cvt->Resize(camera->GetWidth(), camera->GetHeight(), imageConverter::ROSOutputFormat))
  {
    ROS_ERROR("failed to resize camera image converter");

    camera->Close();
    delete camera;
    return false;
  }

  sensor_msgs::Image msg;
  if (!image_cvt->Convert(msg, imageConverter::ROSOutputFormat, capture))
  {
    ROS_ERROR("failed to convert image");

    camera->Close();
    delete camera;
    return false;
  }

  camera->Close();

  delete camera;

  // Whatever
  // res.img = &msg;
  // ROS_INFO("sending back response: [%p]", res.img);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  // Create image converter
  image_cvt = new imageConverter();
  
  // image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("/cameras/head")
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  // Free resources
  delete image_cvt;

  return 0;
}
