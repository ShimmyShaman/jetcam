#include "ros/ros.h"
#include "jetcam/AddTwoInts.h"
#include <cstdlib>

void SaveImageAsPPM(const sensor_msgs::Image *msg, const char *filename)
{
  if (msg->encoding != "bgr8")
  {
    ROS_INFO("SaveImageAsPPM: encoding=%s NOT bgr8", msg->encoding.c_str());
    return; // Can only handle the rgb8 encoding
  }

  ROS_INFO("opening file: %s", filename);
  FILE *file = fopen(filename, "w");
  if (!file)
  {
    ROS_INFO("Could not open file");
    return;
  }

  ROS_INFO("a");
  fprintf(file, "P3\n");
  ROS_INFO("b");
  fprintf(file, "%i %i\n", msg->width, msg->height);
  fprintf(file, "255\n");

  ROS_INFO("c");
  for (uint32_t y = 0; y < msg->height; y++)
  {
    for (uint32_t x = 0; x < msg->width; x++)
    {
      // Get indices for the pixel components
      uint32_t blueByteIdx = y * msg->step + 3 * x;
      uint32_t greenByteIdx = blueByteIdx + 1;
      uint32_t redByteIdx = blueByteIdx + 2;

      fprintf(file, "%i %i %i ",
              msg->data[blueByteIdx],
              msg->data[greenByteIdx],
              msg->data[redByteIdx]);
    }
    fprintf(file, "\n");
  }

  ROS_INFO("closing file");

  fclose(file);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<jetcam::AddTwoInts>("add_two_ints");
  jetcam::AddTwoInts srv;
  // srv.request.a = atoll(argv[1]);
  // srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Img: %p", srv.response.img);

    // SaveImageAsPPM(&msg, "/home/boo/proj/roscol/cap.ppm");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
