// vservo.cpp: Visual servoing via cmvision
// Author: Andrew Melim
// Date: March 2, 2013

//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <sstream>
#include <cmath>
#include <numeric>

#define DEBUG 1

enum Color {YELLOW, CYAN, MAGENTA};
int currentColor = YELLOW;

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

struct color{

  vector<double> h;
  vector<double> s;
  vector<double> v;

  int mean_h;
  int mean_s;
  int mean_v;

  int std_h;
  int std_s;
  int std_v;

}yellow, cyan, magenta;

int mouseClicks = 0;
int samples = 20;

void onMouse(int event, int x, int y, int, void* param)
{
  
  if(event != CV_EVENT_LBUTTONDOWN)
    return;
  Mat image, hsv;
  image = *((Mat*) param);
  cv::cvtColor(image, hsv, CV_BGR2HSV);

  //ROS_INFO("Mouse click at u:%i v:%i, with image W:%i H:%i", x, y, hsv.cols, hsv.rows);
  Vec3b pixel = hsv.at<Vec3b>(y,x);
  //ROS_INFO("H:%d S:%d V:%d", pixel[0], pixel[1], pixel[2]);

  if(currentColor == YELLOW){
    yellow.h.push_back(pixel[0]);
    yellow.s.push_back(pixel[1]);
    yellow.v.push_back(pixel[2]);
    int h_sum=std::accumulate(yellow.h.begin(), yellow.h.end(), 0.0);
    int v_sum=std::accumulate(yellow.s.begin(), yellow.s.end(), 0.0);
    int s_sum=std::accumulate(yellow.v.begin(), yellow.v.end(), 0.0);

    yellow.mean_h = h_sum/yellow.h.size();
    yellow.mean_s = s_sum/yellow.s.size();
    yellow.mean_v = v_sum/yellow.v.size();

    ROS_INFO("YELLOW H mean:%i S mean:%i V mean:%i", yellow.mean_h, yellow.mean_s, yellow.mean_v);
  }
  else if(currentColor == CYAN){
    cyan.h.push_back(pixel[0]);
    cyan.s.push_back(pixel[1]);
    cyan.v.push_back(pixel[2]);
    int h_sum=std::accumulate(cyan.h.begin(), cyan.h.end(), 0.0);
    int v_sum=std::accumulate(cyan.s.begin(), cyan.s.end(), 0.0);
    int s_sum=std::accumulate(cyan.v.begin(), cyan.v.end(), 0.0);

    cyan.mean_h = h_sum/cyan.h.size();
    cyan.mean_s = s_sum/cyan.s.size();
    cyan.mean_v = v_sum/cyan.v.size();
    ROS_INFO("CYAN H mean:%i S mean:%i V mean:%i", cyan.mean_h, cyan.mean_s, cyan.mean_v);
  }
  else{
    magenta.h.push_back(pixel[0]);
    magenta.s.push_back(pixel[1]);
    magenta.v.push_back(pixel[2]);
    int h_sum=std::accumulate(magenta.h.begin(), magenta.h.end(), 0.0);
    int v_sum=std::accumulate(magenta.s.begin(), magenta.s.end(), 0.0);
    int s_sum=std::accumulate(magenta.v.begin(), magenta.v.end(), 0.0);

    magenta.mean_h = h_sum/magenta.h.size();
    magenta.mean_s = s_sum/magenta.s.size();
    magenta.mean_v = v_sum/magenta.v.size();
    ROS_INFO("MAGENTA H mean:%i S mean:%i V mean:%i", magenta.mean_h, magenta.mean_s, magenta.mean_v);
  }
}

void colorCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Mat img = cv_ptr->image;
  setMouseCallback("Color Picker", onMouse, &img);
  cv::imshow("Color Picker", img);
  char k = cv::waitKey(-1);
  if(k=='m'){
    currentColor = MAGENTA;
    ROS_INFO("Changing color to MAGANETA");
  }
  if(k=='c'){
    currentColor = CYAN;
    ROS_INFO("Changing color to CYAN");
  }
  if(k=='y'){
    currentColor = YELLOW;
    ROS_INFO("Changing color to YELLOW");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_picker");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 1, colorCallback);
  namedWindow("Color Picker");
  ros::spin();
  return 0;
}
