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

#define DEBUG 1
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

bool first = true;
ros::Publisher feature_pub;
RNG rng(12345);

geometry_msgs::Pose getFeature(Mat img, Mat dst, Scalar low, Scalar high, Scalar draw)
{
  // Threshold image
  Mat bw, hsv;
  cv::cvtColor(img, hsv, CV_BGR2HSV);
  vector<vector<Point> > contours;

  inRange(hsv, low, high, bw);
  findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  for(size_t i=0; i < contours.size(); i++)
  {
    double area = contourArea(contours[i]);
    // Ensure minimum area and symmetry for circles
    if(area > 200){
      RotatedRect minEllipse;
      minEllipse = fitEllipse(Mat(contours[i]));
      double eW = minEllipse.boundingRect().width;
      double eH = minEllipse.boundingRect().height;
      if(abs(eW-eH)<6){

        ellipse(dst, minEllipse, draw, 2, 8);

        cv::imshow("Range window", dst);
        cv::imshow("HSV window", hsv);
        geometry_msgs::Point pt;
        geometry_msgs::Quaternion q;
        pt.x = minEllipse.center.y;
        pt.y = minEllipse.center.x;
        pt.z = 0.0;
        geometry_msgs::Pose p;
        p.position = pt;
        p.orientation = q;
        return p;
      }
    }
  }
  cv::imshow("Range window", dst);
  cv::imshow("HSV window", hsv);
  return geometry_msgs::Pose();
}

void houghCallback(const sensor_msgs::ImageConstPtr& msg)
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
  Mat img_gray, img_can, video, hsv, bw;
  Mat yellow, blue, magenta;
  Mat dst = Mat::zeros(img.size(), img.type());

  geometry_msgs::PoseArray featureArray;
  featureArray.poses[3];

  geometry_msgs::Pose py = getFeature(img, dst, Scalar(22,100,90), Scalar(30,240,240), Scalar(0,255,255));
  geometry_msgs::Pose pb = getFeature(img, dst, Scalar(95,100,100), Scalar(130,190,230), Scalar(255,255,0));
  geometry_msgs::Pose pm = getFeature(img, dst, Scalar(160,80,110), Scalar(190,220,240), Scalar(255,0,255));

  if((pm.position.x !=0) && (pm.position.y !=0))
    featureArray.poses.push_back(pm);
  if((py.position.x !=0) && (py.position.y !=0))
    featureArray.poses.push_back(py);
  if((pb.position.x !=0) && (pb.position.y !=0))
    featureArray.poses.push_back(pb);

  cv::imshow("Hough window", img);

  feature_pub.publish(featureArray);
  hconcat(img,img,video);
  cv::waitKey(3);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vservo");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  // Second param is size of message queue, how much to buffer before trashing
  image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 1, houghCallback);
  feature_pub = n.advertise<geometry_msgs::PoseArray>("features", 1000);
  ros::spin();
  return 0;
}
