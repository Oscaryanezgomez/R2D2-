#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;

int main(int argc, char **argv)
{
 cout<<"inicializando webcam_man_pub node .."<<endl;
 ros::init(argc, argv, "webcam_man_pub");
 ros::NodeHandle n;
 ros::Rate rate(30);
 
image_transport::ImageTransport it(n);
image_transport::Publisher pubImage = it.advertise("/webcam_image",1);

cv::VideoCapture cap;
cv::Mat frame;
cap.open(0);

 while(ros::ok())
 {
   cap.read(frame);
   
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
   
   pubImage.publish(msg);
   
   ros::spinOnce();
 }
 
 return 0;
}
