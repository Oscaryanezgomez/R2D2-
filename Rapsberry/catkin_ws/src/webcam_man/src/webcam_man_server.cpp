#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <webcam_man/getImage.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32.h>

using namespace std;
ros::Publisher pubSpeedLeft;

cv::Mat frame;

bool callbackImage(webcam_man::getImage::Request &req, webcam_man::getImage::Response &res)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();    
    res.imageSrv = *msg;
   
    return true;
}

int main(int argc, char **argv)
{
    cout<<"Starting webcam_man_server by Luis Näva..."<<endl;
    ros::init(argc, argv, "webcam_man_server");
    ros::NodeHandle nh;
   
    ros::ServiceServer servImage = nh.advertiseService("/webcam_image", callbackImage);
    //ros::Publisher pubImage = nh.advertise<sensor_msgs::Image>("/webcam_image", 1);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pubImage = it.advertise("/webcam_image",1);    
    ros::Rate loop(50);

    cv::VideoCapture capture;
    capture.open(0);
    pubSpeedLeft=nh.advertise<std_msgs::Float32>("/speed_motor_1",1);

    while( ros::ok())
    {
        capture.read(frame);
	cv::resize(frame,frame, cv::Size(320, 240));
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	pubImage.publish(msg);
	// cv::imshow("Hola",frame);
	
        ros::spinOnce();
        loop.sleep();
	//cv::waitKey(1);
    }

    return 0;
}
