#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

#include <std_msgs/Int16.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
int S,H,V;
int SL=256;//Valor mínimo
int SU=0;//Valor máximo
int HL=256;
int HU=0;
int VL=256;
int VU=0;
int contour_index;
ros::Publisher pubpunto; //Arreglo que contiene los 2 puntos
ros::Publisher pubpuntox; //Arreglo que contiene el punto x
ros::Publisher pubpuntoy; //Arreglo que contiene el punto y 
ros::Publisher pubSpeedLeft;


cv::Mat mask;
cv::Mat img_rgb,todo;
cv::Mat img_hsv;
cv::Mat contours;
cv::Mat gray_image;
std::vector<vector <cv::Point> > contornos;
std::vector<cv::Vec4i> hierarchy;
cv::Mat imageErode;

void onMouse( int event, int x, int y, int, void* param ){
	
         cv::Mat* hsv = (cv::Mat*)param;

	if(event == CV_EVENT_LBUTTONDOWN){
		printf("%d %d %d\n",
		(int)(*hsv).at<cv::Vec3b>(y, x)[0],//color azul
		(int)(*hsv).at<cv::Vec3b>(y, x)[1],//color verde
		(int)(*hsv).at<cv::Vec3b>(y, x)[2]);//color rojo

		H=(int)(*hsv).at<cv::Vec3b>(y, x)[0];
		S=(int)(*hsv).at<cv::Vec3b>(y, x)[1];
		V=(int)(*hsv).at<cv::Vec3b>(y, x)[2];

		if(H > HU) {
			HU=H;
		}
		if(H < HL){
			HL=H;
		}

		if(S > SU) {
			SU=S;
		}
		if(S < SL){
			SL=S;
		}

		if(V > VU){
			VU=V;
		}
		if(V < VL){
			VL=V;
		}

             }
}

void callbackImage(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr= cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  cv::Mat frame = cv_ptr->image;
  //cv::Mat grayImage;
  //cv::cvtColor(frame, grayImage,cv::COLOR_BGR2GRAY);
  cv::resize(frame,frame, cv::Size(320, 240));	
  cvtColor(frame,img_hsv,CV_BGR2HSV);
 // cv::imshow("hsv",img_hsv);
  cv::imshow("Frame",frame); 
  cv::setMouseCallback("Frame", onMouse, &img_hsv);  //

 
  cv::inRange(img_hsv,cv::Scalar(HL,SL,VL),cv::Scalar(HU,SU,VU),mask);
 // cv::imshow("Mask",mask);
 
  int dilatationSize = 4;
  cv::Mat element1 = getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2*dilatationSize+1,2*dilatationSize+1),cv::Point    (dilatationSize,dilatationSize));
  cv::Mat imageDilatation;
  dilate(mask,imageDilatation,element1);
   cv::namedWindow("Erosion");
 // imshow("Erosion",imageDilatation);

  int erosionSize = 6;
  cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2*erosionSize+1,2*erosionSize+1),cv::Point(erosionSize,erosionSize));
  erode(imageDilatation,imageErode,element);
  //cv::namedWindow("Erosion");
  imshow("Erosion",imageErode);
  cv::Canny(imageErode,contours,100,20);//le pasamos la imagen en HSV y en la imagen contours aplicamos los contornos
  cv::findContours(contours,contornos,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point(0,0));
  cv::drawContours(frame,contornos,contour_index,cv::Scalar(255,0,0),2,5,hierarchy,0,cv::Point(0,0));//dibuja en la imagen original
  float sumx=0, sumy=0;
  float num_pixel = 0;
  for(int x=0; x<imageErode.cols; x++) {
   for(int y=0; y<imageErode.rows; y++) {
   int val = imageErode.at<uchar>(y,x);
   if( val >= 50) {
       sumx += x;
       sumy += y;
       num_pixel++;
	          }
       }
   }
  cv::Point p(sumx/num_pixel, sumy/num_pixel);
  cv::Moments m = moments(imageErode, false);
  cv::Point p1(m.m10/m.m00, m.m01/m.m00);
  cv::circle(frame, p, 5, cv::Scalar(128,0,0), -1);
  
  std_msgs::Int16MultiArray centroid_msg;
  centroid_msg.data.resize(2);
  centroid_msg.data[0]=sumx/num_pixel;
  centroid_msg.data[1]=sumy/num_pixel;
  pubpunto.publish(centroid_msg);
  
  std_msgs::Int16 puntox_msg;
  puntox_msg.data=sumx/num_pixel;
  pubpuntox.publish(puntox_msg);

  std_msgs::Int16 puntoy_msg;
  puntoy_msg.data=sumy/num_pixel;
  pubpuntoy.publish(puntoy_msg);

  
  
  cout<<"\n El centroide es ";
  cout<<sumx/num_pixel;
  cout<<endl;
  //cv::imshow("Cany",contours);
  cv::imshow("Frame",frame);


 
 cv::waitKey(1);

 

}

int main(int argc, char **argv)
{
 cout<<"inicializando webcam_man_listener node .."<<endl;
 ros::init(argc, argv, "webcam_man_listener");
 ros::NodeHandle n;
 ros::NodeHandle nh;

 ros::Subscriber subImage = n.subscribe("/webcam_image",1,callbackImage);
 pubpunto = n.advertise<std_msgs::Int16MultiArray>("/Centroid",1000);
 pubpuntox = n.advertise<std_msgs::Int16>("/Puntox",1000);
 pubpuntoy = n.advertise<std_msgs::Int16>("/Puntoy",1000);

 pubSpeedLeft=nh.advertise<std_msgs::Float32>("/speed_motor_1",1);

 ros::Rate loop (50);

 while(ros::ok())
 {
   ros::spinOnce();
   loop.sleep();
 }


 return 0;
}
