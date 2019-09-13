#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>

void subEncoderCallback(const std_msgs::Int64::ConstPtr &msg){
std::cout << msg->data << std::endl;
}
int main(int argc, char ** argv){
ros::init (argc, argv, "carrito_node");
ros::NodeHandle nh;
ros::Rate rate (20);
float i=0.0;
//se agrego
ros::Publisher pubVelMotor = nh.advertise<std_msgs::Float32>("/speed_motor_1",1);
ros::Publisher pubVelMotor2 = nh.advertise<std_msgs::Float32>("/speed_motor_2",1);
ros::Subscriber subEncoder1 = nh.subscribe("/encoder_1",1, subEncoderCallback);
 while(ros::ok())
{ 
std_msgs::Float32 msg;
msg.data = i;
pubVelMotor.publish(msg);
pubVelMotor.publish(msg);
i+=0.1;

ros::spinOnce();
rate.sleep();
}

return 1;

}
