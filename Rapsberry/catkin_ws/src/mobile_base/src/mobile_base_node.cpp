#include <ros/ros.h> 
//Cabeceras para mesajes estandar 
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
//Cabeceras para el estado de los joints
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/JointState.h>
// Cabeceras para realizar las transformaciones 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define TICKS_PER_METER 4442.0

#define BASE_WIDTH 0.0895

long currEncoderLeft=0; 
long currEncoderRight=0; 
long lastEncoderLeft=0;
long lastEncoderRight=0; 
double robotX=0.0, robotY=0.0, robotT=0.0;

ros::Publisher pubSpeedLeft;
ros::Publisher pubSpeedRight;

//Este objeto nos sirve para realizar transformaciones entre frames 
tf::TransformListener* tf_listener;
//Funciones para los encoder
void callbackEncoder1(const std_msgs::Int64::ConstPtr &msg){
//std::cout << msg->data << std::endl;
currEncoderRight= -msg->data;
}
void callbackEncoder2(const std_msgs::Int64::ConstPtr &msg){
//std::cout << msg->data << std::endl;
currEncoderLeft= msg->data;
 }

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr & msg)
{
	std::cout<< "Reciving command robot velocity" << std ::endl;
	float rightSpeed= msg->linear.x -msg-> angular.z *BASE_WIDTH/ 2.0;
	float leftSpeed= msg->linear.x + msg-> angular.z *BASE_WIDTH/ 2.0;
	std_msgs:: Float32 msgSend; 
	msgSend.data=leftSpeed;
	pubSpeedLeft.publish(msgSend);
	msgSend.data= -rightSpeed;
	pubSpeedRight.publish(msgSend);

}

float normalizeAngle(float  angle)
{
	while(angle > M_PI)
		angle -=2 * M_PI;
	while (angle < -M_PI)
		angle +=2 *M_PI;
	return angle; 

}

void computeOdom()
{
	long leftTicks = currEncoderLeft - lastEncoderLeft; 
	long rightTicks = currEncoderRight - lastEncoderRight; 
	lastEncoderLeft = currEncoderLeft;
	lastEncoderRight = currEncoderRight;

	double distLeft= leftTicks/ TICKS_PER_METER;
	double distRight= rightTicks / TICKS_PER_METER;

	double deltaTheta = (distRight - distLeft)/BASE_WIDTH; 
	double distX= (distLeft + distRight)/ 2.0;

	robotT = normalizeAngle(robotT + deltaTheta);
	robotX += distX * cos(robotT);
	robotY += distX * sin(robotT);
}
void publishOdom(float robotX, float robotY, float roboT){
}

int main(int argc, char ** argv){

//Inicializacion de nodo 

ros::init(argc,argv, "mobile_base_node");
//se cre el mando del nodo 
ros::NodeHandle nh; 
//Se crea el objerto qwue nos permite realizr el muestreo de los mensajes 
ros::Rate rate (30);

//Se crean los subscritores de los mensajes 

ros::Subscriber subEncoder1= nh.subscribe("/encoder_1",1,callbackEncoder1);
ros::Subscriber subEncoder2= nh.subscribe("/encoder_2",1,callbackEncoder2);
ros::Subscriber subCmdVel= nh.subscribe("/cmd_vel",1, callbackCmdVel);
//Publicador del estado de los JOints  

ros::Publisher pubJointState = nh.advertise<sensor_msgs::JointState>("/joint_states",1);

pubSpeedLeft=nh.advertise<std_msgs::Float32>("/speed_motor_1",1);
pubSpeedRight=nh.advertise<std_msgs::Float32>("/speed_motor_2",1);

//se instancia el objeto parea realizar transformaciones 
tf_listener= new tf::TransformListener();

//Nombre de los Joints 

std::string jointNames[2] ={"left_wheel_joint_connect", "right_wheel_joint_connect"};
float jointPositions[2] = {0.0,0.0};
sensor_msgs::JointState jointState;
// Se asignan los nombre de los joints y las dimensiones 

jointState.name.insert(jointState.name.begin(),jointNames, jointNames +2);
jointState.position.insert(jointState.position.begin(),jointPositions,jointPositions +2);

tf::TransformBroadcaster br;

// loop de ros 

while (ros::ok()){
	computeOdom();

//transformaciones
tf::Transform transform;
transform.setOrigin(tf::Vector3(robotX,robotY,0));
tf:: Quaternion q;
q.setRPY(0,0,robotT);
transform.setRotation(q);
// se envia las transformaciones del baje link al do 
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

//Se publican el estado de los joits 

pubJointState.publish(jointState);
rate.sleep();
ros::spinOnce();
}
}


 
 
