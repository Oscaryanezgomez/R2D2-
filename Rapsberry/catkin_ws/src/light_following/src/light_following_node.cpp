#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

using namespace std;
int valorx;//Punto x del centroide del color 1 
int valory;//Punto y del centroide del color 1 
int valorx1;// Punto x del centroide del color 2
int valory1;// Punto y del centroide del color 2 
int sharp1;// Valor del sensor de distancia izquierdo
int sharp2;// Valor del sensor de distancia derecho
int sharp3;// Valor del sensor de distancia del centro
int cambio; 

ros::Publisher pubSpeedLeft; //Publica la velocidad al motor Izquierda
ros::Publisher pubSpeedRight; // Publica la velocidad del motor Decho 

void adelante() //Funcion para que el robot se mueva adelante
{
        std_msgs::Float32 speedleftmsg;
	speedleftmsg.data=0.5;
	pubSpeedLeft.publish(speedleftmsg);

	std_msgs::Float32 speedrightmsg;
	speedrightmsg.data=1;
	pubSpeedRight.publish(speedrightmsg);

}

void atras()//Funcion para que el robot se mueve atras 
{
	std_msgs::Float32 speedleftmsg;
	speedleftmsg.data=-0.5;
	pubSpeedLeft.publish(speedleftmsg);

	std_msgs::Float32 speedrightmsg;
	speedrightmsg.data=-1;
	pubSpeedRight.publish(speedrightmsg);
}

void paro()// Funcion que para el robot 
{
        std_msgs::Float32 speedleftmsg;
	speedleftmsg.data=0;
	pubSpeedLeft.publish(speedleftmsg);

	std_msgs::Float32 speedrightmsg;
	speedrightmsg.data=0;
	pubSpeedRight.publish(speedrightmsg);
}

void derecha()// Funcion para que el robot valla a la derecha 
{
        
	std_msgs::Float32 speedleftmsg;
	speedleftmsg.data=-0.5;
	pubSpeedLeft.publish(speedleftmsg);

	std_msgs::Float32 speedrightmsg;
	speedrightmsg.data=1;
	pubSpeedRight.publish(speedrightmsg);

}

void izquierda()//FUncion para que el robot valla a la izquierda
{
        std_msgs::Float32 speedleftmsg;
	speedleftmsg.data=0.5;
	pubSpeedLeft.publish(speedleftmsg);

	std_msgs::Float32 speedrightmsg;
	speedrightmsg.data=-1;
	pubSpeedRight.publish(speedrightmsg);

}

void sharp1Callback(const std_msgs::Int16::ConstPtr& msg)//obtenemos el valor del sensor de distancia izquierdo y lo imprimimos
{
	sharp1=msg->data;
	cout<<sharp1<<endl;
}

void sharp2Callback(const std_msgs::Int16::ConstPtr& msg)//obtenemos el valor del censor de distancia derecho y lo imprimimos
{
	sharp2=msg->data;
	cout<<sharp2<<endl;
}

void sharp3Callback(const std_msgs::Int16::ConstPtr& msg)//obtenemos el valor del sensor de distancia central y lo imprimimos
{
	sharp3=msg->data;
	cout<<sharp3<<endl;
}


void centroidCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)//obtenemos el valor del centroide 1
{
	valorx=msg->data[0];
	valory=msg->data[1];
        cout<<"El centroide del color 1 en x es:  " <<valorx<<" en y es " <<valory<<"\n"<<endl;
	
	
   /*     if(sharp3>=400 && sharp1<=200 && sharp2<=200)
	{
          atras();
	}	
        
	else if( sharp3<=400 && sharp1>=200 && sharp2<=200)
	{
          derecha();
	}

	else if (sharp3<=400 && sharp1<=200 && sharp2>=200)
	{
          izquierda();
	}
*/
	/*else*/ if(sharp3<=400 && sharp1<=200 && sharp2<=200)
	{
		if(valorx==0)
		{
	         paro();
		}
		
		else if(valorx>=0 && valorx<=106)
        	{
                 izquierda(); 
	        }
      
		else if(valorx>=106 && valorx<=212)
        	{
	         adelante();
          	}

		else  if(valorx>=212 && valorx<=320)
         	{
                 derecha();
         	}
        
        
	}

       if(valory==200)
       {
	 cambio=1;
       }	       
}


void centroid1Callback(const std_msgs::Int16MultiArray::ConstPtr& msg)//obtenemos el valor del centroide 2 
{

	valorx1=msg->data[0];
	valory1=msg->data[1];
	cout<<"El centroide del color 2 en x es: " <<valorx1<<" en y es: " <<valory1<< "\n" <<endl;
/*  if(cambio==1)
 {
        if(sharp3>=400 && sharp1<=200 && sharp2<=200)
	{
          atras();
	}	
        
	else if( sharp3<=400 && sharp1>=200 && sharp2<=200)
	{
          derecha();
	}

	else if (sharp3<=400 && sharp1<=200 && sharp2>=200)
	{
          izquierda();
	}

	else if(sharp3<=400 && sharp1<=200 && sharp2<=200)
	{
		if(valorx1==0)
		{
	         paro();
		}
		
		else if(valorx1>=0 && valorx1<=106)
        	{
                 izquierda(); 
	        }
      
		else if(valorx1>=106 && valorx1<=212)
        	{
	         adelante();
          	}

		else  if(valorx1>=212 && valorx1<=320)
         	{
                 derecha();
         	}
	}  
 }*/
}

int main(int argc,char **argv)
{
	cout<<"Comportamiento R2-D2 Seguidor de color"<<endl;
        ros::init(argc,argv,"Colormove");
	ros::NodeHandle nh;
        
	

        ros::Subscriber subSharp1=nh.subscribe("/sharpSensor_1",1000,sharp1Callback);
	ros::Subscriber subSharp2=nh.subscribe("/sharpSensor_2",1000,sharp2Callback);
	ros::Subscriber subSharp3=nh.subscribe("/sharpSensor_3",1000,sharp3Callback);
        ros::Subscriber subcentroid=nh.subscribe("/Centroid",1000,centroidCallback);
	ros::Subscriber subcentroid1=nh.subscribe("/Centroid1",1000,centroid1Callback);
	pubSpeedLeft=nh.advertise<std_msgs::Float32>("/speed_motor_1",1);
	pubSpeedRight=nh.advertise<std_msgs::Float32>("/speed_motor_2",1);

	

        while(ros::ok())
        {
        ros::spinOnce();
	}	

	return 0;
}
