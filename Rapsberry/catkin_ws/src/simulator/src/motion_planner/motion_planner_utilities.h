#include "ros/ros.h"

#include "simulator/Parameters.h"
#include "simulator/Laser_values.h"
#include "simulator/simulator_stop.h"
#include "simulator/simulator_robot_step.h"
#include "simulator/simulator_parameters.h"
#include "simulator/simulator_robot_laser_values.h"
#include "simulator/simulator_base.h"
#include "simulator/simulator_laser.h"
#include "simulator/simulator_light.h"
#include "simulator/simulator_algorithm_result.h"
#include <string.h>

movement generate_output(int out ,float advance ,float twist);
//parameters wait_start();
int move_gui(float angle ,float distance ,next_position *next );
//int laser_gui(float *lasers );
float check_collision(float theta ,float distance ,int new_simulation );
//void get_lidar_values(float *lasers );
int get_lidar_values(float *lasers);
int move_robot(float theta,float advance);
int get_light_values();

next_position next;
int new_simulation = 1;
//parameters params;



int quantize_light(float *light_values)
{
    int sensor = 0;

    //for(int i = 0; i < 8; i++){
        //printf("light sensor %d %f\n",i,light_values[i]);
    //}

    for(int i = 1; i < 8; i+=2){
        //printf("light sensor %d %f\n",i,light_values[i]);
        if( light_values[i] > light_values[sensor])
            sensor = i;
    }

    //printf("biggest value sensor %d %f\n",sensor,light_values[sensor]);


    if(sensor == 0)
        return 1;
    else if(sensor == 1)
        return 3;
    else if(sensor == 3)
        return 1;
    else if(sensor == 5)
        return 0;
    else if(sensor == 7)
        return 2;
    else
    	return 0;
}



// It quantizes the inputs
int quantize_inputs(float *observations, int size, float laser_value  ){

   int a,b;
   int iz,de,salida;
   int j;



   //for(j=0;j< size;j++)
	//printf("observations[%d] %f\n",j,observations[j]);



   iz = de = salida = 0;
   if( size % 2 != 0)
   {
        a = ( size - 1 ) / 2;
        b = a + 1;
   }else
   {
        a = b = size / 2;
   }

   for (int i = b; i < size ; i++ ) //izquierda
   {
        if( observations[i] < laser_value  )
         {
                iz = 2;
                break;
         }
   }

   for (int i = 0; i < a ; i++ ) //derecha
   {
        if( observations[i] < laser_value  )
         {
                de = 1;
                break;
         }
   }

   return iz+de ;
}





movement generate_output(int out ,float advance ,float twist)
{

  movement output;

  switch(out){

        case 0: // Stop
                output.advance = 0.0f;
                output.twist = 0.0f;
                //printf("STOP\n");
                break;

        case 1: // Forward
                output.advance=advance;
                output.twist=0.0f;
                //printf("FORWARD\n");
                break;

        case 2: // backward
                output.advance=-advance;
                output.twist=0.0f;
                //printf("BACKWARD\n");
                break;

        case 3:// Turn left
                output.advance=0.0f;
                output.twist=twist;
                //printf("LEFT\n");
                break;

          case 4: // Turn right
                output.advance=0.0f;
                output.twist=-twist;
                printf("RIGHT %f\n",output.twist);
                break;

        default:printf("Output %d not defined used ", out);
                output.advance=0.0f;
                output.twist=0.0f;
                break;
  }

  return(output);

}



int stop()
{
  
  ros::NodeHandle n;
  ros::ServiceClient client;
  simulator::simulator_stop srv;
  client = n.serviceClient<simulator::simulator_stop>("simulator_stop"); 
  srv.request.stop = true;

  if (client.call(srv))
  {     
      
  }
  else
  {
    ROS_ERROR("Failed to call service simulator_stop");
  }
  return 1;
}



int move_gui(float angle ,float distance ,next_position *next )
{
  ros::NodeHandle n;
  ros::ServiceClient client;
  simulator::simulator_robot_step srv;
  client = n.serviceClient<simulator::simulator_robot_step>("simulator_robot_step"); //create the client
  
  srv.request.theta=angle;
  srv.request.distance=distance;
  
  if (client.call(srv))
  {     
      next->robot_x = srv.response.robot_x;
      next->robot_y = srv.response.robot_y;
      next->robot_theta =srv.response.theta;
      //printf("movement gui %f %f %f\n",next->robot_x,next->robot_y,next->robot_theta);
  }
  else
  {
    ROS_ERROR("Failed to call service simulator_robot_step");
    
  }
  return 1;
}


ros::Subscriber* sub;
int prikaz=0;
float lasers_aux[100];

void laserCallback(const simulator::Laser_values::ConstPtr& msg)
{
    for(int i=0;i<100;i++)
      lasers_aux[i] = msg->sensors[i];

    //prikaz=0;
    //sub->shutdown(); 
    return;
}


int get_lidar_values(float *lasers, float robot_x ,float robot_y, float robot_theta)
{
  
  ros::NodeHandle n;
  ros::ServiceClient client;
  simulator::simulator_laser srv;
  client = n.serviceClient<simulator::simulator_laser>("simulator_laser_serv"); //create the client
  //char topic[]="simulator_laser_pub";
  //simulator::Laser_values msg = ros::topic::waitForMessage(topic);
  //sub = new ros::Subscriber;
  //*sub = n.subscribe("simulator_laser_pub", 1, laserCallback);
  //prikaz=1; 
  //while(ros::ok() && prikaz)
  //{
  //        ros::spinOnce();
  //}
  
  //for(int i=0;i<100;i++)
  //    lasers[i] = lasers_aux[i];
  srv.request.robot_x = robot_x;
  srv.request.robot_y = robot_y;
  srv.request.robot_theta = robot_theta; 

  if (client.call(srv))
  {     
      //printf("%s\n","done lasers" );
      for(int i=0;i<100;i++)
        lasers[i] = srv.response.sensors[i];
  }
  else
  {
    ROS_ERROR("Failed to call service simulator_robot_laser_values");
    
  }
  return 1;
}


float check_collision(float theta ,float distance ,int new_simulation )
{
  float final_distance=0;
  ros::NodeHandle n;
  ros::ServiceClient client;
  simulator::simulator_base srv;
  client = n.serviceClient<simulator::simulator_base>("simulator_base"); //create the client
  
  srv.request.x1 = next.robot_x;
  srv.request.y1 = next.robot_y;
  srv.request.theta = next.robot_theta+theta;
  srv.request.distance = distance;
  srv.request.new_simulation =new_simulation;
 
  if (client.call(srv))
  {
    final_distance = srv.response.distance;
  }
  else
  {
    ROS_ERROR("Failed to call service simulator_base");    
  }

  return final_distance;
}


int move_robot(float theta,float advance)
{
  float res;
  res = check_collision(theta ,advance ,new_simulation);
  move_gui(theta ,res ,&next);
  ros::spinOnce();
  return 1;
}



int get_light_values(float *intensity,float *values)
{
  ros::NodeHandle n;
  ros::ServiceClient client;
  simulator::simulator_light srv;
  client = n.serviceClient<simulator::simulator_light>("simulator_light"); //create the client
  srv.request.req=1;
  int sensor;
  int i;

 
  if (client.call(srv))
  {
      for(i=0;i<8;i++)
         values[i]=srv.response.values[i];

      sensor = 0;

      for(i = 1; i < 8; i++) {
                if( values[i] > values[sensor])
                        sensor = i;
      }
      //printf("biggest intensity sensor[%d] %f \n",sensor,values[sensor] );
      *intensity=values[sensor];	

  }
  else
  {
    ROS_ERROR("Failed to call service  simulator_light"); 
  } 
}



int print_algorithm_graph (step *steps )
{
  ros::NodeHandle n;
  ros::ServiceClient client;
  simulator::simulator_algorithm_result srv;
  client = n.serviceClient<simulator::simulator_algorithm_result>("simulator_print_graph"); //create the client
  
  for(int i=0;i<200;i++)
    srv.request.nodes_algorithm[i] = steps[i].node;

  if (client.call(srv))
  {          
      //printf("%s\n","Hecho" );
  }
  else
  {
    ROS_ERROR("Failed to call service simulator_print_graph");
    
  }
  return 1;
}
