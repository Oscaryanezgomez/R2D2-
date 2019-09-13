/***********************************************
*                                              *
*      motion_planner_node.cpp                 *
*                                              *
*      Jesus Savage                            *
*      Diego Cordero                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 17-2-2019                 *
*                                              *
*                                              *
************************************************/




#include "ros/ros.h"
#include "../utilities/simulator_structures.h"
#include "../utilities/random.h"
#include "motion_planner_utilities.h"
#include "../state_machines/light_follower.h"
#include "../state_machines/sm_avoidance.h"
#include "../state_machines/sm_avoidance_destination.h"
#include "../state_machines/sm_destination.h"
#include "../state_machines/user_sm.h"
#include "../state_machines/dijkstra.h"
#include "../state_machines/dfs.h"
#include "clips_ros/SimuladorRepresentation.h"
#include "../behaviors/oracle.h"


parameters params;

void parametersCallback(const simulator::Parameters::ConstPtr& paramss)
{
  params.robot_x             = paramss->robot_x   ;
  params.robot_y             = paramss->robot_y   ;
  params.robot_theta         = paramss->robot_theta   ;    
  params.robot_radio         = paramss->robot_radio   ;    
  params.robot_max_advance   = paramss->robot_max_advance   ;          
  params.robot_turn_angle    = paramss->robot_turn_angle   ;         
  params.laser_num_sensors   = paramss->laser_num_sensors   ;          
  params.laser_origin        = paramss->laser_origin         ;     
  params.laser_range         = paramss->laser_range   ;    
  params.laser_value         = paramss->laser_value   ;    
  strcpy(params.world_name ,paramss -> world_name.c_str());       
  params.noise               = paramss->noise   ;   
  params.run                 = paramss->run   ; 
  params.light_x             = paramss->light_x;
  params.light_y             = paramss->light_y;
  params.behavior            = paramss->behavior; 
  params.steps               = paramss->steps;

}




int main(int argc ,char **argv)
{  
  ros::init(argc ,argv ,"simulator_motion_planner_node");
  ros::NodeHandle n;
  ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub",0, parametersCallback);
  ros::Subscriber sub = n.subscribe("simulator_laser_pub", 0, laserCallback);

  SimuladorRepresentation::setNodeHandle(&n);


  float lidar_readings[100];
  float light_readings[8];
  step steps[200];
  step graph_steps[200];
  int sensor;
  int i;
  int flagOnce;
  int est_sig;
  int cta_steps;

  int q_light;
  int q_inputs;
  float max_advance;
  float max_turn_angle;
  int tmp;
  float intensity;
  int flg_result;
  int flg_noise=0;
  float noise_advance,noise_angle;
  movement movements;
  char path[100];
  float result;


 
  // it sets the environment's path
  strcpy(path,"./src/simulator/src/data/");
  

  while( ros::ok()  )
  {

    flagOnce = 1; 
    cta_steps=0;

    while(params.run) 
    {

      // it gets sensory data
      get_light_values(&intensity,light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
     
      get_lidar_values(lidar_readings,params.robot_x,
	     params.robot_y,params.robot_theta); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

      // it quantizes the sensory data      
      q_light=quantize_light(light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

      q_inputs=quantize_inputs(lidar_readings,
	   params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

      max_advance=params.robot_max_advance;
      max_turn_angle=params.robot_turn_angle;

      switch ( params.behavior)
      {

	case 1:
          // This function sends light sensory data to a function that follows a light source and it issues
          // the actions that the robot needs to perfom.
          // It is located in ~/catkin_ws/src/simulator/src/state_machines/light_follower.h
          flg_result=light_follower(intensity,light_readings,&movements);
          if(flg_result == 1) stop();
	break;


        case 2:
          // This function sends light sensory data to an state machine that follows a light source and it issues
          // the actions that the robot needs to perfom.
          // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_destination.h
	  if(flagOnce)
          {
            est_sig = 1;
            flagOnce = 0;
          }
          flg_result=sm_destination(intensity,q_light,&movements,&est_sig,
                                                        params.robot_max_advance,params.robot_turn_angle);

          if(flg_result == 1) stop();
        break;


        case 3:
	  // This function sends quantized sensory data to an state machine that avoids obstacles and it issues
          // the actions that the robot needs to perfom.
          // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance.h
          if(flagOnce)
          {
            est_sig = 0;
            flagOnce = 0;
          }
          sm_avoid_obstacles(q_inputs,&movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle);
        break;


        case 4:
	  // This function sends quantized sensory data to an state machine that follows a light source and avoids obstacles
	  // and it issues the actions that the robot needs to perfom.
          // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance_destination.h
          if(flagOnce)
          {
            est_sig = 0;
            flagOnce = 0;
          }
          flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
							params.robot_max_advance ,params.robot_turn_angle);
	  
	  if(flg_result == 1) stop();
        break;


	case 5:
          // This function sends the intensity and the quantized sensory data to a Clips node and it receives the actions that 
          // the robot needs to perfom to avoid obstacles and reach a light source according to first order logic
          // It is located in ~/catkin_ws/src/simulator/src/behaviors/oracle.h
          result=oracle_clips(intensity,q_light,q_inputs,&movements,max_advance ,max_turn_angle);
	  if(result == 1.0) stop();
        break;


	case 6:
	  // it finds a path from the origen to a destination using depth first search
          if(flagOnce)
          {
          	for(i = 0; i < 200; i++) steps[i].node=-1;

          	dfs(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
          	print_algorithm_graph (steps);
            	flagOnce = 0;
          }

	  movements.twist = 0.0; 
          movements.advance = 0.0;
          stop();
        break;


        case 7:
	  // it finds a path from the origen to a destination using the Dijkstra algorithm
          if(flagOnce)
          {
            for(i = 0; i < 200; i++)steps[i].node=-1;

            dijkstra(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
            print_algorithm_graph (steps);
            flagOnce = 0;
          }
          movements.twist = 0.0; 
          movements.advance = 0.0;
          stop();
        break;


        case 8:
	  // Here it goes your code for selection 8
	  if(flagOnce)
          {
            est_sig = 0;
            flagOnce = 0;
          }
	  user_sm(intensity,light_readings, lidar_readings, params.laser_num_sensors,params.laser_value,
					q_light,q_inputs,&movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle);
        break;

  
        default:
          movements.twist = 3.1416/4;
          movements.advance = .03;
        break;

    
      }

      ros::spinOnce();
      printf("\n\n             MOTION PLANNER \n________________________________\n");
      printf("Light: x = %f  y = %f \n",params.light_x,params.light_y);
      printf("Robot: x = %f  y = %f \n",params.robot_x,params.robot_y);
      printf("Step: %d \n",cta_steps++);
      printf("Movement: twist: %f advance: %f \n" ,movements.twist ,movements.advance );

      flg_noise=params.noise; 
      if(flg_noise==1){
                // it adds noise to the movement 
                get_random_advance_angle(&noise_advance,&noise_angle,path);
                movements.twist = movements.twist + noise_angle;
                //printf("angle + noise %f\n",movements.twist);
                movements.advance = movements.advance + noise_advance;
                //printf("distance + noise %f\n",movements.advance );
     }

      move_robot(movements.twist,movements.advance);
      ros::spinOnce();
      new_simulation =0;
    }
    ros::spinOnce();
 
  }

}
