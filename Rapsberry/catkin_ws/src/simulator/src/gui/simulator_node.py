from MobileRobotSimulator import *
from simulator.srv import *
from simulator.msg import Parameters
#from simulator.msg import Laser_values
import time
import rospy

gui=MobileRobotSimulator()


def handle_simulator_stop(req):

	resp = simulator_stopResponse()
	gui.s_t_simulation(False)
	return resp


def handle_robot_step(req):

	resp = simulator_robot_stepResponse()
	gui.sensors_value = req.sensors;
	gui.handle_service(req.theta,req.distance)
	parameters = gui.get_parameters()
	resp.robot_x = parameters[0]
	resp.robot_y = parameters[1]
	resp.theta = parameters[2]
	return resp

def handle_print_graph(req):

	resp = simulator_algorithm_resultResponse()
	gui.handle_print_graph(req.nodes_algorithm)
	resp.success=1;
	return resp

#def callback(data):
#    gui.sensors_value = data.sensors;
#    gui.X_pose = data.X_pose
#    gui.Y_pose = data.Y_pose


def ros():

	rospy.init_node('simulator_gui_node')
	a = rospy.Service('simulator_robot_step', simulator_robot_step, handle_robot_step)
	d = rospy.Service('simulator_print_graph', simulator_algorithm_result, handle_print_graph)
	b = rospy.Service('simulator_stop', simulator_stop, handle_simulator_stop)

	pub_params = rospy.Publisher('simulator_parameters_pub', Parameters, queue_size = 0)
	#rospy.Subscriber("simulator_laser_pub", Laser_values, callback)

	msg_params = Parameters()

	rate = rospy.Rate(500)

	while not gui.stopped:
	#while not rospy.is_shutdown():
		#if gui.get_vel() :
		#	rate = rospy.Rate(600)
		#else:
		#	rate = rospy.Rate(100)
		parameters = gui.get_parameters()
		msg_params.robot_x = parameters[0]
		msg_params.robot_y = parameters[1]
		msg_params.robot_theta = parameters[2]
		msg_params.robot_radio = parameters[3]
		msg_params.robot_max_advance = parameters[4]
		msg_params.robot_turn_angle = parameters[5]
		msg_params.laser_num_sensors = parameters[6]
		msg_params.laser_origin = parameters[7]
		msg_params.laser_range = parameters[8]
		msg_params.laser_value = parameters[9]
		msg_params.world_name = parameters[10]
		msg_params.noise = parameters[11]
		msg_params.light_x = parameters[12]
		msg_params.light_y = parameters[13]
		msg_params.run = parameters[14]
		msg_params.behavior = parameters[15]
		msg_params.steps = parameters[16]
		pub_params.publish(msg_params)
		rate.sleep()

if __name__ == "__main__":
    ros()