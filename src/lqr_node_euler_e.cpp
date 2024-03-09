#include "ros/ros.h"
#include <lqr_controller/lqr_euler.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ControlActionDrone.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/State.h>
#include <mavros/frame_tf.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

// HomePosition
// rosservice call gazebo/set_model_state '{model_state: { model_name: iris, pose: { position: { x: 0.0, y: 0 ,z: 10 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
//  rostopic pub -r 20 /mavros/global_position/local "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'your_frame_id'}, geo: {latitude: 37.7749, longitude: -122.4194, altitude: 0.0}, position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, approach: {x: 4.0, y: 5.0, z: 6.0}}"
//  rostopic pub -r 20 /mavros/global_position/local mavros_msgs/GlobalPositionTarget "{header: {seq: 0, stamp: now, frame_id: 'your_frame_id'}, latitude: 37.7749, longitude: -122.4194, altitude: -10.0, coordinate_frame: 6, type_mask: 4095, velocity: {x: 0.0, y: 0.0, z: 0.0}, yaw_rate: 0.0}"

int main(int argc, char **argv)
{
	// rossrv list

	ros::init(argc, argv, "LQR_Controller_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb); // Non utilizzato
	ros::Publisher cmd_raw = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10); // Non utilizzato
	ros::Publisher ctrl_raw = nh.advertise<mavros_msgs::ControlActionDrone>("mavros/controlactiondrone", 10);

	ros::Rate rate(100);

	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	lqr::LQR_Euler lqr(nh);
	Eigen::Vector3d cmd_body_rate_aircraft;
	Eigen::Vector3d cmd_body_rate_baselink;
	bool NN;
	bool FILTERED;
	// main while loop
	while (ros::ok())
	{

		mavros_msgs::AttitudeTarget bodyrate_msg;
		mavros_msgs::ControlActionDrone ctrl_msg;

		if (lqr.getPosition()(2) > -0.4)
			NN = false;
		else
			NN = true;

		// NN = true;
		FILTERED = true;

		// NN = false;

		if (NN)
		{
			lqr.computeOutputNN(lqr.getError());
			if (!FILTERED)
				lqr.setOutput(lqr.getOutput());
		}
		else
			lqr.setOutput(lqr.getTrajectoryControl() - lqr.getGain() * lqr.getError());

		std::cout << "Altezza drone: " << lqr.getPosition()(2) << std::endl;
		std::cout << "Altezza di riferimento: " << lqr.getRefStates()(2) << std::endl;
		std::cout << "\n\n"
				  << std::endl;

		if (!FILTERED)
		{
			cmd_body_rate_aircraft << lqr.getOutput()(0),
				lqr.getOutput()(1),
				lqr.getOutput()(2);
			cmd_body_rate_baselink = mavros::ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(cmd_body_rate_aircraft);

			bodyrate_msg.body_rate.x = cmd_body_rate_baselink(0);
			bodyrate_msg.body_rate.y = cmd_body_rate_baselink(1);
			bodyrate_msg.body_rate.z = cmd_body_rate_baselink(2);
			bodyrate_msg.thrust = lqr.getMotorCmd();

			if (!NN)
				std::cout << "Output LQR: [";
			else
				std::cout << "Output Neural Network: [";

			std::cout << cmd_body_rate_baselink(0) << "; ";
			std::cout << cmd_body_rate_baselink(1) << "; ";
			std::cout << cmd_body_rate_baselink(2) << "; ";
			std::cout << lqr.getMotorCmd() << "; ";
			std::cout << "]" << std::endl;
			std::cout << "\n\n"
					  << std::endl;
		}
		else
		{
			bodyrate_msg.body_rate.x = lqr.getOutput()(0);
			bodyrate_msg.body_rate.y = lqr.getOutput()(1);
			bodyrate_msg.body_rate.z = lqr.getOutput()(2);
			bodyrate_msg.thrust = lqr.getOutput()(3);

			std::cout << "Output Neural Network: [";
			for (int i = 0; i < 4; i++)
			{
				std::cout << lqr.getOutput()(i) << "; ";
			}
			std::cout << "]" << std::endl;
			std::cout << "\n\n"
					  << std::endl;
		}

		// Position vector
		state_vector_t state = lqr.getPosition();
		std::vector<float> state_vector(state.data(), state.data() + state.size());

		// Real Control Action
		std::vector<float> action_vector(4);
		action_vector[0] = bodyrate_msg.body_rate.x;
		action_vector[1] = bodyrate_msg.body_rate.y;
		action_vector[2] = bodyrate_msg.body_rate.z;
		action_vector[3] = bodyrate_msg.thrust;

		// Lqr Control Action (before the manipulation)
		control_vector_t action_lqr = lqr.getTrajectoryControl() - lqr.getGain() * lqr.getError();
		std::vector<float> lqr_vector(action_lqr.data(), action_lqr.data() + action_lqr.size());

		ctrl_msg.state = state_vector;
		ctrl_msg.action = action_vector;
		ctrl_msg.action_lqr = lqr_vector;
		ctrl_msg.V = lqr.getLyapV();

		bodyrate_msg.type_mask = 128;
		cmd_raw.publish(bodyrate_msg);
		ctrl_raw.publish(ctrl_msg);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
