#include "ros/ros.h"
#include <lqr_controller/lqr_euler.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ControlActionDrone.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "gazebo_msgs/SetModelState.h"
#include <mavros_msgs/State.h>
#include <mavros/frame_tf.h>
#include <chrono>
#include <tf2_eigen/tf2_eigen.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "LQR_Controller_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Publisher cmd_raw = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
	ros::Publisher myPublisher = nh.advertise<mavros_msgs::ControlActionDrone>("mavros/ControlActionDrone", 10);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	ros::Rate rate(100);
	int i = 0;
	double pos_x[] = {0, 0, 0, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 0};
	double pos_y[] = {0, 0, 0, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 0};

	double pos_r[] = {-0.05, -0.05, 0.08, 0.1};
	double pos_p[] = {0, 0, 0, 0};
	// double pos_p[] = {-0.1, 0.1, -0.1, 0.1};

	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	lqr::LQR_Euler lqr(nh);

	double start_timer = ros::Time::now().toSec(); // il primo timer
												   // We need to initialize duration for the first iteration
	std::fstream FileName;
	std::fstream FileName2;

	// main while loop
	while (ros::ok())
	{

		mavros_msgs::AttitudeTarget bodyrate_msg;
		mavros_msgs::ControlActionDrone ctrl_msg;

		geometry_msgs::Point drone_position;
		geometry_msgs::Quaternion drone_orientation;
		geometry_msgs::Pose drone_pose;
		geometry_msgs::Twist drone_vel;
		gazebo_msgs::ModelState modelstate;
		gazebo_msgs::SetModelState sms;
		double rpy_in[3];
		double quat_in[3];

		// std::cout << "Altezza drone" <<lqr.getPosition()(2) <<std::endl;
		// std::cout << "Altezza di riferimento" <<lqr.getRefStates()(2) <<std::endl;
		Eigen::Vector3d cmd_body_rate_aircraft;
		Eigen::Vector3d cmd_body_rate_baselink;

		double end_timer = ros::Time::now().toSec();

		double duration = end_timer - start_timer;

		if (duration < 15.0 && lqr.takeoff == true)
		{
			double end_timer = ros::Time::now().toSec();
			double duration = end_timer - start_timer; // duration change of 0.01 each iteration

			std::cout << "Remaining Time Lqr Control: " << (15 - duration) << std::endl;
			lqr.setOutput(lqr.getTrajectoryControl() - lqr.getGain() * lqr.getError());
			bodyrate_msg.thrust = lqr.getMotorCmd(); // Take off (//Position control   getMotoCmd invece di getMotoCmdNN perchè ci serve normalizzare)
			// std::cout << "Lqr Take off" << std::endl;

			cmd_body_rate_aircraft << lqr.getOutput()(0),
				lqr.getOutput()(1),
				lqr.getOutput()(2);

			cmd_body_rate_baselink = mavros::ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(cmd_body_rate_aircraft);

			bodyrate_msg.body_rate.x = cmd_body_rate_baselink(0);
			bodyrate_msg.body_rate.y = cmd_body_rate_baselink(1);
			bodyrate_msg.body_rate.z = cmd_body_rate_baselink(2);

			// The lower part is for despawning and respawning the drone each time in different positions

			// if(ros::Time::now().toSec()- start_timer > 50.0)
			// {
			//     // /////////////////per settare le posizioni lineari
			//     // drone_position.x = ((rand()/(double)RAND_MAX)* (0.5 + 0.5))-0.5;       //X and Y will vary between +- 5, Z stays at 1
			//     // drone_position.y = ((rand()/(double)RAND_MAX)* (0.5 + 0.5))-0.5; //stiamo fornendo queste posizioni rispetto al sistema di riferimento globale (ENU)
			// 	// drone_position.x = pos_x[i];
			// 	// drone_position.y = pos_y[i];
			//     // drone_position.z = 1;

			// 	drone_position.x = 0;
			// 	drone_position.y = 0;
			// 	drone_position.z = 1;

			// 	// rpy_in[0] = ((rand()/(double)RAND_MAX)* (0.1 + 0.1))-0.1;
			// 	// rpy_in[1] = ((rand()/(double)RAND_MAX)* (0.1 + 0.1))-0.1;

			// 	rpy_in[0] = pos_r[i];
			// 	rpy_in[1] = pos_p[i];

			//     i = i+1;
			// 	// // rpy_in[2] = ((rand()/(double)RAND_MAX)* (0.1 + 0.1))-0.1;
			// 	// // rpy_in[2]=1.57;

			// 	lqr.eulerToQuat(rpy_in, quat_in);

			// 	drone_orientation.w=quat_in[0];
			// 	drone_orientation.x=quat_in[1];
			// 	drone_orientation.y=quat_in[2];
			// 	drone_orientation.z=quat_in[3];

			//     FileName.open("/home/alessiaphd/Scrivania/Materia_Giulio_Damiano_Super_Fixed/LQR_Position_19_02_fixated.txt",std::ios::app);
			//     FileName <<drone_position.x << "," << drone_position.y<< "\n";
			//     FileName.close();
			// 	FileName2.open("/home/alessiaphd/Scrivania/Materia_Giulio_Damiano_Super_Fixed/lqr_Orientation_19_02_+-0_1.txt",std::ios::app);
			//     FileName2 <<drone_orientation.w << "," << drone_orientation.x<<"," << drone_orientation.y<<"," << drone_orientation.z<< "\n";
			//     FileName2.close();

			//     /////////per settare le velocità angolari
			//     drone_vel.linear.x = 0;
			//     drone_vel.linear.y = 0;
			//     drone_vel.linear.z = 0;

			//     drone_vel.angular.x = 0;
			//     drone_vel.angular.y = 0;
			//     drone_vel.angular.z = 0;

			//     drone_pose.position = drone_position;
			//     drone_pose.orientation = drone_orientation;

			//     modelstate.model_name = "iris";
			//     modelstate.reference_frame = "world";
			//     modelstate.pose = drone_pose;
			//     modelstate.twist = drone_vel;

			//     sms.request.model_state = modelstate;

			//     if(client.call(sms))
			//     {
			//         printf("DRONE CHANGED!!!");
			//         start_timer = ros::Time::now().toSec();
			//     }
			//     else
			//     {
			//         printf("Nada!");
			//     }
			// }
		}
		else
		{ // once the amount of time ends                                               //SWITCH

			lqr.takeoff = false; // our neuralNetwork control substitutes the LQR and we never go back

			bodyrate_msg.body_rate.x = lqr.getOutputNN()(0);
			bodyrate_msg.body_rate.y = lqr.getOutputNN()(1);
			bodyrate_msg.body_rate.z = lqr.getOutputNN()(2);
			bodyrate_msg.thrust = lqr.getOutputNN()(3);
			// std::cout << "Control from NeuralNetwork" << std::endl;

			// The lower part is for despawning and respawning the drone each time in different positions

			// if(ros::Time::now().toSec()- start_timer > 50000000.0)
			// {
			//     /////////////per settare le posizioni lineari
			//     // drone_position.x = ((rand()/(double)RAND_MAX)* (2 + 2))-2;
			//     // drone_position.y = ((rand()/(double)RAND_MAX)* (2 + 2))-2; //stiamo fornendo queste posizioni rispetto al sistema di riferimento globale (ENU)
			// 	// drone_position.x = pos_x[i];
			// 	// drone_position.y = pos_y[i];
			//     // drone_position.z = 1;

			// 	drone_position.x = 0;
			// 	drone_position.y = 0;
			// 	drone_position.z = 1;

			// 	// rpy_in[0] = ((rand()/(double)RAND_MAX)* (0.5 + 0.5))-0.5;
			// 	// rpy_in[1] = ((rand()/(double)RAND_MAX)* (0.5 + 0.5))-0.5;      //roll and pitch will vary between +- 0.5, yaw stays at 1.57

			// 	rpy_in[0] = pos_r[i];
			// 	rpy_in[1] = pos_p[i];

			// 	i=i+1;

			// 	// // // rpy_in[2] = ((rand()/(double)RAND_MAX)* (0.1 + 0.1))-0.1;
			// 	// // // rpy_in[2]=1.57;

			// 	lqr.eulerToQuat(rpy_in, quat_in);

			// 	drone_orientation.w=quat_in[0];
			// 	drone_orientation.x=quat_in[1];
			// 	drone_orientation.y=quat_in[2];
			// 	drone_orientation.z=quat_in[3];

			//     FileName.open("/home/alessiaphd/Scrivania/Materia_Giulio_Damiano_Super_Fixed/Position_19_02_fixated.txt",std::ios::app);
			//     FileName <<drone_position.x << "," << drone_position.y<< "\n";
			//     FileName.close();
			// 	FileName2.open("/home/alessiaphd/Scrivania/Materia_Giulio_Damiano_Super_Fixed/Orientation_19_02_+-0_1.txt",std::ios::app);
			//     FileName2 <<drone_orientation.w << "," << drone_orientation.x<<"," << drone_orientation.y<<"," << drone_orientation.z<< "\n";
			//     FileName2.close();

			//     /////////////////////per settare le velocità angolari
			//     drone_vel.linear.x = 0;
			//     drone_vel.linear.y = 0;
			//     drone_vel.linear.z = 0;

			//     drone_vel.angular.x = 0;
			//     drone_vel.angular.y = 0;
			//     drone_vel.angular.z = 0;

			//    	drone_pose.position = drone_position;
			//     drone_pose.orientation = drone_orientation;

			//     modelstate.model_name = "iris";
			//     modelstate.reference_frame = "world";
			//     modelstate.pose = drone_pose;
			//     modelstate.twist = drone_vel;

			//     sms.request.model_state = modelstate;

			//     if(client.call(sms))
			//     {
			//         printf("DRONE CHANGED!!!");
			//         start_timer = ros::Time::now().toSec();
			//     }
			//     else
			//     {
			//         printf("Nada!");
			//     }
			//  }
		}

		// Error vector managing
		state_vector_t xerror_2;
		xerror_2 = lqr.getError();
		Eigen::Map<Eigen::VectorXd> M(xerror_2.data(), 9, 1);
		Eigen::VectorXd M2(M);

		std::vector<float> errorvector(&M2[0], M2.data() + M2.cols() * M2.rows());

		// xref vector managing
		state_vector_t xref2save;
		xref2save = lqr.getRefStates();
		Eigen::Map<Eigen::VectorXd> Mxref(xref2save.data(), 9, 1);
		Eigen::VectorXd M2xref(Mxref);

		std::vector<float> xrefvector(&M2xref[0], M2xref.data() + M2xref.cols() * M2xref.rows());

		// Uref vector managing
		control_vector_t uref_2;
		uref_2 = lqr.getTrajectoryControl();
		Eigen::Map<Eigen::VectorXd> M3(uref_2.data(), 4, 1); // we have only 4 uref (roll pitch yaw and thrust)
		Eigen::VectorXd M4(M3);

		std::vector<float> urefvector(&M4[0], M4.data() + M4.cols() * M4.rows());

		// Position vector managing
		state_vector_t position2;
		position2 = lqr.getPosition();
		Eigen::Map<Eigen::VectorXd> M5(position2.data(), 9, 1);
		Eigen::VectorXd M6(M5);

		std::vector<float> positionvector(&M6[0], M6.data() + M6.cols() * M6.rows());

		// Controll input managing
		control_vector_t ke;
		ke = lqr.getGain() * lqr.getError();			 // implemented by us
		Eigen::Map<Eigen::VectorXd> M7(ke.data(), 4, 1); // 4 uref (roll pitch yaw thrust)
		Eigen::VectorXd M8(M7);

		std::vector<float> ke_f(&M7[0], M7.data() + M7.cols() * M7.rows());

		//   ctrl_msg.V = lqr.getLyapV();     //HOME MADE
		//   ctrl_msg.timer_NN  = lqr.getTimerNN();
		//   ctrl_msg.timer_LQR  = lqr.getTimerLQR();
		//   ctrl_msg.xref = xrefvector;
		//   ctrl_msg.error= errorvector;  //IMPLEMENTED BY US
		//   ctrl_msg.uref= urefvector;    //IMPLEMENTED BY US
		//   ctrl_msg.position= positionvector; //IMPLEMENTED BY US (per avere xyz e roll pitch yaw con la stessa frequenza di come abbiamo errore e uref, nota che devi prendere i primi 6 elementi)

		bodyrate_msg.type_mask = 128;
		cmd_raw.publish(bodyrate_msg);

		//   myPublisher.publish(ctrl_msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
