/* #include "ros/ros.h"
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


mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

int main(int argc, char** argv)
{

  ros::init(argc,argv,"LQR_Controller_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher cmd_raw = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  ros::Publisher myPublisher = nh.advertise<mavros_msgs::ControlActionDrone>("mavros/ControlActionDrone", 10);
  //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//   ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");		
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo_msg/SetModelState");		
  ros::Rate rate(100);
  
  while(ros::ok() && !current_state.connected) {
     ros::spinOnce();
     rate.sleep();
  }

  lqr::LQR_Euler lqr(nh);

  auto start_timer = std::chrono::high_resolution_clock::now();       
      //We need to initialize duration for the first iteration


  //main while loop
  while (ros::ok()) {

	  mavros_msgs::AttitudeTarget bodyrate_msg;
	  mavros_msgs::ControlActionDrone ctrl_msg;                                                


	  //std::cout << "Altezza drone" <<lqr.getPosition()(2) <<std::endl;
	  //std::cout << "Altezza di riferimento" <<lqr.getRefStates()(2) <<std::endl;
	  Eigen::Vector3d cmd_body_rate_aircraft;
	  Eigen::Vector3d cmd_body_rate_baselink;
      
      geometry_msgs::Point drone_position;
      geometry_msgs::Pose drone_pose;
      gazebo_msgs::ModelState modelstate;
      gazebo_msgs::SetModelState sms;   
      
      
	auto end_timer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_timer - start_timer; 
	if(duration.count()<18.0 && lqr.takeoff == true)
    { 
		auto end_timer = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> duration = end_timer - start_timer;        //duration change of 0.01 each iteration 
		std::cout << "Duration: " << duration.count() << std::endl;
	
        lqr.setOutput(lqr.getTrajectoryControl() - lqr.getGain()*lqr.getError());
		bodyrate_msg.thrust = lqr.getMotorCmd();                                    //Take off
        cmd_body_rate_aircraft << lqr.getOutput()(0),
								lqr.getOutput()(1),
								lqr.getOutput()(2);
	   }
	   else 
       {   
           
            drone_position.x = 5.0;
            drone_position.y = 0.0;
            drone_position.z = 0.0;

            pr2_pose.position = drone_position;

            modelstate.model_name = "iris";
            modelstate.reference_frame = "world";
            modelstate.pose = pr2_pose;

            
            sms.request.model_state = modelstate;

            if(client.call(sms))
            {
                printf("DRONE CHANGED!!!\n");
            } 
            else
            {
                printf("Nada!\n");
            }
           
           
           //once the amount of time ends                                               //SWITCH
// 		lqr.takeoff = false; //our neuralNetwork control substitutes the LQR and we never go back
// 		lqr.setOutput(lqr.getOutputNN());
// 		bodyrate_msg.thrust = lqr.getMotorCmd();                                  //Position control   getMotoCmd invece di getMotoCmdNN perchè ci serve normalizzare
// 		std::cout << "Control from NeuralNetwork" << std::endl;
// 	  	cmd_body_rate_aircraft << lqr.getOutputNN()(0),
// 								lqr.getOutputNN()(1),
// 								lqr.getOutputNN()(2);
		}
       
	  std::cout << "motor cmd:" << lqr.getMotorCmd() << std::endl;

	  cmd_body_rate_baselink = mavros::ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(cmd_body_rate_aircraft);

      //Error vector managing
	  state_vector_t xerror_2;
	  xerror_2 =lqr.getError();
	  Eigen::Map<Eigen::VectorXd> M(xerror_2.data(), 9, 1);
   	  Eigen::VectorXd M2(M);

	  std::vector<float> errorvector(&M2[0], M2.data()+M2.cols()*M2.rows());

	  //Uref vector managing
	  control_vector_t uref_2;
	  uref_2= lqr.getTrajectoryControl();
	  Eigen::Map<Eigen::VectorXd> M3(uref_2.data(), 4, 1);//we have only 4 uref (roll pitch yaw and thrust)
   	  Eigen::VectorXd M4(M3);

	  std::vector<float> urefvector(&M4[0], M4.data()+M4.cols()*M4.rows());

	  //Position vector managing
	  state_vector_t position2;
	  position2 =lqr.getPosition();
	  Eigen::Map<Eigen::VectorXd> M5(position2.data(), 9, 1);
   	  Eigen::VectorXd M6(M5);

	  std::vector<float> positionvector(&M6[0], M6.data()+M6.cols()*M6.rows());

      //Controll input managing
 	  control_vector_t ke;
	  ke = lqr.getGain()*lqr.getError();    //implemented by us
      Eigen::Map<Eigen::VectorXd> M7(ke.data(), 4, 1);            //4 uref (roll pitch yaw thrust)
   	  Eigen::VectorXd M8(M7);

	  std::vector<float> ke_f(&M7[0], M7.data()+M7.cols()*M7.rows());


	  bodyrate_msg.body_rate.x = cmd_body_rate_baselink(0);
	  bodyrate_msg.body_rate.y = cmd_body_rate_baselink(1);
	  bodyrate_msg.body_rate.z = cmd_body_rate_baselink(2);

      bodyrate_msg.type_mask = 128;
	  cmd_raw.publish(bodyrate_msg);
	  
	  
//Those 4 upper assignments are the only way that the controller has to communicate with the quadrotor. In facts without the publish it doesn't work.
//The idea is to overwrite those 4 instructions with outputs from the neural networks

	  ctrl_msg.V = lqr.getLyapV();     //HOME MADE
	  ctrl_msg.error= errorvector;  //IMPLEMENTED BY US
	  ctrl_msg.uref= urefvector;    //IMPLEMENTED BY US
	  ctrl_msg.position= positionvector; //IMPLEMENTED BY US (per avere xyz e roll pitch yaw con la stessa frequenza di come abbiamo errore e uref, nota che devi prendere i primi 6 elementi)
	  //RICORDA CHE HAI MODIFICATO IL FILE catkin_ws/src/mavros/mavros_msgs/msg AGGIUNGENDO IL TIPO E IL NOME DI OGNI CAMPO DEI MESSAGGI CHE HAI FATTO STAMPARE
	  ctrl_msg.keRoll = ke_f[0];    //IMPLEMENTED BY US
      ctrl_msg.kePitch = ke_f[1];    //IMPLEMENTED BY US
	  ctrl_msg.keYaw = ke_f[2];    //IMPLEMENTED BY US
	  ctrl_msg.keThrust = ke_f[3];    //IMPLEMENTED BY US
	  
	  myPublisher.publish(ctrl_msg);
	  ros::spinOnce();
	  rate.sleep();

  }


  return 0;

}
 */