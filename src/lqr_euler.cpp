// INTERO CONTROLLORE
#include "lqr_controller/lqr_euler.hpp"
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <nlohmann/json.hpp>
#include <unistd.h>
// #include <std>

using json = nlohmann::json;

namespace lqr
{
  void LQR_Euler::loadNeuralNetworkConfig(const std::string &configFile)
  {
    std::ifstream file(configFile);

    std::string weightsPath;
    if (file.is_open())
    {
      json jsonData;
      file >> jsonData;

      numLayers = jsonData["num_layers"];
      isQuaternion = jsonData["isQuaternion"];
      weightsPath = jsonData["weights_paths"];
      if (isQuaternion)
        numOutNeurons = jsonData["num_output_neurons"];
      file.close();

      W.reserve(numLayers);
      B.reserve(numLayers);

      for (int i = 0; i < numLayers; i++)
      {
        W.push_back(load_csv<Eigen::MatrixXd>(ros::package::getPath("lqr_controller") + "/" + weightsPath + "/W" + std::to_string(i + 1) + ".csv"));
        B.push_back(load_csv<Eigen::MatrixXd>(ros::package::getPath("lqr_controller") + "/" + weightsPath + "/B" + std::to_string(i + 1) + ".csv"));
      }

      std::cout << "Neural Net configuration acquired." << std::endl;
      std::cout << "Weights Path: " << weightsPath << std::endl;
      std::cout << "Number of Layers: " << numLayers << std::endl;
      std::cout << "Is Quaternion: " << isQuaternion << std::endl;
      if (isQuaternion)
        std::cout << "Number of output neurons: " << numOutNeurons << std::endl;
      std::cout << "\n\n"
                << std::endl;
      sleep(2);
    }
    else
    {
      std::cerr << "Error opening the Neural Net configuration file." << std::endl;
      std::exit(0);
    }
  }

  void LQR_Euler::loadTestConfig(const std::string &configFile)
  {
    std::ifstream file(configFile);

    std::string weightsPath;
    if (file.is_open())
    {
      json jsonData;
      file >> jsonData;

      testDuration = jsonData["testDuration"];
      logInterval = jsonData["logInterval"];
      auto referenceList = jsonData["referenceList"];

      for (const auto &element : referenceList)
      {
        Eigen::Vector4d vector4d;
        for (size_t i = 0; i < element.size(); ++i)
          vector4d(i) = element[i];
        staticRefNN.push_back(vector4d);
      }
    }
    else
    {
      std::cerr << "Error opening the Test configuration file." << std::endl;
      std::exit(0);
    }
  }

  LQR_Euler::LQR_Euler(ros::NodeHandle &nodeHandle)
      : nodeHandle_(nodeHandle)
  {
    loadNeuralNetworkConfig(ros::package::getPath("lqr_controller") + "/Config/NeuralNet.json");
    loadTestConfig(ros::package::getPath("lqr_controller") + "/Config/TestReference.json");

    quadraticCost_.loadConfigFile(ros::package::getPath("lqr_controller") + "/lqrCostEuler.info", "termLQR");
    Q_ = quadraticCost_.getStateWeight();
    R_ = quadraticCost_.getControlWeight();
    odom_sub_ = nodeHandle_.subscribe("/mavros/local_position/odom", 1, &LQR_Euler::topicCallback, this);
    marker_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
    RCIn_sub_ = nodeHandle_.subscribe("/mavros/rc/in", 1, &LQR_Euler::setRCIn, this);
    initiated = false;
    traj_index = 1;
    traj_offset = 0;
    traj_time = 1;
    index = 0;
    counter = -1;

    channels_.resize(18);

    if (!readParameters())
    {
      ROS_ERROR("Could not read parameters.");
      ros::requestShutdown();
    }

    callBack_ = ros::Time::now();
    last_log_time = init_time_ = ros::Time::now().toSec();
  }

  LQR_Euler::~LQR_Euler()
  {
  }

  void LQR_Euler::setRCIn(const mavros_msgs::RCIn::ConstPtr &msg)
  {
    channels_ = msg->channels;
  }

  void LQR_Euler::topicCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    setStates(msg, x_);

    if (!takeoff)
    {
      // Set new state for testing
      if (counter == -1 || (ros::Time::now().toSec() - init_time_ > testDuration && counter < (staticRefNN.size() - 1)))
      {
        counter += 1;
        setStaticReference(xref_, uref_, staticRefNN[counter]);
        init_time_ = ros::Time::now().toSec();
      }

      if (ros::Time::now().toSec() - last_log_time > logInterval)
      {
        last_log_time = ros::Time::now().toSec();

        std::cout << "Remaing time for next state:" << (int)(testDuration - (ros::Time::now().toSec() - init_time_)) << "\n\n";

        std::cout << "Current State: [";
        for (int i = 0; i < 3; i++)
        {
          std::cout << x_[i] << "; ";
        }
        std::cout << "]" << std::endl;
        std::cout << "\n"
                  << std::endl;
        std::cout << "Reference State: [";
        for (int i = 0; i < 3; i++)
        {
          std::cout << staticRefNN[counter][i] << "; ";
        }
        std::cout << "]" << std::endl;
        std::cout << "\n\n\n"
                  << std::endl;
      }
    }
    else
      setStaticReference(xref_, uref_, staticRefNN[0]);
    setError(xref_, x_, xerror_);

    start_timerNN = std::chrono::system_clock::now();
    neuralNetwork(xerror_);
    end_timerNN = std::chrono::system_clock::now();

    elapsed_secondsNN = end_timerNN - start_timerNN;
    end_time = std::chrono::system_clock::to_time_t(end_timerNN);
    this->timeNN = elapsed_secondsNN.count();

    // std::cout << "elapsed time NN: " << this->timeNN << std::endl;

    // start_timerNN = ros::Time::now();
    // neuralNetwork(xerror_);
    // end_timerNN = ros::Time::now();
    // std::cout << "TIMER NN FUNCTION: " << (end_timerNN-start_timerNN).toNSec() << std::endl;

    start_timerAB = std::chrono::system_clock::now();
    A_ = A_quadrotor(x_, uref_);
    B_ = B_quadrotor(x_, uref_);

    if (lqrSolver_.compute(Q_, R_, A_, B_, Knew_, P_Ricc)) // Lqr_solver sta in ct_optcon
    {
      if (!Knew_.hasNaN())
      {
        Kold_ = Knew_;
      }
    }

    end_timerAB = std::chrono::system_clock::now();
    elapsed_secondsLQR = end_timerAB - start_timerAB;
    this->timeLQR = elapsed_secondsLQR.count();
    // std::cout << "elapsed time AB: " << this->timeLQR<< std::endl;

    // if (lqr_test == true)
    // {
    //   this->V = xerror_.transpose() * P_Ricc * xerror_; // FATTA DA ME, fixata con vettore errore
    // }
    // else
    // {
    //   lyapunov_net(xerror_);
    // }
    /*std::cout << "A matrix:" << std::endl
              << A_ << std::endl;
    std::cout << "B matrix:" << std::endl
              << B_ << std::endl;
    std::cout << "LQR gain matrix:" << std::endl
              << Kold_ << std::endl;
    std::cout << " P:" << std::endl
              << P_Ricc << std::endl;
    //  std::cout << "q:" << std::endl << q_ned_.coeffs() << std::endl;
    std::cout << "x:" << std::endl
              << x_ << std::endl;*/

    // callBack_ = ros::Time::now();
    // }
  }

  state_matrix_t LQR_Euler::A_quadrotor(const state_vector_t &x, const control_vector_t &u)
  {
    double wx = u(0);
    double wy = u(1);
    double wz = u(2);
    double norm_thrust = u(3);
    Eigen::Matrix<double, 3, 3> dEdot_dE;
    Eigen::Matrix<double, 3, 3> dvdot_dE;

    double phi = x(3);
    double theta = x(4);
    double psi = x(5);

    state_matrix_t A;
    A.setZero();

    // Position
    A(0, 6) = 1;
    A(1, 7) = 1;
    A(2, 8) = 1;
    Eigen::Matrix<double, 4, 4> Identity;

    // Orientation
    dEdot_dE << wy * cos(phi) * tan(theta) - wz * sin(phi) * tan(theta), pow(1 / cos(theta), 2) * (wy * sin(phi) + wz * cos(phi)), 0,
        -wy * sin(phi) - wz * cos(phi), 0, 0,
        (1 / cos(theta)) * (wy * cos(phi) - wz * sin(phi)), (1 / cos(theta)) * tan(theta) * (wy * sin(phi) + wz * cos(phi)), 0;

    A(3, 3) = dEdot_dE(0, 0);
    A(3, 4) = dEdot_dE(0, 1);
    A(3, 5) = dEdot_dE(0, 2);

    A(4, 3) = dEdot_dE(1, 0);
    A(4, 4) = dEdot_dE(1, 1);
    A(4, 5) = dEdot_dE(1, 2);

    A(5, 3) = dEdot_dE(2, 0);
    A(5, 4) = dEdot_dE(2, 1);
    A(5, 5) = dEdot_dE(2, 2);

    // Velocity
    dvdot_dE << cos(phi) * sin(psi) - sin(phi) * sin(theta) * cos(psi), cos(phi) * cos(theta) * cos(psi), sin(phi) * cos(psi) - cos(phi) * sin(theta) * sin(psi),
        -cos(phi) * cos(psi) - sin(phi) * sin(theta) * sin(psi), cos(phi) * cos(theta) * sin(psi), sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi),
        -sin(phi) * cos(theta), cos(phi) * sin(theta), sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi);
    dvdot_dE = -norm_thrust * dvdot_dE;

    A(6, 3) = dvdot_dE(0, 0);
    A(6, 4) = dvdot_dE(0, 1);
    A(6, 5) = dvdot_dE(0, 2);

    A(7, 3) = dvdot_dE(1, 0);
    A(7, 4) = dvdot_dE(1, 1);
    A(7, 5) = dvdot_dE(1, 2);

    A(8, 3) = dvdot_dE(2, 0);
    A(8, 4) = dvdot_dE(2, 1);
    A(8, 5) = dvdot_dE(2, 2);

    return A;
  }

  control_gain_matrix_t LQR_Euler::B_quadrotor(const state_vector_t &x, const control_vector_t &u)
  {
    double wx = u(0);
    double wy = u(1);
    double wz = u(2);
    double norm_thrust = u(3);
    Eigen::Matrix<double, 3, 1> dvdot_dc;
    Eigen::Matrix<double, 3, 3> dEdot_dw;

    double phi = x(3);
    double theta = x(4);
    double psi = x(5);

    control_gain_matrix_t B;
    B.setZero();

    dvdot_dc << -(sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi)),
        -(-sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi)),
        -(cos(phi) * cos(theta));

    B(6, 3) = dvdot_dc(0);
    B(7, 3) = dvdot_dc(1);
    B(8, 3) = dvdot_dc(2);

    dEdot_dw << 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
        0, cos(phi), -sin(phi),
        0, sin(phi) * (1 / cos(theta)), cos(phi) * (1 / cos(theta));

    B(3, 0) = dEdot_dw(0, 0);
    B(3, 1) = dEdot_dw(0, 1);
    B(3, 2) = dEdot_dw(0, 2);

    B(4, 0) = dEdot_dw(1, 0);
    B(4, 1) = dEdot_dw(1, 1);
    B(4, 2) = dEdot_dw(1, 2);

    B(5, 0) = dEdot_dw(2, 0);
    B(5, 1) = dEdot_dw(2, 1);
    B(5, 2) = dEdot_dw(2, 2);

    return B;
  }

  void LQR_Euler::setStates(const nav_msgs::Odometry::ConstPtr &msg, state_vector_t &x)
  {
    // Orientation
    q_enu_.w() = -1 * msg->pose.pose.orientation.w;
    q_enu_.x() = -1 * msg->pose.pose.orientation.x;
    q_enu_.y() = -1 * msg->pose.pose.orientation.y;
    q_enu_.z() = -1 * msg->pose.pose.orientation.z;

    q_ned_ = mavros::ftf::transform_orientation_enu_ned<Eigen::Quaterniond>(q_enu_);

    q_ned_ = mavros::ftf::transform_orientation_baselink_aircraft<Eigen::Quaterniond>(q_ned_);

    Eigen::Vector3d euler = quaternion_to_rpy_wrap(q_ned_);

    // position transformation
    position_enu_ << msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z;
    Eigen::Vector3d position_ned = mavros::ftf::transform_frame_enu_ned(position_enu_);

    // velocity transformation

    velocity_enu_ << msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z;
    Eigen::Vector3d velocity_ned = mavros::ftf::transform_frame_enu_ned(mavros::ftf::transform_frame_baselink_enu(velocity_enu_, q_enu_));

    // STATES
    // Position
    x(0) = position_ned(0);
    x(1) = position_ned(1);
    x(2) = position_ned(2);

    // Orientation
    x(3) = euler.x(); // roll pitch yaw
    x(4) = euler.y();
    x(5) = euler.z();

    // Linear Velocities
    x(6) = velocity_ned(0);
    x(7) = velocity_ned(1);
    x(8) = velocity_ned(2);
  }

  void LQR_Euler::setError(const state_vector_t &xref, const state_vector_t &x, state_vector_t &xerror)
  {
    // Position error
    xerror(0) = -1 * (x(0) - xref(0));
    xerror(1) = -1 * (x(1) - xref(1));
    xerror(2) = -1 * (x(2) - xref(2));

    // Angular position error CREDO
    xerror(3) = (x(3) - xref(3));
    xerror(4) = (x(4) - xref(4));
    xerror(5) = (x(5) - xref(5));

    // Velocity error
    xerror(6) = -1 * (x(6) - xref(6));
    xerror(7) = -1 * (x(7) - xref(7));
    xerror(8) = -1 * (x(8) - xref(8));
  }

  bool LQR_Euler::setTrajectoryReference(state_vector_t &xref, control_vector_t &uref, int trajectory_type)
  {
    if (!initiated)
    {
      generateTrajectory(states_, trajectory_type);
      setTrajectoryStates(&states_);
    }
    initiated = true;
    Eigen::Vector3d position_ref_enu;
    Eigen::Vector3d position_ref_ned;
    Eigen::Vector3d accel;
    Eigen::Vector3d orient_yaw;
    Eigen::Vector3d thrust;
    Eigen::Vector3d thrust_dir;
    Eigen::Quaterniond q_yaw;
    Eigen::Vector3d rpy_ref_enu;
    Eigen::Quaterniond qref_enu;
    Eigen::Quaterniond qref_ned;
    Eigen::Vector3d rpy_ref_ned;
    Eigen::Vector3d velocity_enu;
    Eigen::Vector3d velocity_ned;
    std::vector<double> gamma;
    gamma.resize(states_.size() - 1);
    std::vector<double> dist_traj;
    dist_traj.resize(states_.size() - 1);

    int i;

    if (!closed_traj_)
    {
      i = traj_index - 1;
    }
    else
    {
      i = 0;
    }

    do
    {
      i++;
      Eigen::Vector3d distance_vec(states_[i].position_W - states_[i - 1].position_W);
      gamma[i - 1] = (distance_vec.dot(position_enu_ - states_[i - 1].position_W) / pow(distance_vec.norm(), 2));
      dist_traj[i - 1] = ((states_[i].position_W - position_enu_).norm());

      std::cout << "gamma" << gamma[i - 1] << std::endl;

    } while (i < states_.size() - 1);

    if (traj_index > (states_.size() - 5) && !closed_traj_)
    {
      traj_index = states_.size() - 1;

      position_ref_enu = states_[traj_index].position_W;
      position_ref_ned << mavros::ftf::transform_frame_enu_ned(position_ref_enu);
      qref_ned = mavros::ftf::transform_orientation_baselink_aircraft(mavros::ftf::transform_orientation_enu_ned(states_[traj_index].orientation_W_B));
      rpy_ref_ned = (quaternion_to_rpy_wrap(qref_ned));

      xref(0) = position_ref_ned.x();
      xref(1) = position_ref_ned.y();
      xref(2) = position_ref_ned.z();

      xref(3) = 0;
      xref(4) = 0;
      xref(5) = rpy_ref_ned.z();

      xref(6) = 0;
      xref(7) = 0;
      xref(8) = 0;

      uref(0) = 0;
      uref(1) = 0;
      uref(2) = 0;
      uref(3) = -9.81;
      // std::cout << "xref" << xref << std::endl;
      // std::cout << "uref" << uref << std::endl;
      return true;
    }
    else
    {

      std::vector<double>::iterator result;

      if (!closed_traj_)
      {
        result = std::min_element(dist_traj.begin() + traj_index, dist_traj.end());
      }
      else
      {
        result = std::min_element(dist_traj.begin(), dist_traj.end());
      }

      traj_index = std::distance(dist_traj.begin(), result);
      std::cout << "traj_index" << traj_index << std::endl;

      if (traj_index == 0)
      {
        traj_index = 1;
      }
      // Position

      position_ref_enu = states_[traj_index - 1].position_W + gamma[traj_index] * (states_[traj_index].position_W - states_[traj_index - 1].position_W);
      position_ref_ned = mavros::ftf::transform_frame_enu_ned<Eigen::Vector3d>(position_ref_enu);

      // Orientation

      accel = (states_[traj_index - 1].acceleration_W + gamma[traj_index] * (states_[traj_index].acceleration_W - states_[traj_index - 1].acceleration_W));
      thrust = (Eigen::Vector3d(0, 0, quadrotor::grav_const) + accel);
      thrust_dir = (thrust.normalized());

      //  orient_yaw = (quaternion_to_rpy_wrap(states_[traj_index-1].orientation_W_B) + gamma[traj_index]*(quaternion_to_rpy_wrap(states_[traj_index].orientation_W_B) - quaternion_to_rpy_wrap(states_[traj_index-1].orientation_W_B)));
      //  q_yaw = (Eigen::AngleAxisd(orient_yaw.z(),Eigen::Vector3d(0,0,1)));
      //  Eigen::Vector3d interm_thrust_frame(mavros::ftf::detail::transform_frame(thrust_dir,q_yaw));
      //  rpy_ref_enu << atan2(-1*interm_thrust_frame.y(),sqrt(pow(interm_thrust_frame.x(),2)+pow(interm_thrust_frame.z(),2))),
      //                 atan2(interm_thrust_frame.x(),interm_thrust_frame.z()),
      //                 orient_yaw.z();

      rpy_ref_enu = (quaternion_to_rpy_wrap(states_.at(traj_index - 1).orientation_W_B) + gamma[traj_index] * (quaternion_to_rpy_wrap(states_.at(traj_index).orientation_W_B) - quaternion_to_rpy_wrap(states_.at(traj_index - 1).orientation_W_B)));

      qref_enu = mavros::ftf::quaternion_from_rpy(rpy_ref_enu);
      qref_ned = (mavros::ftf::transform_orientation_baselink_aircraft(mavros::ftf::transform_orientation_enu_ned(qref_enu)));

      rpy_ref_ned = (quaternion_to_rpy_wrap(qref_ned));

      velocity_enu << states_.at(traj_index - 1).velocity_W.x() + gamma[traj_index] * (states_.at(traj_index).velocity_W.x() - states_.at(traj_index - 1).velocity_W.x()),
          states_.at(traj_index - 1).velocity_W.y() + gamma[traj_index] * (states_.at(traj_index).velocity_W.y() - states_.at(traj_index - 1).velocity_W.y()),
          states_.at(traj_index - 1).velocity_W.z() + gamma[traj_index] * (states_.at(traj_index).velocity_W.z() - states_.at(traj_index - 1).velocity_W.z());
      velocity_ned << (mavros::ftf::transform_frame_enu_ned(velocity_enu));

      xref(0) = position_ref_ned.x();
      xref(1) = position_ref_ned.y();
      xref(2) = position_ref_ned.z();

      xref(3) = rpy_ref_ned.x();
      xref(4) = rpy_ref_ned.y();
      xref(5) = rpy_ref_ned.z();

      xref(6) = velocity_ned.x();
      xref(7) = velocity_ned.y();
      xref(8) = velocity_ned.z();

      Eigen::Vector3d angular_velocity_des_W;
      angular_velocity_des_W << states_.at(traj_index - 1).angular_velocity_W + gamma[traj_index] * (states_.at(traj_index).angular_velocity_W - states_.at(traj_index - 1).angular_velocity_W);
      Eigen::Vector3d angular_velocity_des_B(mavros::ftf::transform_frame_enu_baselink(angular_velocity_des_W, qref_enu));
      angular_velocity_des_B = mavros::ftf::transform_frame_baselink_aircraft(angular_velocity_des_W);

      uref(0) = angular_velocity_des_B.x();
      uref(1) = angular_velocity_des_B.y();
      uref(2) = angular_velocity_des_B.z();
      uref(3) = -1 * thrust.norm();

      // std::cout << "xref" << xref << std::endl;
      // std::cout << "uref" << uref << std::endl;
      return false;
    }
  }

  bool LQR_Euler::setStaticReference(state_vector_t &xref, control_vector_t &uref, Eigen::Vector4d &flat_states)
  {
    Eigen::Vector3d position_ref_enu;
    position_ref_enu << flat_states(0),
        flat_states(1),
        flat_states(2);
    Eigen::Vector3d position_ref_ned(mavros::ftf::transform_frame_enu_ned(position_ref_enu));

    xref(0) = position_ref_ned.x();
    xref(1) = position_ref_ned.y();
    xref(2) = position_ref_ned.z();

    /*std::cout << "X:" << xref(0) << "\n";
    std::cout << "Y:" << xref(1) << "\n";
    std::cout << "Z:" << xref(2) << "\n";*/

    Eigen::Quaterniond qref_enu(mavros::ftf::quaternion_from_rpy(0, 0, flat_states(3)));
    Eigen::Quaterniond qref_ned(mavros::ftf::transform_orientation_baselink_aircraft(mavros::ftf::transform_orientation_enu_ned(qref_enu)));
    Eigen::Vector3d rpy_ref_ned(quaternion_to_rpy_wrap(qref_ned));

    xref(3) = 0;
    xref(4) = 0;
    xref(5) = rpy_ref_ned.z();

    // std::cout << "Yaw:" << xref(5) << "\n";

    xref(6) = 0;
    xref(7) = 0;
    xref(8) = 0;

    uref(0) = 0;
    uref(1) = 0;
    uref(2) = 0;
    uref(3) = -9.81;

    double ref_pos_diff = sqrt(pow(x_(0) - xref(0), 2) + pow(x_(1) - xref(1), 2) + pow(x_(2) - xref(2), 2));
    double ref_yaw_diff = abs(x_(5) - xref(5));

    if (ref_pos_diff < 0.05 && ref_yaw_diff < 0.05)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void LQR_Euler::generateTrajectory(mav_msgs::EigenTrajectoryPoint::Vector &states, int trajectory_type)
  {
    switch (trajectory_type)
    {
    case POLYNOMIAL:
    {
      std::vector<double> segment_times;
      mav_trajectory_generation::Vertex::Vector vertices;
      const int deriv_to_opt = mav_trajectory_generation::derivative_order::SNAP;
      mav_trajectory_generation::Trajectory trajectory;
      Eigen::Vector3d rpy_enu = (quaternion_to_rpy_wrap(q_enu_));

      mav_trajectory_generation::Vertex start(dimension), interm1(dimension), interm2(dimension), interm3(dimension), interm4(dimension), interm5(dimension), interm6(dimension), end(dimension);

      // start.makeStartOrEnd(Eigen::Vector3d(0,0,-0.1), deriv_to_opt);
      // vertices.push_back(start);

      interm1.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 0, 1));
      interm1.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0.5, 0, 0));
      vertices.push_back(interm1);

      interm2.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1, 2, 1.5));
      vertices.push_back(interm2);

      interm3.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 4, 2));
      vertices.push_back(interm3);

      interm4.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1, 2, 1.5));
      vertices.push_back(interm4);

      interm5.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 0, 1));
      interm5.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0.5, 0, 0));
      vertices.push_back(interm5);

      // end.makeStartOrEnd(Eigen::Vector3d(-10,-25,1), deriv_to_opt);
      // vertices.push_back(end);

      segment_times = estimateSegmentTimes(vertices, v_max, a_max);

      const int N = 10;
      mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
      opt.setupFromVertices(vertices, segment_times, deriv_to_opt);
      opt.solveLinear();

      opt.getTrajectory(&trajectory);

      bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

      double distance = 1.0;
      std::string frame_id = "map";
      mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers);
      closed_traj_ = true;
    }
    break;

    case CIRCLE:
    {
      CirTrajectory circle_traj(1.0, 2.0, 1.0); // INPUTS: w (rad/s), R (m), h (m)
      circle_traj.SampleTrajectory(0.1, &states);
      double distance = 1.0;
      std::string frame_id = "map";
      mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers);
      closed_traj_ = true;
    }
    break;
    default:
    {
    }
    }
  }

  state_vector_t LQR_Euler::getError()
  {
    return this->xerror_;
  }

  state_vector_t LQR_Euler::getPosition() // IMPLEMENTED BY US
  {
    return this->x_;
  }

  ct::core::FeedbackMatrix<nStates, nControls> LQR_Euler::getGain()
  {
    return this->Kold_;
  }

  void LQR_Euler::setOutput(control_vector_t output)
  { // rpy limit for the controller
    for (int i = 0; i < 3; i++)
    {
      if (output(i) > 2.0 || output(i) < -2.0)
      {
        if (output(i) > 2.0)
        {
          output(i) = 2.0;
        }
        else
        {
          output(i) = -2.0;
        }
      }
    }
    // thrust limit for the controller
    output(3) = -1 * (output(3));

    if (output(3) > 18.0)
    {
      output(3) = 18.0;
    }
    else if (output(3) < 2.0)
    {
      output(3) = 2.0;
    }

    this->output_ = output;
  }

  void LQR_Euler::setOutputNN(control_vector_t output)
  {
    for (int i = 0; i < 3; i++)
    {
      if (output(i) > 2.0 || output(i) < -2.0)
      {
        if (output(i) > 2.0)
        {
          output(i) = 2.0;
        }
        else
        {
          output(i) = -2.0;
        }
      }
    }
    // thrust limit for the controller
    output(3) = -1 * (output(3));

    if (output(3) > 18.0)
    {
      output(3) = 18.0;
    }
    else if (output(3) < 2.0)
    {
      output(3) = 2.0;
    }
    this->output_nn = output;
  }

  control_vector_t LQR_Euler::getOutput()
  {
    return this->output_;
  }

  state_vector_t LQR_Euler::getRefStates()
  {
    return this->xref_;
  }

  control_vector_t LQR_Euler::getTrajectoryControl()
  {
    return this->uref_;
  }

  double LQR_Euler::getLyapV()
  {
    return this->V;
  }

  double LQR_Euler::getTimerNN()
  {
    return this->timeNN;
  }

  double LQR_Euler::getTimerLQR()
  {
    return this->timeLQR;
  }

  double LQR_Euler::getMotorCmd()
  {
    double u;
    u = LQR_Euler::output_(3) / 17;
    return u;
  }

  double LQR_Euler::getMotorCmdNN()
  {
    double u;
    u = LQR_Euler::output_(3) / 17;
    return u;
  }

  bool LQR_Euler::readParameters()
  {
    if (!nodeHandle_.getParam("lqr_controller_euler/thrust_model/k1", k1_))
      return false;
    if (!nodeHandle_.getParam("lqr_controller_euler/thrust_model/k2", k2_))
      return false;
    if (!nodeHandle_.getParam("lqr_controller_euler/thrust_model/k3", k3_))
      return false;

    return true;
  }

  control_vector_t LQR_Euler::getOutputNN()
  { // IMPLEMENTED BY US
    return this->output_nn;
  }

  Eigen::VectorXd LQR_Euler::neuralNetworkModel(const Eigen::VectorXd x)
  {
    Eigen::VectorXd output = x;

    for (int i = 0; i < W.size(); i++)
    {
      output = W[i] * output;
      for (int j = 0; j < B[i].size(); j++)
      {
        output(j) = output(j) + B[i](j);
        if (i < (W.size() - 1) && output(j) < 0)
          output(j) = output(j) * 0.01;
      }
    }
    return output;
  }

  void LQR_Euler::neuralNetwork(const state_vector_t x)
  {
    Eigen::Matrix<double, 9, 1> x_eq;
    Eigen::Matrix<double, 4, 1> control_eq;
    x_eq << 0, 0, -1.0, 0, 0, 1.57, 0, 0, 0;
    control_eq << 0, 0, 0, 0.605;

    if (isQuaternion)
    {
      Eigen::VectorXd xQ = stateToQuaternion(x);
      Eigen::VectorXd x_eqQ = stateToQuaternion(x_eq);
      Eigen::VectorXd control_eqQ = actionToQuaternion(control_eq);
      this->output_nn = quaternionToAction(neuralNetworkModel(xQ) - neuralNetworkModel(x_eqQ) + control_eqQ);
    }
    else
      this->output_nn = neuralNetworkModel(x) - neuralNetworkModel(x_eq) + control_eq;
  }

  Eigen::VectorXd LQR_Euler::stateToQuaternion(const state_vector_t x)
  {
    Eigen::VectorXd q(12);
    q[0] = 0.5;
    q[1] = x[0];
    q[2] = x[1];
    q[3] = x[2];
    q[4] = 0.5;
    q[5] = x[3];
    q[6] = x[4];
    q[7] = x[5];
    q[8] = 0.5;
    q[9] = x[6];
    q[10] = x[7];
    q[11] = x[8];
    return q;
  }

  Eigen::VectorXd LQR_Euler::actionToQuaternion(const control_vector_t u)
  {
    Eigen::VectorXd q(8);
    if (numOutNeurons == 1)
      return u;
    q[0] = 0.5;
    q[1] = u[0];
    q[2] = u[1];
    q[3] = u[2];
    q[4] = u[3];
    q[5] = 0;
    q[6] = 0;
    q[7] = 0;
    return q;
  }

  Eigen::VectorXd LQR_Euler::quaternionToAction(const Eigen::VectorXd q)
  {
    control_vector_t u;
    if (numOutNeurons == 1)
      return q;
    u[0] = q[1];
    u[1] = q[2];
    u[2] = q[3];
    u[3] = q[4];
    return u;
  }

  void LQR_Euler::eulerToQuat(double *rpy_in, double *quat_in)
  {

    quat_in[0] = cos(rpy_in[2] * 0.5) * cos(rpy_in[1] * 0.5) * cos(rpy_in[0] * 0.5) + sin(rpy_in[2] * 0.5) * sin(rpy_in[1] * 0.5) * sin(rpy_in[0] * 0.5); // w
    quat_in[1] = cos(rpy_in[2] * 0.5) * cos(rpy_in[1] * 0.5) * sin(rpy_in[0] * 0.5) - sin(rpy_in[2] * 0.5) * sin(rpy_in[1] * 0.5) * cos(rpy_in[0] * 0.5); // x
    quat_in[2] = sin(rpy_in[2] * 0.5) * cos(rpy_in[1] * 0.5) * sin(rpy_in[0] * 0.5) + cos(rpy_in[2] * 0.5) * sin(rpy_in[1] * 0.5) * cos(rpy_in[0] * 0.5); // y
    quat_in[3] = sin(rpy_in[2] * 0.5) * cos(rpy_in[1] * 0.5) * cos(rpy_in[0] * 0.5) - cos(rpy_in[2] * 0.5) * sin(rpy_in[1] * 0.5) * sin(rpy_in[0] * 0.5); // z
  }

  void LQR_Euler::copy_Eigen_to_double(double *target, Eigen::VectorXd &source, int length)
  {
    for (int i = 0; i < length; i++)
    {
      target[i] = source(i);
    }
  }

  void LQR_Euler::lyapunov_net(const state_vector_t x)
  {

    Eigen::Matrix<double, 7, 9> W1 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Scrivania/biorobotics_group/Lyap_stable_drone/quadrotorpx4/Test_08-03-2024_19-10-18_pixhawk_quad/199/W1_post_train_lyapunov_08_03_traj_back_ori_seed0.csv");
    Eigen::Matrix<double, 7, 7> W2 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Scrivania/biorobotics_group/Lyap_stable_drone/quadrotorpx4/Test_08-03-2024_19-10-18_pixhawk_quad/199/W2_post_train_lyapunov_08_03_traj_back_ori_seed0.csv");
    Eigen::Matrix<double, 1, 7> W3 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Scrivania/biorobotics_group/Lyap_stable_drone/quadrotorpx4/Test_08-03-2024_19-10-18_pixhawk_quad/199/W3_post_train_lyapunov_08_03_traj_back_ori_seed0.csv");

    Eigen::Matrix<double, 7, 1> B1 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Scrivania/biorobotics_group/Lyap_stable_drone/quadrotorpx4/Test_08-03-2024_19-10-18_pixhawk_quad/199/b1_post_train_lyapunov_08_03_traj_back_ori_seed0.csv");
    Eigen::Matrix<double, 7, 1> B2 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Scrivania/biorobotics_group/Lyap_stable_drone/quadrotorpx4/Test_08-03-2024_19-10-18_pixhawk_quad/199/b2_post_train_lyapunov_08_03_traj_back_ori_seed0.csv");
    Eigen::Matrix<double, 1, 1> B3 = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Scrivania/biorobotics_group/Lyap_stable_drone/quadrotorpx4/Test_08-03-2024_19-10-18_pixhawk_quad/199/b3_post_train_lyapunov_08_03_traj_back_ori_seed0.csv");

    Eigen::Matrix<double, 15, 9> R = load_csv<Eigen::MatrixXd>("/home/alessiaphd/Scrivania/biorobotics_group/Lyap_stable_drone/quadrotorpx4/Test_08-03-2024_19-10-18_pixhawk_quad/199/R_post_train_lyapunov_08_03_traj_back_ori_seed0.csv");

    Eigen::Matrix<double, 1, 1> V_lambda;
    V_lambda << 0.7;

    Eigen::MatrixXd hid1, hid2, hid1_eq, hid2_eq;
    Eigen::MatrixXd output, output2;
    Eigen::Matrix<double, 9, 1> x_eq;

    state_vector_t x_eq_norm;
    x_eq_norm(0) = 0;
    x_eq_norm(1) = 0;
    x_eq_norm(2) = -1.0;
    x_eq_norm(3) = 0;
    x_eq_norm(4) = 0;
    x_eq_norm(5) = 1.57;
    x_eq_norm(6) = 0;
    x_eq_norm(7) = 0;
    x_eq_norm(8) = 0;

    x_eq << 0, 0, -1.0, 0, 0, 1.57, 0, 0, 0;

    // Obtaining model(state)
    hid1 = W1 * x;

    for (int i = 0; i < 7; i++)
    {
      hid1(i) = hid1(i) + B1(i);
    }

    for (int i = 0; i < 7; i++)
    {
      if (hid1(i) < 0)
      { // leaky ReLU
        hid1(i) = hid1(i) * 0.1;
      }
    }

    hid2 = W2 * hid1;

    for (int i = 0; i < 7; i++)
    {
      hid2(i) = hid2(i) + B2(i);
    }

    for (int i = 0; i < 7; i++)
    {
      if (hid2(i) < 0)
      { // leaky ReLU
        hid2(i) = hid2(i) * 0.1;
      }
    }

    output = (W3 * hid2) + B3;

    // obtaining model(x_eq)

    hid1_eq = W1 * x_eq;

    for (int i = 0; i < 7; i++)
    {
      hid1_eq(i) = hid1_eq(i) + B1(i);
    }

    for (int i = 0; i < 7; i++)
    {
      if (hid1_eq(i) < 0)
      { // leaky ReLU
        hid1_eq(i) = hid1_eq(i) * 0.1;
      }
    }

    hid2_eq = W2 * hid1_eq;

    for (int i = 0; i < 7; i++)
    {
      hid2_eq(i) = hid2_eq(i) + B2(i);
    }

    for (int i = 0; i < 7; i++)
    {
      if (hid2_eq(i) < 0)
      { // leaky ReLU
        hid2_eq(i) = hid2_eq(i) * 0.1;
      }
    }

    output2 = (W3 * hid2_eq) + B3;

    Eigen::MatrixXd tmp;
    tmp = output - output2 + V_lambda * (R * (x - x_eq_norm)).lpNorm<1>();

    Eigen::Map<Eigen::VectorXd> M(tmp.data(), 1, 1);
    Eigen::VectorXd M2(M);

    double V_fin[0];
    copy_Eigen_to_double(V_fin, M2, 1);

    this->V = V_fin[0];
  }

} /* namespace LQR */
