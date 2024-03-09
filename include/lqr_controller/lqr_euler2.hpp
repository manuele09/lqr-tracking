#pragma once

#include <cmath>
#include <ros/ros.h>
#include <ct/optcon/optcon.h>
#include <lqr_controller/declarations_euler.hpp>
#include <lqr_controller/quadModelParameters.hpp>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <mavros/frame_tf.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <lqr_controller/trajectory.hpp>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/RCIn.h>
#include <ros/package.h>

namespace lqr
{
  class LQR_Euler
  {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    LQR_Euler(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~LQR_Euler();

    control_vector_t getTrajectoryControl();
    state_vector_t getError();    // IMPLEMENTED BY US
    state_vector_t getPosition(); // IMPLEMENTED BY US
    ct::core::FeedbackMatrix<nStates, nControls> getGain();
    void setOutput(control_vector_t output);
    control_vector_t getOutput();
    control_vector_t getOutputNN();
    state_vector_t getRefStates();
    double getMotorCmd();
    double getMotorCmdNN();
    double getLyapV(); // IMPLEMENTED BY US
    control_vector_t neuralNetwork(const state_vector_t x);
    void computeOutputNN(const state_vector_t x);

    Eigen::Matrix<double, 7, 9> W1;
    Eigen::Matrix<double, 4, 7> W2;
    Eigen::Matrix<double, 7, 1> B1;
    Eigen::Matrix<double, 4, 1> B2;

    // Eigen::Matrix<double, 8, 9> W1;
    // Eigen::Matrix<double, 4, 8> W2;
    // Eigen::Matrix<double, 8, 1> B1;
    // Eigen::Matrix<double, 4, 1> B2;

    // Eigen::Matrix<double, 8, 12> W1;
    // Eigen::Matrix<double, 4, 8> W2;
    // Eigen::Matrix<double, 8, 1> B1;
    // Eigen::Matrix<double, 4, 1> B2;

    std::vector<Eigen::MatrixXd> W;
    std::vector<Eigen::MatrixXd> B;


    template <typename M>
    M load_csv(const std::string &path)
    {
      std::ifstream indata;
      indata.open(path);
      std::string line;
      std::vector<double> values;
      uint rows = 0;
      while (std::getline(indata, line))
      {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ','))
        {
          values.push_back(std::stod(cell));
        }
        ++rows;
      }
      return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size() / rows);
    }

    bool takeoff = true; // Used as a flag to switch the regulator

  private:
    double k1_; // quadratic first coefficient
    double k2_; // quadratic second coefficient
    double k3_; // quadratic third coefficient

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */
    void setRCIn(const mavros_msgs::RCIn::ConstPtr &msg);
    void topicCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void setStates(const nav_msgs::Odometry::ConstPtr &msg, state_vector_t &x);
    void setError(const state_vector_t &xref, const state_vector_t &x, state_vector_t &xerror);
    bool setTrajectoryReference(state_vector_t &xref, control_vector_t &uref, int trajectory_type);
    bool setStaticReference(state_vector_t &xref, control_vector_t &uref, Eigen::Vector4d &flat_states);
    // Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond &q);
    void generateTrajectory(mav_msgs::EigenTrajectoryPoint::Vector &states, int trajectory_type);
    bool readParameters();
    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber odom_sub_;
    ros::Subscriber RCIn_sub_;

    // Marker publisher
    ros::Publisher marker_pub_;

    //! State and control matrix dimensions
    const size_t state_dim = nStates;
    const size_t control_dim = nControls;

    // Trajectory
    double sampling_interval = 0.1;
    const double v_max = 6;
    const double a_max = 3;
    const int dimension = 3;
    int traj_index;
    bool ref_reached_ = false;
    bool initiated;
    std::vector<uint16_t> channels_;
    mav_msgs::EigenTrajectoryPoint::Vector states_;
    visualization_msgs::MarkerArray markers;
    bool closed_traj_;
    enum trajectory_type
    {
      POLYNOMIAL,
      CIRCLE
    };

    Eigen::Vector3d position_enu_;
    Eigen::Vector3d velocity_enu_;
    Eigen::Quaterniond q_enu_;
    Eigen::Quaterniond q_ned_;
    state_matrix_t A_;
    control_gain_matrix_t B_;
    ct::core::FeedbackMatrix<nStates, nControls> Kold_;
    ct::core::FeedbackMatrix<nStates, nControls> Knew_;
    ct::core::StateMatrix<nStates> P_Ricc;
    ros::Time callBack_;
    double init_time_;
    double V;
    state_vector_t x_;
    control_vector_t u_;
    state_vector_t xref_;
    control_vector_t uref_;
    state_vector_t xerror_;
    control_vector_t output_;
    control_vector_t output_nn; // ADDED BY US OUTPUT OF THE NEURAL NETWORK

    ct::optcon::TermQuadratic<nStates, nControls> quadraticCost_;
    ct::optcon::TermQuadratic<nStates, nControls>::state_matrix_t Q_;
    ct::optcon::TermQuadratic<nStates, nControls>::control_matrix_t R_;
    ct::optcon::LQR<nStates, nControls> lqrSolver_;

    state_matrix_t A_quadrotor(const state_vector_t &x, const control_vector_t &u);
    control_gain_matrix_t B_quadrotor(const state_vector_t &x, const control_vector_t &u);
  };

} /* namespace */
