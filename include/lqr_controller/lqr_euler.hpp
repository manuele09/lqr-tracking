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
    control_vector_t getTrajectoryControlMODIFIED(); // IMPLEMENTED BY US
    state_vector_t getError();                       // IMPLEMENTED BY US
    state_vector_t getPosition();                    // IMPLEMENTED BY US
    ct::core::FeedbackMatrix<nStates, nControls> getGain();
    void setOutput(control_vector_t output);
    void setOutputNN(control_vector_t output);
    control_vector_t getOutput();
    control_vector_t getOutputNN(); // INPLEMENTED BY US
    state_vector_t getRefStates();
    double getMotorCmd();
    double getMotorCmdNN();
    double getLyapV();                          // IMPLEMENTED BY US
    double getTimerNN();                        // IMPLEMENTED BY US
    double getTimerLQR();                       // IMPLEMENTED BY USù

    void loadNeuralNetworkConfig(const std::string &configFile);
    void loadTestConfig(const std::string &configFile);
    void neuralNetwork(const state_vector_t x); // IMPLEMENTD BY US
    Eigen::VectorXd neuralNetworkModel(const Eigen::VectorXd x);
    Eigen::VectorXd stateToQuaternion(const state_vector_t x);
    Eigen::VectorXd actionToQuaternion(const control_vector_t u);
    Eigen::VectorXd quaternionToAction(const Eigen::VectorXd q);

    void lyapunov_net(const state_vector_t x);
    void copy_Eigen_to_double(double *target, Eigen::VectorXd &source, int length);
    void eulerToQuat(double *rpy_in, double *quat_in); // COPIED FROM MINICHEETAH

    std::vector<Eigen::MatrixXd> W;
    std::vector<Eigen::MatrixXd> B;
    int numLayers;
    bool isQuaternion;
    int numOutNeurons;

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

    int position_index; // IMPLEMENTED BY US
    int x_positions[8]; // IMPLEMENTED BY US
    int y_positions[8]; // IMPLEMENTED BY US
    double init_time_;  // IMPLEMENTED BY US

    double traj_offset;                                               // IMPLEMENTED BY US
    std::chrono::time_point<std::chrono::system_clock> start_timerAB; // IMPLEMENTED BY US
    std::chrono::time_point<std::chrono::system_clock> end_timerAB;   // IMPLEMENTED BY US
    std::chrono::time_point<std::chrono::system_clock> start_timerNN; // IMPLEMENTED BY US
    std::chrono::time_point<std::chrono::system_clock> end_timerNN;   // IMPLEMENTED BY US
    std::chrono::duration<double> elapsed_secondsNN;                  // IMPLEMENTED BY US
    std::chrono::duration<double> elapsed_secondsLQR;                 // IMPLEMENTED BY US
    std::time_t end_time;
    int traj_time;                 // IMPLEMENTED BY US
    int index;                     // IMPLEMENTED BY US
    std::ifstream FileTrajPoinsts; // IMPLEMENTED BY US

    // ROA TEST

    bool lqr_test = false;
    bool nn_test = true;

    double y_val = 0.0;
    int jp = 0;
    bool firstFlight = true;
    bool firstFlight_NN = true;

    std::vector<Eigen::Vector4d> staticRefNN;
    int testDuration;    // Duration time for each of the states
    int counter;               // Index relative to the current state under test
    double logInterval; // For logging every x seconds
    double last_log_time;

    // LQR MAX DIST

    // CROCE (non ritestati ancora)
    /*double max_dist_0   = 4.0;
    double max_dist_1   = 3.0;
    double max_dist_2   = 2.0;
    double max_dist_3   = 3.0;*/

    // DIAGONALE
    double max_dist_4 = 2.0; // riprovati, massimale per LQR calmo e dopato, questo funzionerebbe a 2 ma sbatte a terra
    double max_dist_5 = 1.0; // riprovati, massimale per LQR calmo e dopato
    double max_dist_6 = 2.0; // riprovati, massimale per LQR calmo e dopato
    double max_dist_7 = 1.0; // riprovati, massimale per LQR calmo e dopato

    // NN MAX DIST

    // CROCE (non ritestati ancora)
    /*double max_dist_0_nn = 5.0;
    double max_dist_1_nn = 4.0;
    double max_dist_2_nn = 3.0;
    double max_dist_3_nn = 3.0;*/

    // DIAGONALE -> punti massimi ottenuti da rete allenata su quadrato [2,1,2,1]
    double max_dist_4_nn = 3.0;
    double max_dist_5_nn = 2.0;
    double max_dist_6_nn = 3.0;
    double max_dist_7_nn = 2.0;

    bool start_rec = false;

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
    double V;
    double timeNN;
    double timeLQR;
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
