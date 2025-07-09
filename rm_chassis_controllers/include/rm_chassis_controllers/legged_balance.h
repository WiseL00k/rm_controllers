//
// Created by kook on 1/15/25.
//

#pragma once

#include <eigen3/Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include "rm_chassis_controllers/chassis_base.h"
#include "rm_common/filters/kalman_filter.h"
#include "rm_msgs/LegCmd.h"
#include "rm_common/LQRController.h"
#include "rm_chassis_controllers/vmc/VMC.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class LeggedBalanceController
  : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                       hardware_interface::EffortJointInterface>
{
  // clang-format off
  enum BalanceMode {NORMAL, BLOCK, RECOVERY, SITDOWN, SHUTDOWN};
  enum JumpState {IDLE, LEG_RETRACTION, JUMP_UP, OFF_GROUND};
  // clang-format on

public:
  LeggedBalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override
  {
    start_ = false;
  }

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void normal(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);
  void sitDown(const ros::Time& time, const ros::Duration& period);
  void recovery(const ros::Time& time, const ros::Duration& period);
  void shutDown(const ros::Time& time, const ros::Duration& period);

  void updateEstimation(const ros::Time& time, const ros::Duration& period);
  //  void unstickDetection();

  void unstickDetection(const ros::Time& time, const ros::Duration& period, const double& left_theta,
                        const double& left_dtheta, const double& right_theta, const double& right_dtheta,
                        const double& ddzm, const double& left_dL0, const double& left_L0, const double& left_F,
                        const double& left_Tp, const double& right_dL0, const double& right_L0, const double& right_F,
                        const double& right_Tp);
  geometry_msgs::Twist odometry() override;
  void follow(const ros::Time& time, const ros::Duration& period) override;
  static const int STATE_DIM = 10;
  static const int CONTROL_DIM = 4;

  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_{}, x;
  double vmc_bias_angle_, left_angle[2], right_angle[2], left_dangle[2], right_dangle[2];
  double leg_aver = 0.10;
  double wheel_radius_ = 0.06, wheel_track_ = 0.49;
  double body_mass_ = 18.7, leg_mass_ = 0.618, g_ = 9.81;
  double position_des_ = 0;
  double position_offset_ = 0.;
  double position_clear_threshold_ = 0.;
  double yaw_des_ = 0;

  int balance_mode_ = BalanceMode::NORMAL;
  ros::Time block_time_, last_block_time_, start_time_;
  ros::Time maybeOverturnTime_, lastSitDownTime_;
  double block_angle_, block_duration_, block_velocity_, block_effort_, anti_block_effort_, block_overtime_;
  double pitchProtectAngle_, rollProtectAngle_, legProtectLength_, legProtectAngle_;
  double left_wheel_torque_, right_wheel_torque_;
  double torque_wheel_k_ = 0.3;
  bool balance_state_changed_ = false, maybe_block_ = false;
  bool left_unstick_ = false, right_unstick_ = false;
  bool start_ = false, move_flag_ = false;
  bool maybeOverturn_ = false, overturnStateChanged_ = false;

  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_, left_front_leg_joint_handle_,
      left_back_leg_joint_handle_, right_front_leg_joint_handle_, right_back_leg_joint_handle_;

  geometry_msgs::Vector3 angular_vel_base_;
  double roll_{}, pitch_{}, yaw_{}, yaw_total_{}, yaw_last{};

  control_toolbox::Pid pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_length_diff_, pid_roll_, pid_center_gravity_,
      pid_yaw_pos_, pid_yaw_spd_;

  // Slippage_detection
  Eigen::Matrix<double, 2, 2> A_, B_, H_, Q_, R_;
  Eigen::Matrix<double, 2, 1> X_, U_, angularz_X_, angularz_U_;
  int i = 0, sample_times_ = 3;
  std::shared_ptr<KalmanFilter<double>> kalmanFilterPtr_, angularz_kalmanFilterPtr_;
  std::shared_ptr<MovingAverageFilter<double>> left_averFNPtr_, right_averFNPtr_;
  std::shared_ptr<RampFilter<double>> vel_FilterPtr_;
  // Jump
  int jumpState_ = JumpState::IDLE;
  ros::Time lastJumpTime_;
  int jumpTime_;
  double jumpOverTime_, p1_, p2_, p3_, p4_;

  // pub
  ros::Publisher legLengthPublisher_, legPendulumSupportForcePublisher_, legGroundSupportForcePublisher_,
      leftUnStickPublisher_, rightUnStickPublisher_, unStickPublisher_, startPublisher_, recoveryPublisher_;
  ros::Publisher observationPublisher_, inputPublisher_, lqrErrorPublisher_, lqrTargetPublisher_;
  // sub
  ::ros::Subscriber leg_cmd_subscriber_;
  rm_msgs::LegCmd leg_cmd_;

  //  std::mutex legCmdMutex_;

  void legCmdCallback(const rm_msgs::LegCmdConstPtr& msg);

  double coeff[40][6];

  double coeff_debug[CONTROL_DIM][STATE_DIM]{};

  std::shared_ptr<LQRController<double>> lqrControllerPtr_;
  Eigen::Matrix<double, STATE_DIM, 1> target_, error_;

  std::shared_ptr<VMC> vmcPtr_;

  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> getK_debug()
  {
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_coeff_debug_const_leg_;
    for (int i = 0; i < CONTROL_DIM; i++)
    {
      for (int j = 0; j < STATE_DIM; j++)
      {
        k_coeff_debug_const_leg_(i, j) = coeff_debug[i][j];
      }
    }
    return k_coeff_debug_const_leg_;
  }
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> getK(double& l_l, double& l_r)
  {
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k;
    for (int i = 0; i < CONTROL_DIM; i++)
    {
      for (int j = 0; j < STATE_DIM; j++)
      {
        k(i, j) = coeff[i * 10 + j][0] + coeff[i * 10 + j][1] * l_l + coeff[i * 10 + j][2] * l_r +
                  coeff[i * 10 + j][3] * l_l * l_l + coeff[i * 10 + j][4] * l_l * l_r +
                  coeff[i * 10 + j][5] * l_r * l_r;
      }
    }
    return k;
  }
};

}  // namespace rm_chassis_controllers
