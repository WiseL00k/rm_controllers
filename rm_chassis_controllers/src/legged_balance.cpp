//
// Created by kook on 1/15/25.
//

#include "rm_chassis_controllers/legged_balance.h"
#include "rm_chassis_controllers/vmc/leg_conv.h"
#include "rm_chassis_controllers/vmc/leg_pos.h"
#include "rm_chassis_controllers/vmc/leg_spd.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_msgs/BalanceState.h>
#include <angles/angles.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

namespace rm_chassis_controllers
{
bool LeggedBalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("base_imu")));
  std::string left_wheel_joint, right_wheel_joint, left_front_leg_joint, left_back_leg_joint, right_front_leg_joint,
      right_back_leg_joint;
  if (!controller_nh.getParam("left/wheel_joint", left_wheel_joint) ||
      !controller_nh.getParam("left/front_leg_joint", left_front_leg_joint) ||
      !controller_nh.getParam("left/back_leg_joint", left_back_leg_joint) ||
      !controller_nh.getParam("right/wheel_joint", right_wheel_joint) ||
      !controller_nh.getParam("right/front_leg_joint", right_front_leg_joint) ||
      !controller_nh.getParam("right/back_leg_joint", right_back_leg_joint) ||
      !controller_nh.getParam("pitch_protect_angle", pitchProtectAngle_) ||
      !controller_nh.getParam("roll_protect_angle", rollProtectAngle_))
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_wheel_joint);
  left_front_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_front_leg_joint);
  left_back_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_back_leg_joint);
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_wheel_joint);
  right_front_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_front_leg_joint);
  right_back_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_back_leg_joint);
  joint_handles_.push_back(left_wheel_joint_handle_);
  joint_handles_.push_back(right_wheel_joint_handle_);
  if (!controller_nh.getParam("vmc_bias_angle", vmc_bias_angle_))
  {
    ROS_ERROR("Params vmc_bias_angle doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue xml_coeff;
  if (!controller_nh.getParam("coeff", xml_coeff))
  {  // 注意参数路径
    ROS_ERROR("Failed to load coefficients!");
    return -1;
  }
  // 遍历YAML条目
  int row = 0;
  for (auto& entry : xml_coeff)
  {
    if (entry.second.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.second.size() != 4)
    {
      ROS_WARN("Invalid data format at %s", entry.first.c_str());
      continue;
    }

    for (int col = 0; col < 4; ++col)
    {
      if (entry.second[col].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {
        coeff[row][col] = static_cast<double>(entry.second[col]);
      }
      else if (entry.second[col].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        coeff[row][col] = static_cast<int>(entry.second[col]);
      }
    }
    row++;
  }

  if (controller_nh.hasParam("pid_left_leg"))
  {
    if (!pid_left_leg_.init(ros::NodeHandle(controller_nh, "pid_left_leg")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_right_leg"))
  {
    if (!pid_right_leg_.init(ros::NodeHandle(controller_nh, "pid_right_leg")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_theta_diff"))
  {
    if (!pid_theta_diff_.init(ros::NodeHandle(controller_nh, "pid_theta_diff")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_roll"))
  {
    if (!pid_roll_.init(ros::NodeHandle(controller_nh, "pid_roll")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_yaw_pos"))
  {
    if (!pid_yaw_pos_.init(ros::NodeHandle(controller_nh, "pid_yaw_pos")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_yaw_spd"))
  {
    if (!pid_yaw_spd_.init(ros::NodeHandle(controller_nh, "pid_yaw_spd")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_center_gravity"))
  {
    if (!pid_center_gravity_.init(ros::NodeHandle(controller_nh, "pid_center_gravity")))
    {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_length_diff"))
  {
    if (!pid_length_diff_.init(ros::NodeHandle(controller_nh, "pid_length_diff")))
    {
      return false;
    }
  }

  // Slippage detection
  A_ << 1, 0, 0, 1;
  H_ << 1, 0, 0, 1;
  Q_ << 0.5, 0, 0, 0.5;
  R_ << 100, 0, 0, 100;
  B_.setZero();
  X_.setZero();
  U_.setZero();
  kalmanFilterPtr_ = std::make_shared<KalmanFilter<double>>(A_, B_, H_, Q_, R_);
  kalmanFilterPtr_->clear(X_);

  left_averFNPtr_ = std::make_shared<MovingAverageFilter<double>>(5);
  right_averFNPtr_ = std::make_shared<MovingAverageFilter<double>>(5);

  // sub
  leg_cmd_subscriber_ =
      root_nh.subscribe<rm_msgs::LegCmd>("/leg_cmd", 1, &LeggedBalanceController::legCmdCallback, this);
  legGroundSupportForcePublisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("leg_support_force", 1);
  observationPublisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("state", 1);
  inputPublisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("input", 1);
  unStickPublisher_ =
      controller_nh.advertise<std_msgs::Bool>("/controllers/legged_balance_controller/unstick/two_leg_unstick", 1);
  balance_mode_ = BalanceMode::NORMAL;
  //  leg_cmd_.leg_length = 0.15;
  return true;
}

void LeggedBalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  updateEstimation(time, period);

  if (abs(pitch_) > pitchProtectAngle_ || abs(roll_) > rollProtectAngle_)
  {
    balance_mode_ = BalanceMode::SHUTDOWN;
  }

  switch (balance_mode_)
  {
    case BalanceMode::NORMAL:
    {
      normal(time, period);
      break;
    }
    case BalanceMode::BLOCK:
    {
      block(time, period);
      break;
    }
    case BalanceMode::SHUTDOWN:
    {
      shutDown(time, period);
      break;
    }
  }

  // Power limit
  //  double limit = balanceInterface_->getLeggedBalanceControlCmd()->getPowerLimit();
  //  double a = 0., b = 0., c = 0.;  // Three coefficients of a quadratic equation in one variable
  //  for (const auto& joint : powerLimitJointHandles_) {
  //    double cmd_effort = joint.getCommand();
  //    double real_vel = joint.getVelocity();
  //    a += square(cmd_effort);
  //    b += cmd_effort * real_vel;
  //    c += abs(real_vel);
  //  }
  //  a *= params_.powerCoeffEffort_;
  //  c = c * params_.powerCoeffVel_ + params_.powerOffset_ - limit;  // offset different from rm_chassis_controller
  //  double zoom = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
  //  for (auto joint : powerLimitJointHandles_) {
  //    joint.setCommand(zoom <= 1 && zoom > 0 ? joint.getCommand() * zoom : joint.getCommand());
  //  }
}

void LeggedBalanceController::normal(const ros::Time& time, const ros::Duration& period)
{
  if (balance_state_changed_)
  {
    ROS_INFO("[balance] Enter NOMAl");
    balance_state_changed_ = false;
  }

  // PID
  double T_theta_diff = pid_theta_diff_.computeCommand(left_pos_[1] - right_pos_[1], period);
  double F_length_diff = pid_length_diff_.computeCommand(left_pos_[0] - right_pos_[0], period);
  double leg_aver = (left_pos_[0] + right_pos_[0]) / 2;

  // set target state
  yaw_des_ += vel_cmd_.z * period.toSec();
  position_des_ += vel_cmd_.x * period.toSec();
  Eigen::Matrix<double, CONTROL_DIM, 1> u;
  auto x = x_;
  x(2) -= position_des_;
  if (state_ != RAW)
    x(3) -= vel_cmd_.x;
  k_ = getK(leg_aver);
  u = k_ * (-x);
  pid_yaw_pos_.computeCommand((yaw_des_ - yaw_total_), period);
  pid_yaw_spd_.computeCommand((pid_yaw_pos_.getCurrentCmd() - angular_vel_base_.z), period);

  // Leg control
  Eigen::Matrix<double, 2, 3> j;
  Eigen::Matrix<double, 3, 1> p;
  Eigen::Matrix<double, 2, 1> F_leg, F_bl;
  double F_roll, F_gravity, F_inertial;
  F_leg[0] = pid_left_leg_.computeCommand(leg_cmd_.leg_length - leg_aver, period) - F_length_diff;
  F_leg[1] = pid_right_leg_.computeCommand(leg_cmd_.leg_length - leg_aver, period) + F_length_diff;
  F_roll = pid_roll_.computeCommand(0 - roll_, period);
  F_inertial = 0;
  F_gravity = 1. / 2 * body_mass_ * g_;
  // clang-format off
  j << 1, cos(left_pos_[1]), -1,
      -1, cos(right_pos_[1]), 1;
  // clang-format on
  p << F_roll, F_gravity, F_inertial;
  F_bl = j * p + F_leg;

  double left_T[2], right_T[2];
  leg_conv(F_bl[0], -u(1) - T_theta_diff, left_angle[0], left_angle[1], left_T);
  leg_conv(F_bl[1], -u(1) + T_theta_diff, right_angle[0], right_angle[1], right_T);

  unstickDetection(time, period, x_[0], x_[1], imu_handle_.getLinearAcceleration()[2], left_spd_[0], left_pos_[0],
                   F_bl[0], -u(1) - T_theta_diff, right_spd_[0], right_pos_[0], F_bl[1], -u(1) + T_theta_diff);
  left_wheel_joint_handle_.setCommand(u(0) - pid_yaw_spd_.getCurrentCmd());
  right_wheel_joint_handle_.setCommand(u(0) + pid_yaw_spd_.getCurrentCmd());

  if (start_)
  {
    if (abs(x_[1]) < 0.1 && abs(x_[5]) < 0.1 && (ros::Time::now() - start_time_) > ros::Duration(0.5))
    {
      start_ = false;
    }
  }
  else
  {
    if (leg_aver > 0.10 && leg_aver < 0.4)
    {
      if (left_unstick_)
        left_wheel_joint_handle_.setCommand(0.);
      if (right_unstick_)
        right_wheel_joint_handle_.setCommand(0.);
    }
    left_front_leg_joint_handle_.setCommand(left_T[1]);
    right_front_leg_joint_handle_.setCommand(right_T[1]);
    left_back_leg_joint_handle_.setCommand(left_T[0]);
    right_back_leg_joint_handle_.setCommand(right_T[0]);
  }
  //  std::cout << leg_cmd_.leg_length << std::endl;
  //  std::cout << leg_aver << std::endl;

  std_msgs::Float64MultiArray input_;
  input_.data.push_back(u(0));
  input_.data.push_back(u(1));
  input_.data.push_back(u(0) - pid_yaw_spd_.getCurrentCmd());
  input_.data.push_back(u(0) + pid_yaw_spd_.getCurrentCmd());
  inputPublisher_.publish(input_);
}

void LeggedBalanceController::shutDown(const ros::Time& time, const ros::Duration& period)
{
  left_wheel_joint_handle_.setCommand(0);
  right_wheel_joint_handle_.setCommand(0);
  left_front_leg_joint_handle_.setCommand(0);
  right_front_leg_joint_handle_.setCommand(0);
  left_back_leg_joint_handle_.setCommand(0);
  right_back_leg_joint_handle_.setCommand(0);
  ROS_INFO_THROTTLE(3, "[balance] Enter ShutDown");

  if (abs(pitch_) <= pitchProtectAngle_ && abs(roll_) <= rollProtectAngle_)
  {
    balance_mode_ = BalanceMode::NORMAL;
    x_[2] = 0;
    ROS_INFO_THROTTLE(3, "[balance] Exit ShutDown");
  }
}

void LeggedBalanceController::updateEstimation(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  try
  {
    tf2::doTransform(gyro, angular_vel_base_,
                     robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2imu, imu2base, odom2base;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    left_wheel_joint_handle_.setCommand(0.);
    right_wheel_joint_handle_.setCommand(0.);
    left_front_leg_joint_handle_.setCommand(0.);
    left_back_leg_joint_handle_.setCommand(0.);
    right_front_leg_joint_handle_.setCommand(0.);
    right_back_leg_joint_handle_.setCommand(0.);
    return;
  }
  tf2::Quaternion odom2imu_quaternion;
  tf2::Vector3 odom2imu_origin;
  odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                               imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
  odom2imu_origin.setValue(0, 0, 0);
  odom2imu.setOrigin(odom2imu_origin);
  odom2imu.setRotation(odom2imu_quaternion);
  odom2base = odom2imu * imu2base;
  quatToRPY(toMsg(odom2base).rotation, roll_, pitch_, yaw_);

  // vmc
  left_angle[0] =
      vmc_bias_angle_ + left_back_leg_joint_handle_.getPosition();  // [0]:back_vmc_joint [1]:front_vmc_joint
  left_angle[1] = left_front_leg_joint_handle_.getPosition() + M_PI - vmc_bias_angle_;
  right_angle[0] = vmc_bias_angle_ + right_back_leg_joint_handle_.getPosition();
  right_angle[1] = right_front_leg_joint_handle_.getPosition() + M_PI - vmc_bias_angle_;
  leg_pos(left_angle[0], left_angle[1], left_pos_);
  leg_pos(right_angle[0], right_angle[1], right_pos_);
  leg_spd(left_back_leg_joint_handle_.getVelocity(), left_front_leg_joint_handle_.getVelocity(), left_angle[0],
          left_angle[1], left_spd_);
  leg_spd(right_back_leg_joint_handle_.getVelocity(), right_front_leg_joint_handle_.getVelocity(), right_angle[0],
          right_angle[1], right_spd_);

  // Slippage_detection
  double leftWheelVel = (left_wheel_joint_handle_.getVelocity() - angular_vel_base_.z + left_spd_[0]) * wheel_radius_;
  double rightWheelVel =
      (right_wheel_joint_handle_.getVelocity() + angular_vel_base_.z + right_spd_[0]) * wheel_radius_;
  double leftWheelVelAbsolute =
      leftWheelVel + left_pos_[0] * left_spd_[1] * cos(left_pos_[1]) + left_spd_[0] * sin(left_pos_[1]);
  double rightWheelVelAbsolute =
      rightWheelVel + right_pos_[0] * right_spd_[1] * cos(right_pos_[1]) + right_spd_[0] * sin(right_pos_[1]);

  double wheel_vel_aver = (leftWheelVelAbsolute + rightWheelVelAbsolute) / 2.;
  if (i >= sample_times_)
  {  // oversampling
    i = 0;
    X_(0) = wheel_vel_aver;
    X_(1) = imu_handle_.getLinearAccelerationCovariance()[0];
    kalmanFilterPtr_->predict(U_);
    kalmanFilterPtr_->update(X_);
  }
  else
  {
    kalmanFilterPtr_->predict(U_);
    i++;
  }
  auto x_hat = kalmanFilterPtr_->getState();

  double yaw_last = yaw_total_;
  yaw_total_ = yaw_last + angles::shortest_angular_distance(yaw_last, yaw_);
  // update state
  x_[0] = (left_pos_[1] + right_pos_[1]) / 2 + pitch_;
  x_[1] = (left_spd_[1] + right_spd_[1]) / 2 + angular_vel_base_.y;
  x_[2] = (left_wheel_joint_handle_.getPosition() + right_wheel_joint_handle_.getPosition()) / 2 * wheel_radius_;
  x_[3] = x_hat(0);
  x_[4] = pitch_;
  x_[5] = angular_vel_base_.y;

  std_msgs::Float64MultiArray state_;
  state_.data.push_back(x_[0]);
  state_.data.push_back(x_[1]);
  state_.data.push_back(x_[2]);
  state_.data.push_back(x_[3]);
  state_.data.push_back(x_[4]);
  state_.data.push_back(x_[5]);
  observationPublisher_.publish(state_);
}

void LeggedBalanceController::unstickDetection(const ros::Time& time, const ros::Duration& period, const double& theta,
                                               const double& dtheta, const double& ddzm, const double& left_dL0,
                                               const double& left_L0, const double& left_F, const double& left_Tp,
                                               const double& right_dL0, const double& right_L0, const double& right_F,
                                               const double& right_Tp)
{
  static double left_FN, left_P, left_ddzw(0), left_ddL0(0), g(9.81), mw(0.8), lastLeftLegDL0(0), lastLeftLegDDL0(0),
      lastLeftLegDDTheta(0), left_ddtheta(0), lastLeftLegDTheta(0), lpfRatio(0.5);
  static double right_FN, right_P, right_ddzw(0), right_ddL0(0), lastRightLegDL0(0), lastRightLegDDL0(0),
      lastRightLegDDTheta(0), right_ddtheta(0), lastRightLegDTheta(0);

  left_ddL0 = (left_dL0 - lastLeftLegDL0) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastLeftLegDDL0;
  left_ddtheta = (dtheta - lastLeftLegDTheta) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastLeftLegDDTheta;
  left_P = left_F * cos(theta) + (left_Tp * sin(theta) / left_L0);
  left_ddzw = ddzm + g - left_ddL0 * cos(theta) + 2 * left_dL0 * dtheta * sin(theta) +
              left_L0 * left_ddtheta * sin(theta) + left_L0 * (dtheta * dtheta * cos(theta));
  left_FN = mw * left_ddzw + mw * g + (left_P);
  lastLeftLegDDL0 = left_ddL0;
  lastLeftLegDL0 = left_dL0;
  lastLeftLegDTheta = dtheta;
  lastLeftLegDDTheta = left_ddtheta;

  right_ddL0 = (right_dL0 - lastRightLegDL0) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastRightLegDDL0;
  right_ddtheta = (dtheta - lastRightLegDTheta) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastRightLegDDTheta;
  right_P = right_F * cos(theta) + (right_Tp * sin(theta) / right_L0);
  right_ddzw = ddzm + g - right_ddL0 * cos(theta) + 2 * right_dL0 * dtheta * sin(theta) +
               right_L0 * right_ddtheta * sin(theta) + right_L0 * (dtheta * dtheta * cos(theta));
  right_FN = mw * right_ddzw + mw * g + (right_P);
  lastRightLegDDL0 = right_ddL0;
  lastRightLegDL0 = right_dL0;
  lastRightLegDTheta = dtheta;
  lastRightLegDDTheta = right_ddtheta;

  left_averFNPtr_->input(left_FN);
  right_averFNPtr_->input(right_FN);
  left_FN = left_averFNPtr_->output();
  right_FN = right_averFNPtr_->output();

  std_msgs::Float64MultiArray support;
  //  support.data.push_back(left_ddzw);
  //  support.data.push_back(left_P);
  //  support.data.push_back(left_F);
  //  support.data.push_back(left_Tp);
  support.data.push_back(left_FN);
  support.data.push_back(right_FN);

  std_msgs::Bool unstickFlag;

  if (left_FN < 10)
    left_unstick_ = true;
  //    x_[2] = 0;
  else
    left_unstick_ = false;
  if (right_FN < 10)
    right_unstick_ = true;
  else
    right_unstick_ = false;
  if (left_unstick_ && right_unstick_)
  {
    unstickFlag.data = 1;
  }
  else
  {
    unstickFlag.data = 0;
  }
  legGroundSupportForcePublisher_.publish(support);
  unStickPublisher_.publish(unstickFlag);
}

void LeggedBalanceController::starting(const ros::Time&)
{
  start_ = true;
  start_time_ = ros::Time::now();
}

geometry_msgs::Twist LeggedBalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = x_[3];
  twist.angular.z = angular_vel_base_.z;
  return twist;
}

void LeggedBalanceController::legCmdCallback(const rm_msgs::LegCmdConstPtr& msg)
{
  //  std::lock_guard<std::mutex> lock(legCmdMutex_);
  leg_cmd_ = *msg;
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::LeggedBalanceController, controller_interface::ControllerBase)
