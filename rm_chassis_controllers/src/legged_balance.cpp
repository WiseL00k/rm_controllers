//
// Created by kook on 1/15/25.
//

#include "rm_chassis_controllers/legged_balance.h"
#include "rm_chassis_controllers/vmc/leg_conv.h"
#include "rm_chassis_controllers/vmc/leg_pos.h"
#include "rm_chassis_controllers/vmc/leg_spd.h"
#include "rm_common/math_utilities.h"

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
      !controller_nh.getParam("right/back_leg_joint", right_back_leg_joint))
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("leg_protect_length", legProtectLength_) ||
      !controller_nh.getParam("leg_protect_angle", legProtectAngle_) ||
      !controller_nh.getParam("pitch_protect_angle", pitchProtectAngle_) ||
      !controller_nh.getParam("roll_protect_angle", rollProtectAngle_))
  {
    ROS_ERROR("Load param fail, check the resist of leg_protect_angle ,pitch_protect_angle, roll_protect_angle and "
              "leg_protect_length");
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

  if (!controller_nh.getParam("jumpOverTime", jumpOverTime_) || !controller_nh.getParam("p1", p1_) ||
      !controller_nh.getParam("p2", p2_) || !controller_nh.getParam("p3", p3_) || !controller_nh.getParam("p4", p4_))
  {
    ROS_ERROR("Load param fail, check the resist of jump_over_time, p1, p2, p3, p4");
    return false;
  }

  XmlRpc::XmlRpcValue xml_coeff;
  if (!controller_nh.getParam("k_coeff", xml_coeff))
  {  // 注意参数路径
    ROS_ERROR("Failed to load k_coefficients!");
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

  if (!controller_nh.getParam("power/effort_coeff", effort_coeff_) ||
      !controller_nh.getParam("power/vel_coeff", velocity_coeff_) ||
      !controller_nh.getParam("power/power_offset", power_offset_))
  {
    ROS_ERROR("Failed to load power coefficients!");
    return -1;
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
  legLengthPublisher_ = controller_nh.advertise<std_msgs::Float64>("leg_length", 1);
  balance_mode_ = BalanceMode::NORMAL;
  //  leg_cmd_.leg_length = 0.15;
  return true;
}

void LeggedBalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  updateEstimation(time, period);

  if (cmd_struct_.cmd_chassis_.mode == 4)
  {
    balance_mode_ = BalanceMode::SITDOWN;
  }

  // Check jump
  if (jumpState_ == IDLE && ros::Time::now() - lastJumpTime_ > ros::Duration(jumpOverTime_) && leg_cmd_.jump == true)
  {
    jumpState_ = JumpState::LEG_RETRACTION;
  }

  // Check overturn
  double overturnDuration = 0.2;
  if (balance_mode_ != BalanceMode::SITDOWN)
  {
    if (abs(left_x_(0) + right_x_(0)) / 2 > legProtectAngle_)
    {
      if (!maybeOverturn_)
      {
        maybeOverturnTime_ = time;
        maybeOverturn_ = true;
      }
      if ((time - maybeOverturnTime_).toSec() >= overturnDuration)
      {
        balance_mode_ = BalanceMode::SITDOWN;
        overturnStateChanged_ = true;
      }
    }
    else
    {
      maybeOverturn_ = false;
    }
  }

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
    case BalanceMode::SITDOWN:
    {
      sitDown(time, period);
      break;
    }
    case BalanceMode::SHUTDOWN:
    {
      shutDown(time, period);
      break;
    }
  }

  // Power limit
  powerLimit();
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
  leg_aver = (left_pos_[0] + right_pos_[0]) / 2;

  Eigen::Matrix<double, CONTROL_DIM, 1> left_u, right_u;
  auto left_x = left_x_, right_x = right_x_;
  // set target state

  if (!left_unstick_ && !right_unstick_)
  {
    yaw_des_ += vel_cmd_.z * period.toSec();
  }
  if (state_ != RAW)
  {
    left_x(3) -= vel_cmd_.x;
    right_x(3) -= vel_cmd_.x;
    if (!left_unstick_ && !right_unstick_)
    {
      position_des_ += vel_cmd_.x * period.toSec();
    }
  }
  left_x(2) -= position_des_;
  right_x(2) -= position_des_;
  left_k_ = getK(left_pos_[0]);
  right_k_ = getK(right_pos_[0]);
  left_u = left_k_ * (-left_x);
  right_u = right_k_ * (-right_x);
  pid_yaw_pos_.computeCommand((yaw_des_ - yaw_total_), period);
  pid_yaw_spd_.computeCommand((pid_yaw_pos_.getCurrentCmd() - angular_vel_base_.z), period);

  // Leg control
  Eigen::Matrix<double, 2, 3> j;
  Eigen::Matrix<double, 3, 1> p;
  Eigen::Matrix<double, 2, 1> F_leg, F_bl;
  double F_roll, F_gravity, F_inertial;

  if (jumpState_ == JumpState::IDLE)
  {
    F_leg[0] = pid_left_leg_.computeCommand(leg_cmd_.leg_length - leg_aver, period) - F_length_diff;
    F_leg[1] = pid_right_leg_.computeCommand(leg_cmd_.leg_length - leg_aver, period) + F_length_diff;
    F_roll = pid_roll_.computeCommand(0 - roll_, period);
    F_inertial = 0;
    F_gravity = 1. / 2 * body_mass_ * g_;
  }
  else
  {
    switch (jumpState_)
    {
      case JumpState::LEG_RETRACTION:
        F_leg(0) = pid_left_leg_.computeCommand(0.1 - left_pos_[0], period);
        F_leg(1) = pid_right_leg_.computeCommand(0.1 - right_pos_[0], period);
        F_roll = pid_roll_.computeCommand(0 - roll_, period);
        F_gravity = (1. / 2 * body_mass_) * g_;

        if (leg_aver < 0.11)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 8)
        {
          jumpTime_ = 0;
          jumpState_ = JumpState::JUMP_UP;
        }
        break;
      case JumpState::JUMP_UP:
        F_leg(0) = p1_ * pow(left_pos_[0], 3) + p2_ * pow(left_pos_[0], 2) + p3_ * left_pos_[0] + p4_;
        F_leg(1) = p1_ * pow(right_pos_[0], 3) + p2_ * pow(right_pos_[0], 2) + p3_ * right_pos_[0] + p4_;
        F_roll = pid_roll_.computeCommand(0 - roll_, period);
        F_gravity = 0;

        if (leg_aver > 0.32)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 2)
        {
          jumpTime_ = 0;
          jumpState_ = JumpState::OFF_GROUND;
        }
        break;
      case JumpState::OFF_GROUND:
        F_leg(0) = -(p1_ * pow(left_pos_[0], 3) + p2_ * pow(left_pos_[0], 2) + p3_ * left_pos_[0] + p4_);
        F_leg(1) = -(p1_ * pow(right_pos_[0], 3) + p2_ * pow(right_pos_[0], 2) + p3_ * right_pos_[0] + p4_);
        F_roll = 0;
        F_gravity = 0;
        if (leg_aver < 0.11)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 3)
        {
          jumpTime_ = 0;
          jumpState_ = JumpState::IDLE;
          lastJumpTime_ = ros::Time::now();
        }
        break;
    }
  }

  left_wheel_speed_ = left_u(0) - pid_yaw_spd_.getCurrentCmd();
  right_wheel_speed_ = right_u(0) + pid_yaw_spd_.getCurrentCmd();

  if ((!start_ && left_unstick_ && right_unstick_ && jumpState_ == JumpState::IDLE) ||
      jumpState_ == JumpState::OFF_GROUND)
  {
    left_wheel_speed_ = 0;
    right_wheel_speed_ = 0;
    left_u(1) = left_k_(1, 0) * (-left_x(0)) + left_k_(1, 1) * (-left_x(1));
    right_u(1) = right_k_(1, 0) * (-right_x(0)) + right_k_(1, 1) * (-right_x(1));
    F_gravity = 0;
  }

  // clang-format off
  j << 1, 1./cos(left_pos_[1]), -1,
      -1, 1./cos(right_pos_[1]), 1;
  // clang-format on
  p << F_roll, F_gravity, F_inertial;
  F_bl = j * p + F_leg;

  double left_T[2], right_T[2];
  leg_conv(F_bl[0], -left_u(1) - T_theta_diff, left_angle[0], left_angle[1], left_T);
  leg_conv(F_bl[1], -right_u(1) + T_theta_diff, right_angle[0], right_angle[1], right_T);

  unstickDetection(time, period, left_x_[0], left_x_[1], right_x_[0], right_x_[1],
                   imu_handle_.getLinearAcceleration()[2], left_spd_[0], left_pos_[0], F_bl[0],
                   -left_u(1) - T_theta_diff, right_spd_[0], right_pos_[0], F_bl[1], -right_u(1) + T_theta_diff);

  left_wheel_joint_handle_.setCommand(left_wheel_speed_);
  right_wheel_joint_handle_.setCommand(right_wheel_speed_);
  if (start_)
  {
    if (abs(left_x_[0] + right_x_[0]) / 2 < 0.2 && abs(left_x_[1] + right_x_[1]) / 2 < 0.1 &&
        abs(left_x_[4] + right_x_[4]) / 2 < 0.2 && abs(left_x_[5] + right_x_[5]) / 2 < 0.1 &&
        (ros::Time::now() - start_time_) > ros::Duration(0.1))
    {
      start_ = false;
    }
  }
  else
  {
    left_front_leg_joint_handle_.setCommand(left_T[1]);
    right_front_leg_joint_handle_.setCommand(right_T[1]);
    left_back_leg_joint_handle_.setCommand(left_T[0]);
    right_back_leg_joint_handle_.setCommand(right_T[0]);
  }

  std_msgs::Float64MultiArray input_;
  input_.data.push_back(left_u(0));
  input_.data.push_back(left_u(1));
  input_.data.push_back(right_u(0));
  input_.data.push_back(right_u(1));

  //  input_.data.push_back(left_u_(0) - pid_yaw_spd_.getCurrentCmd());
  //  input_.data.push_back(left_u_(0) + pid_yaw_spd_.getCurrentCmd());
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
  double leftWheelVelAbsolute = leftWheelVel + left_pos_[0] * left_spd_[1] * cos(left_pos_[1] + pitch_) +
                                left_spd_[0] * sin(left_pos_[1] + pitch_);
  double rightWheelVelAbsolute = rightWheelVel + right_pos_[0] * right_spd_[1] * cos(right_pos_[1] + pitch_) +
                                 right_spd_[0] * sin(right_pos_[1] + pitch_);

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
  left_x_[0] = left_pos_[1] + pitch_;
  left_x_[1] = left_spd_[1] + angular_vel_base_.y;
  left_x_[2] += x_hat(0) * period.toSec();
  left_x_[3] = x_hat(0);
  left_x_[4] = pitch_;
  left_x_[5] = angular_vel_base_.y;

  right_x_[0] = right_pos_[1] + pitch_;
  right_x_[1] = right_spd_[1] + angular_vel_base_.y;
  right_x_[2] += x_hat(0) * period.toSec();
  right_x_[3] = x_hat(0);
  right_x_[4] = pitch_;
  right_x_[5] = angular_vel_base_.y;

  std_msgs::Float64MultiArray state_;
  state_.data.push_back(left_x_[0]);
  state_.data.push_back(left_x_[1]);
  state_.data.push_back(left_x_[2]);
  state_.data.push_back(left_x_[3]);
  state_.data.push_back(left_x_[4]);
  state_.data.push_back(left_x_[5]);
  observationPublisher_.publish(state_);

  std_msgs::Float64 leg_length_;
  leg_length_.data = leg_aver;
  legLengthPublisher_.publish(leg_length_);
}

void LeggedBalanceController::unstickDetection(const ros::Time& time, const ros::Duration& period,
                                               const double& left_theta, const double& left_dtheta,
                                               const double& right_theta, const double& right_dtheta,
                                               const double& ddzm, const double& left_dL0, const double& left_L0,
                                               const double& left_F, const double& left_Tp, const double& right_dL0,
                                               const double& right_L0, const double& right_F, const double& right_Tp)
{
  static double left_FN, left_P, left_ddzw(0), left_ddL0(0), mw(0.8), lastLeftLegDL0(0), lastLeftLegDDL0(0),
      lastLeftLegDDTheta(0), left_ddtheta(0), lastLeftLegDTheta(0), lpfRatio(0.5);
  static double right_FN, right_P, right_ddzw(0), right_ddL0(0), lastRightLegDL0(0), lastRightLegDDL0(0),
      lastRightLegDDTheta(0), right_ddtheta(0), lastRightLegDTheta(0);
  static ros::Time leftJudgeTime, rightJudgeTime;
  static bool leftMaybeChange, rightMaybeChange;

  left_ddL0 = (left_dL0 - lastLeftLegDL0) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastLeftLegDDL0;
  left_ddtheta = (left_dtheta - lastLeftLegDTheta) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastLeftLegDDTheta;
  left_P = left_F * cos(left_theta) + (left_Tp * sin(left_theta) / left_L0);
  left_ddzw = ddzm + g_ - left_ddL0 * cos(left_theta) + 2 * left_dL0 * left_dtheta * sin(left_theta) +
              left_L0 * left_ddtheta * sin(left_theta) + left_L0 * (left_dtheta * left_dtheta * cos(left_theta));
  left_FN = mw * left_ddzw + mw * g_ + (left_P);
  lastLeftLegDDL0 = left_ddL0;
  lastLeftLegDL0 = left_dL0;
  lastLeftLegDTheta = left_dtheta;
  lastLeftLegDDTheta = left_ddtheta;

  right_ddL0 = (right_dL0 - lastRightLegDL0) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastRightLegDDL0;
  right_ddtheta =
      (right_dtheta - lastRightLegDTheta) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastRightLegDDTheta;
  right_P = right_F * cos(right_theta) + (right_Tp * sin(right_theta) / right_L0);
  right_ddzw = ddzm + g_ - right_ddL0 * cos(right_theta) + 2 * right_dL0 * right_dtheta * sin(right_theta) +
               right_L0 * right_ddtheta * sin(right_theta) +
               right_L0 * (right_dtheta * right_dtheta * cos(right_theta));
  right_FN = mw * right_ddzw + mw * g_ + (right_P);
  lastRightLegDDL0 = right_ddL0;
  lastRightLegDL0 = right_dL0;
  lastRightLegDTheta = right_dtheta;
  lastRightLegDDTheta = right_ddtheta;

  left_averFNPtr_->input(left_FN);
  right_averFNPtr_->input(right_FN);
  left_FN = left_averFNPtr_->output();
  right_FN = right_averFNPtr_->output();

  std_msgs::Float64MultiArray support;
  support.data.push_back(left_ddzw);
  support.data.push_back(left_P);
  support.data.push_back(left_F);
  support.data.push_back(left_Tp);
  support.data.push_back(ddzm);
  support.data.push_back(left_theta);
  support.data.push_back(left_L0);
  support.data.push_back(left_ddL0);

  support.data.push_back(left_FN);
  support.data.push_back(right_FN);

  std_msgs::Bool unstickFlag;

  bool leftUnTouch = false, rightUnTouch = false;
  leftUnTouch = left_FN < 20;
  rightUnTouch = right_FN < 20;

  if (leftUnTouch != left_unstick_)
  {
    if (!leftMaybeChange)
    {
      leftJudgeTime = ros::Time::now();
      leftMaybeChange = true;
    }
    else
    {
      if (ros::Time::now() - leftJudgeTime > ros::Duration(0.01))
      {
        left_unstick_ = leftUnTouch;
      }
    }
  }
  else
  {
    leftMaybeChange = false;
  }
  if (rightUnTouch != right_unstick_)
  {
    if (!rightMaybeChange)
    {
      rightJudgeTime = ros::Time::now();
      rightMaybeChange = true;
    }
    else
    {
      if (ros::Time::now() - rightJudgeTime > ros::Duration(0.01))
      {
        right_unstick_ = rightUnTouch;
      }
    }
  }
  else
  {
    rightMaybeChange = false;
  }

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

void LeggedBalanceController::sitDown(const ros::Time& time, const ros::Duration& period)
{
  if (overturnStateChanged_)
  {
    ROS_INFO("[balance] Enter SitDown for overturn");
    overturnStateChanged_ = false;
    lastSitDownTime_ = time;
  }

  if (cmd_struct_.cmd_chassis_.mode != 4)
  {
    if ((time - lastSitDownTime_).toSec() > 1.0 && abs(left_x_(0) + right_x_(0)) / 2 < legProtectAngle_)
    {
      balance_mode_ = BalanceMode::NORMAL;
      start_ = false;
      position_des_ = (left_x_(2) + right_x_(2)) / 2;
      ROS_INFO("[balance] Exit SitDown");
    }
  }

  // PID
  double T_theta_diff = pid_theta_diff_.computeCommand(left_pos_[1] - right_pos_[1], period);
  double F_length_diff = pid_length_diff_.computeCommand(left_pos_[0] - right_pos_[0], period);
  leg_aver = (left_pos_[0] + right_pos_[0]) / 2;

  // set target state
  if (!left_unstick_ && !right_unstick_)
  {
    yaw_des_ += vel_cmd_.z * period.toSec();
    position_des_ += vel_cmd_.x * period.toSec();
  }
  Eigen::Matrix<double, CONTROL_DIM, 1> left_u, right_u;
  auto left_x = left_x_, right_x = right_x_;
  left_x(2) -= position_des_;
  right_x(2) -= position_des_;
  if (state_ != RAW)
  {
    left_x(3) -= vel_cmd_.x;
    right_x(3) -= vel_cmd_.x;
  }
  left_k_ = getK(left_pos_[0]);
  right_k_ = getK(right_pos_[0]);
  left_u = left_k_ * (-left_x);
  right_u = right_k_ * (-right_x);
  pid_yaw_pos_.computeCommand((yaw_des_ - yaw_total_), period);
  pid_yaw_spd_.computeCommand((pid_yaw_pos_.getCurrentCmd() - angular_vel_base_.z), period);

  // Leg control
  Eigen::Matrix<double, 2, 3> j;
  Eigen::Matrix<double, 3, 1> p;
  Eigen::Matrix<double, 2, 1> F_leg, F_bl;
  double F_roll, F_gravity, F_inertial;

  // Kp too small? F_leg not enough?
  F_leg[0] = pid_left_leg_.computeCommand(0.1 - leg_aver, period) - F_length_diff;
  F_leg[1] = pid_right_leg_.computeCommand(0.1 - leg_aver, period) + F_length_diff;
  F_roll = 0;
  F_inertial = 0;
  if (!start_)
  {
    F_gravity = 1. / 2 * body_mass_ * g_;
  }
  else
  {
    F_gravity = 0;
  }

  // 倒地自起检测
  if (!start_ && (abs(left_x_[0] + right_x_[0]) / 2 > 0.4) && (abs(left_x_[4] + right_x_[4]) / 2) > 0.3)
  {
    if (leg_aver < 0.11)
    {
      start_ = true;
      start_time_ = ros::Time::now();
    }
  }

  // clang-format off
  j << 1, 1./cos(left_pos_[1]), -1,
      -1, 1./cos(right_pos_[1]), 1;
  // clang-format on
  p << F_roll, F_gravity, F_inertial;
  F_bl = j * p + F_leg;

  double left_T[2], right_T[2];
  leg_conv(F_bl[0], -T_theta_diff, left_angle[0], left_angle[1], left_T);
  leg_conv(F_bl[1], +T_theta_diff, right_angle[0], right_angle[1], right_T);

  left_wheel_joint_handle_.setCommand(left_u(0) - pid_yaw_spd_.getCurrentCmd());
  right_wheel_joint_handle_.setCommand(right_u(0) + pid_yaw_spd_.getCurrentCmd());

  if (start_)
  {
    if (abs(left_x_[0] + right_x_[0]) / 2 < 0.2 && abs(left_x_[1] + right_x_[1]) / 2 < 0.1 &&
        abs(left_x_[4] + right_x_[4]) / 2 < 0.2 && abs(left_x_[5] + right_x_[5]) / 2 < 0.1 &&
        (ros::Time::now() - start_time_) > ros::Duration(0.1))
    {
      start_ = false;
    }
  }
  else
  {
    left_front_leg_joint_handle_.setCommand(left_T[1]);
    right_front_leg_joint_handle_.setCommand(right_T[1]);
    left_back_leg_joint_handle_.setCommand(left_T[0]);
    right_back_leg_joint_handle_.setCommand(right_T[0]);
  }

  std_msgs::Float64MultiArray input_;
  input_.data.push_back(left_u(0));
  input_.data.push_back(left_u(1));
  input_.data.push_back(right_u(0));
  input_.data.push_back(right_u(1));
  inputPublisher_.publish(input_);
}

void LeggedBalanceController::starting(const ros::Time&)
{
  start_ = true;
  start_time_ = ros::Time::now();
}

geometry_msgs::Twist LeggedBalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = (left_x_[3] + right_x_[3]) / 2;
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
