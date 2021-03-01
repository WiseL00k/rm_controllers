//
// Created by qiayuan on 8/14/20.
//

#ifndef SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_
#define SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_

#include <rm_msgs/GimbalTrackCmd.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <rm_gimbal_controllers/GimbalConfig.h>
#include <dynamic_reconfigure/server.h>
#include <robot_state_controller/robot_state_interface.h>
#include "cpp_types.h"
#include <ros_utilities.h>

struct Config {
  double resistance_coff, g, delay, dt, timeout;
};

class BulletSolver {
 public:
  explicit BulletSolver(ros::NodeHandle &controller_nh) {

    map2gimbal_des_.header.frame_id = "map";
    map2gimbal_des_.child_frame_id = "gimbal_des";
    controller_nh.param("publish_rate", publish_rate_, 50.0);

    // init config
    config_ = {.resistance_coff = getParam(controller_nh, "resistance_coff", 0.),
        .g = getParam(controller_nh, "g", 0.),
        .delay = getParam(controller_nh, "delay", 0.),
        .dt = getParam(controller_nh, "dt", 0.),
        .timeout = getParam(controller_nh, "timeout", 0.)};
    config_rt_buffer_.initRT(config_);

    d_srv_ =
        new dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig>(controller_nh);
    dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig>::CallbackType
        cb = boost::bind(&BulletSolver::reconfigCB, this, _1, _2);
    d_srv_->setCallback(cb);

    path_pub_ = controller_nh.advertise<visualization_msgs::Marker>("bullet_model", 10);
  };
  virtual ~BulletSolver() = default;
  virtual void setTarget(const DVec<double> &pos, const DVec<double> &vel) = 0;
  virtual void setBulletSpeed(double speed) { bullet_speed_ = speed; };
  virtual void reconfigCB(rm_gimbal_controllers::GimbalConfig &config, uint32_t /*level*/) {
    ROS_INFO("[Gimbal] Dynamic params change");
    if (!dynamic_reconfig_initialized_) {
      Config init_config = *config_rt_buffer_.readFromNonRT(); // config init use yaml
      config.resistance_coff = init_config.resistance_coff;
      config.g = init_config.g;
      config.delay = init_config.delay;
      config.dt = init_config.dt;
      config.timeout = init_config.timeout;
      dynamic_reconfig_initialized_ = true;
    }
    Config config_non_rt{
        .resistance_coff=config.resistance_coff,
        .g = config.g,
        .delay = config.delay,
        .dt=config.dt,
        .timeout  =config.timeout
    };
    config_rt_buffer_.writeFromNonRT(config_non_rt);
  };
  virtual bool solve(const DVec<double> &angle_init, geometry_msgs::TransformStamped map2pitch,
                     realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) = 0;
  virtual void modelRviz(double x_offset, double y_offset, double z_offset) = 0;

 protected:
  double bullet_speed_{};
  ros::Publisher path_pub_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalConfig> *d_srv_{};
  geometry_msgs::TransformStamped map2gimbal_des_;
  double publish_rate_{};
  ros::Time last_publish_time_;
  bool dynamic_reconfig_initialized_ = false;
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  Config config_{};
};

class Bullet2DSolver : public BulletSolver {
 public:
  using BulletSolver::BulletSolver;
  void setTarget(const DVec<double> &pos, const DVec<double> &vel) override {
    target_x_ = pos[0] + vel[0] * config_.delay;
    target_z_ = pos[1] + vel[1] * config_.delay;
    target_dx_ = vel[0];
    target_dz_ = vel[1];
  };
  bool solve(const DVec<double> &angle_init, geometry_msgs::TransformStamped map2pitch,
             realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) override;
 protected:
  virtual double computeError(double pitch) = 0;
  double target_x_{}, target_z_{}, target_dx_{}, target_dz_{};
  double fly_time_{};
  double pitch_solved_;
};

class Iter2DSolver : public Bullet2DSolver {
 public:
  using Bullet2DSolver::Bullet2DSolver;
  using Bullet2DSolver::solve;
 private:
  double computeError(double pitch) override;
};

class Approx2DSolver : public Bullet2DSolver {
 public:
  using Bullet2DSolver::Bullet2DSolver;
  using Bullet2DSolver::solve;
 private:
  double computeError(double pitch) override;
};

class Bullet3DSolver : public BulletSolver {
 public:
  using BulletSolver::BulletSolver;
  void setTarget(const DVec<double> &pos, const DVec<double> &vel) override {
    target_x_ = pos[0] + vel[0] * config_.delay;
    target_y_ = pos[1] + vel[1] * config_.delay;
    target_z_ = pos[2] + vel[2] * config_.delay;
    target_dx_ = vel[0];
    target_dy_ = vel[1];
    target_dz_ = vel[2];
  };
  bool solve(const DVec<double> &angle_init, geometry_msgs::TransformStamped map2pitch,
             realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) override;
  void modelRviz(double x_offset, double y_offset, double z_offset) override;
  geometry_msgs::TransformStamped getResult(const ros::Time &time);
  std::vector<Vec3<double>> getPointData3D();
 protected:
  virtual double computeError(double yaw, double pitch, double *error_polar) = 0;
  double target_x_{}, target_y_{}, target_z_{},
      target_dx_{}, target_dy_{}, target_dz_{};
  double fly_time_{};
  double pitch_solved_, yaw_solved_;
  Vec3<double> pos_{};
  Vec3<double> vel_{};
  std::vector<Vec3<double>> model_data_;
  double map2pitch_offset_x_{};
  double map2pitch_offset_y_{};
  double map2pitch_offset_z_{};
};

class Iter3DSolver : public Bullet3DSolver {
 public:
  using Bullet3DSolver::Bullet3DSolver;
  using Bullet3DSolver::solve;
  using Bullet3DSolver::getPointData3D;
  using Bullet3DSolver::getResult;
 private:
  double computeError(double yaw, double pitch, double *error_polar) override;
};

class Approx3DSolver : public Bullet3DSolver {
 public:
  using Bullet3DSolver::Bullet3DSolver;
  using Bullet3DSolver::solve;
  using Bullet3DSolver::getPointData3D;
  using Bullet3DSolver::getResult;
 private:
  double computeError(double yaw, double pitch, double *error_polar) override;
};
#endif //SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_
