//
// Created by wiselook on 7/4/25.
//

#include "rm_chassis_controllers/vmc/leg_conv.h"
#include "rm_chassis_controllers/vmc/leg_pos.h"
#include "rm_chassis_controllers/vmc/leg_spd.h"

class VMC
{
public:
  VMC(const double& vmc_bias_angle) : vmc_bias_angle_(vmc_bias_angle)
  {
    left_averFNPtr_ = std::make_shared<MovingAverageFilter<double>>(5);
    right_averFNPtr_ = std::make_shared<MovingAverageFilter<double>>(5);
  };

  void updateVMC()
  {
    static double last_left_spd[2]{}, last_right_spd[2]{};
    leg_pos(left_angle_[0], left_angle_[1], left_pos_);
    leg_pos(right_angle_[0], right_angle_[1], right_pos_);
    leg_spd(left_dangle_[0], left_dangle_[1], left_angle_[0], left_angle_[1], left_spd_);
    leg_spd(right_dangle_[0], right_dangle_[1], right_angle_[0], right_angle_[1], right_spd_);
    for (int i = 0; i < 2; ++i)
    {
      left_spd_[i] = lpfRatio * last_left_spd[i] + (1 - lpfRatio) * left_spd_[i];
      right_spd_[i] = lpfRatio * last_right_spd[i] + (1 - lpfRatio) * right_spd_[i];
      last_left_spd[i] = left_spd_[i];
      last_right_spd[i] = right_spd_[i];
    }
    theta_[0] = lpfRatio * theta_[0] + (1 - lpfRatio) * (left_pos_[1] + pitch_);
    theta_[1] = lpfRatio * theta_[1] + (1 - lpfRatio) * (right_pos_[1] + pitch_);
    dtheta_[0] = lpfRatio * dtheta_[0] + (1 - lpfRatio) * (left_spd_[1] + dpitch_);
    dtheta_[1] = lpfRatio * dtheta_[1] + (1 - lpfRatio) * (right_spd_[1] + dpitch_);
  }

  void updateTor()
  {
    leg_conv(F_[0], Tp_[0], left_angle_[0], left_angle_[1], left_T_);
    leg_conv(F_[1], Tp_[1], right_angle_[0], right_angle_[1], right_T_);
  }

  void CalcLegFN(const ros::Duration& period)
  {
    static double left_FN, left_P, left_ddzw(0), left_ddL0(0), mw(0.8), lastLeftLegDL0(0), lastLeftLegDDL0(0),
        lastLeftLegDDTheta(0), left_ddtheta(0), lastLeftLegDTheta(0);
    static double right_FN, right_P, right_ddzw(0), right_ddL0(0), lastRightLegDL0(0), lastRightLegDDL0(0),
        lastRightLegDDTheta(0), right_ddtheta(0), lastRightLegDTheta(0);

    left_ddL0 = (left_spd_[0] - lastLeftLegDL0) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastLeftLegDDL0;
    left_ddtheta = (dtheta_[0] - lastLeftLegDTheta) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastLeftLegDDTheta;
    left_P = F_[0] * cos(theta_[0]) + (Tp_[0] * sin(theta_[0]) / left_pos_[0]);
    left_ddzw = ddot_z_M_ + g_ - left_ddL0 * cos(theta_[0]) + 2 * left_spd_[0] * dtheta_[0] * sin(theta_[0]) +
                left_pos_[0] * left_ddtheta * sin(theta_[0]) +
                left_pos_[0] * (dtheta_[0] * dtheta_[0] * cos(theta_[0]));
    left_FN = mw * left_ddzw + mw * g_ + (left_P);
    lastLeftLegDDL0 = left_ddL0;
    lastLeftLegDL0 = left_spd_[0];
    lastLeftLegDTheta = dtheta_[0];
    lastLeftLegDDTheta = left_ddtheta;

    right_ddL0 = (right_spd_[0] - lastRightLegDL0) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastRightLegDDL0;
    right_ddtheta =
        (dtheta_[1] - lastRightLegDTheta) / period.toSec() * lpfRatio + (1 - lpfRatio) * lastRightLegDDTheta;
    right_P = F_[1] * cos(theta_[1]) + (Tp_[1] * sin(theta_[1]) / right_pos_[0]);
    right_ddzw = ddot_z_M_ + g_ - right_ddL0 * cos(theta_[1]) + 2 * right_spd_[0] * dtheta_[1] * sin(theta_[1]) +
                 right_pos_[0] * right_ddtheta * sin(theta_[1]) +
                 right_pos_[0] * (dtheta_[1] * dtheta_[1] * cos(theta_[1]));
    right_FN = mw * right_ddzw + mw * g_ + (right_P);
    lastRightLegDDL0 = right_ddL0;
    lastRightLegDL0 = right_spd_[0];
    lastRightLegDTheta = dtheta_[1];
    lastRightLegDDTheta = right_ddtheta;

    left_averFNPtr_->input(left_FN);
    right_averFNPtr_->input(right_FN);
    F_N_[0] = left_FN = left_averFNPtr_->output();
    F_N_[1] = right_FN = right_averFNPtr_->output();
  }

  void setBodyState(const double& ddot_z_M, const double& pitch, const double& dpitch)
  {
    ddot_z_M_ = ddot_z_M_ * lpfRatio + (1 - lpfRatio) * ddot_z_M;
    pitch_ = pitch;
    dpitch_ = lpfRatio * dpitch_ + (1 - lpfRatio) * dpitch;
  }

  void setLegState(const double left_angle[2], const double right_angle[2], const double left_dangle[2],
                   const double right_dangle[2])
  {
    for (int i = 0; i < 2; ++i)
    {
      left_angle_[i] = left_angle[i];
      right_angle_[i] = right_angle[i];
      left_dangle_[i] = left_dangle[i];
      right_dangle_[i] = right_dangle[i];
    }
  }

  void setTor(const Eigen::Matrix<double, 2, 1>& F, const Eigen::Matrix<double, 2, 1>& Tp)
  {
    for (int i = 0; i < 2; ++i)
    {
      F_[i] = F(i);
      Tp_[i] = Tp(i);
    }
  }

  inline double getLeftFN() const
  {
    return F_N_[0];
  }
  inline double getRightFN() const
  {
    return F_N_[1];
  }

  void getForceNormal(double F_N[2]) const
  {
    for (int i = 0; i < 2; ++i)
    {
      F_N[i] = F_N_[i];
    }
  };

  double left_pos_[2]{}, left_spd_[2]{}, right_pos_[2]{}, right_spd_[2]{}, left_T_[2]{}, right_T_[2]{}, F_N_[2]{},
      theta_[2]{}, dtheta_[2]{};

  double ddot_z_M_{}, pitch_{}, dpitch_{};

private:
  double vmc_bias_angle_{}, left_angle_[2]{}, right_angle_[2]{}, left_dangle_[2]{}, right_dangle_[2]{}, F_[2]{},
      Tp_[2]{};
  double g_{ 9.81 }, lpfRatio{ 0.7 };
  std::shared_ptr<MovingAverageFilter<double>> left_averFNPtr_, right_averFNPtr_;
};