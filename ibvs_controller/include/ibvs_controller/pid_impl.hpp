// Copyright (c) 2026 Aitor Ibarguren
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISUAL_SERVOING_CONTROLLER__PID_2D_IMP_HPP_
#define VISUAL_SERVOING_CONTROLLER__PID_2D_IMP_HPP_

#include "Eigen/Geometry"

namespace visual_servoing_controller
{

class PID2D
{
private:
  Eigen::VectorXd kp_;
  Eigen::VectorXd ki_;
  Eigen::VectorXd kd_;

  Eigen::VectorXd integral_;
  Eigen::VectorXd prev_error_;

public:
  PID2D(const Eigen::VectorXd & p, const Eigen::VectorXd & i, const Eigen::VectorXd & d)
  : kp_(p),
    ki_(i),
    kd_(d),
    integral_(Eigen::VectorXd::Zero(2)),
    prev_error_(Eigen::VectorXd::Zero(2))
  {
  }

  Eigen::VectorXd calculate(const Eigen::VectorXd & error, double dt)
  {
    // Integral
    integral_ += error * dt;

    // Derivative
    Eigen::VectorXd derivative = (error - prev_error_) / dt;

    // PID output
    Eigen::VectorXd output =
      kp_.cwiseProduct(error) + ki_.cwiseProduct(integral_) + kd_.cwiseProduct(derivative);

    prev_error_ = error;

    return output;
  }

  void reset()
  {
    integral_.setZero();
    prev_error_.setZero();
  }
};

}  // namespace visual_servoing_controller

#endif  // VISUAL_SERVOING_CONTROLLER__PID_2D_IMP_HPP_