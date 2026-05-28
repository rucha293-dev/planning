/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file cost_functions.cpp
 **/

#include "cost_functions.h"

using namespace std;

namespace cost_functions {
// COST FUNCTIONS

double diff_cost(vector<double> coeff, double duration,
                 std::array<double, 3> goals, std::array<float, 3> sigma,
                 double cost_weight) {
  /*
  Penalizes trajectories whose coordinate(and derivatives)
  differ from the goal.
  */
  double cost = 0.0;
  vector<double> evals = evaluate_f_and_N_derivatives(coeff, duration, 2);

  for (size_t i = 0; i < evals.size(); i++) {
    double diff = fabs(evals[i] - goals[i]);
    cost += logistic(diff / sigma[i]);
  }
  return cost_weight * cost;
}

double collision_circles_cost_spiral(const std::vector<PathPoint>& spiral,
                                     const std::vector<State>& obstacles) {
  bool collision{false};
  auto n_circles = CIRCLE_OFFSETS.size();

  for (auto wp : spiral) {
    if (collision) {
      break;
    }
    double cur_x = wp.x;
    double cur_y = wp.y;
    double cur_yaw = wp.theta;  // Already in radians

    for (size_t c = 0; c < n_circles && !collision; ++c) {
      // Place collision circles along the ego vehicle's body at offsets
      // in the direction of the vehicle's heading (cur_yaw)
      auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);
      auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);

      for (auto obst : obstacles) {
        if (collision) {
          break;
        }
        auto actor_yaw = obst.rotation.yaw;
        for (size_t c2 = 0; c2 < n_circles && !collision; ++c2) {
          auto actor_center_x =
              obst.location.x + CIRCLE_OFFSETS[c2] * std::cos(actor_yaw);
          auto actor_center_y =
              obst.location.y + CIRCLE_OFFSETS[c2] * std::sin(actor_yaw);

          // Euclidean distance between ego circle and obstacle circle
          double delta_x = circle_center_x - actor_center_x;
          double delta_y = circle_center_y - actor_center_y;
          double dist = std::sqrt(delta_x * delta_x + delta_y * delta_y);

          collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
        }
      }
    }
  }
  return (collision) ? COLLISION : 0.0;
}

double close_to_main_goal_cost_spiral(const std::vector<PathPoint>& spiral,
                                      State main_goal) {
  // The last point on the spiral should be used to check how close we are to
  // the Main (center) goal.
  auto n = spiral.size();

  auto delta_x = main_goal.location.x - spiral[n - 1].x;
  auto delta_y = main_goal.location.y - spiral[n - 1].y;
  auto delta_z = main_goal.location.z - spiral[n - 1].z;

  auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y) +
                        (delta_z * delta_z));

  auto cost = logistic(dist);
  return cost;
}
}  // namespace cost_functions
