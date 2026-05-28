/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

#include "motion_planner.h"

MotionPlanner::~MotionPlanner() {}

State MotionPlanner::get_goal_state_in_ego_frame(const State& ego_state,
                                                 const State& goal_state) {
  auto goal_state_ego_frame = goal_state;

  // Translate: ego is origin
  goal_state_ego_frame.location.x -= ego_state.location.x;
  goal_state_ego_frame.location.y -= ego_state.location.y;
  goal_state_ego_frame.location.z -= ego_state.location.z;

  // Rotate: ego heading becomes 0 in new frame
  auto theta_rad = -ego_state.rotation.yaw;
  auto cos_theta = std::cos(theta_rad);
  auto sin_theta = std::sin(theta_rad);
  auto goal_x = goal_state_ego_frame.location.x;
  auto goal_y = goal_state_ego_frame.location.y;

  goal_state_ego_frame.location.x =
      cos_theta * goal_x - sin_theta * goal_y;
  goal_state_ego_frame.location.y =
      sin_theta * goal_x + cos_theta * goal_y;

  goal_state_ego_frame.rotation.yaw += theta_rad;

  goal_state_ego_frame.rotation.yaw = utils::keep_angle_range_rad(
      goal_state_ego_frame.rotation.yaw, -M_PI, M_PI);

  return goal_state_ego_frame;
}

std::vector<State> MotionPlanner::generate_offset_goals_ego_frame(
    const State& ego_state, const State& goal_state) {
  auto goal_state_ego_frame =
      get_goal_state_in_ego_frame(ego_state, goal_state);
  return generate_offset_goals(goal_state_ego_frame);
}

std::vector<State> MotionPlanner::generate_offset_goals_global_frame(
    const State& goal_state) {
  return generate_offset_goals(goal_state);
}

std::vector<State> MotionPlanner::generate_offset_goals(
    const State& goal_state) {
  std::vector<State> goals_offset;

  // Perpendicular to the goal heading: add pi/2 to get the lateral direction
  auto yaw = goal_state.rotation.yaw + M_PI / 2.0;

  for (int i = 0; i < _num_paths; ++i) {
    auto goal_offset = goal_state;
    float offset = (i - (int)(_num_paths / 2)) * _goal_offset;

    // Offset goal positions along the perpendicular direction
    goal_offset.location.x += offset * std::cos(yaw);
    goal_offset.location.y += offset * std::sin(yaw);

    if (valid_goal(goal_state, goal_offset)) {
      goals_offset.push_back(goal_offset);
    }
  }
  return goals_offset;
}

bool MotionPlanner::valid_goal(const State& main_goal,
                               const State& offset_goal) {
  auto max_offset = ((int)(_num_paths / 2) + 1) * _goal_offset;
  auto dist = utils::magnitude(main_goal.location - offset_goal.location);
  return dist < max_offset;
}

std::vector<int> MotionPlanner::get_best_spiral_idx(
    const std::vector<std::vector<PathPoint>>& spirals,
    const std::vector<State>& obstacles, const State& goal_state) {
  double best_cost = DBL_MAX;
  std::vector<int> collisions;
  int best_spiral_idx = -1;
  for (size_t i = 0; i < spirals.size(); ++i) {
    double cost = calculate_cost(spirals[i], obstacles, goal_state);

    if (cost < best_cost) {
      best_cost = cost;
      best_spiral_idx = i;
    }
    if (cost > DBL_MAX) {
      collisions.push_back(i);
    }
  }
  if (best_spiral_idx != -1) {
    collisions.push_back(best_spiral_idx);
    return collisions;
  }
  std::vector<int> noResults;
  return noResults;
}

std::vector<std::vector<PathPoint>>
MotionPlanner::transform_spirals_to_global_frame(
    const std::vector<std::vector<PathPoint>>& spirals,
    const State& ego_state) {
  std::vector<std::vector<PathPoint>> transformed_spirals;
  for (auto spiral : spirals) {
    std::vector<PathPoint> transformed_single_spiral;
    for (auto path_point : spiral) {
      PathPoint new_path_point;
      new_path_point.x = ego_state.location.x +
                         path_point.x * std::cos(ego_state.rotation.yaw) -
                         path_point.y * std::sin(ego_state.rotation.yaw);
      new_path_point.y = ego_state.location.y +
                         path_point.x * std::sin(ego_state.rotation.yaw) +
                         path_point.y * std::cos(ego_state.rotation.yaw);
      new_path_point.theta = path_point.theta + ego_state.rotation.yaw;

      transformed_single_spiral.emplace_back(new_path_point);
    }
    transformed_spirals.emplace_back(transformed_single_spiral);
  }
  return transformed_spirals;
}

std::vector<std::vector<PathPoint>> MotionPlanner::generate_spirals(
    const State& ego_state, const std::vector<State>& goals) {
  PathPoint start;
  start.x = ego_state.location.x;
  start.y = ego_state.location.y;
  start.z = ego_state.location.z;
  start.theta = ego_state.rotation.yaw;
  start.kappa = 0.0;
  start.s = 0.0;
  start.dkappa = 0.0;
  start.ddkappa = 0.0;

  std::vector<std::vector<PathPoint>> spirals;
  for (auto goal : goals) {
    PathPoint end;
    end.x = goal.location.x;
    end.y = goal.location.y;
    end.z = goal.location.z;
    end.theta = goal.rotation.yaw;
    end.kappa = 0.0;
    end.s = std::sqrt((end.x * end.x) + (end.y * end.y));
    end.dkappa = 0.0;
    end.ddkappa = 0.0;

    if (_cubic_spiral.GenerateSpiral(start, end)) {
      std::vector<PathPoint>* spiral = new std::vector<PathPoint>;
      auto ok = _cubic_spiral.GetSampledSpiral(P_NUM_POINTS_IN_SPIRAL, spiral);
      if (ok && valid_spiral(*spiral, goal)) {
        spirals.push_back(*spiral);
      }
    }
  }
  return spirals;
}

bool MotionPlanner::valid_spiral(const std::vector<PathPoint>& spiral,
                                 const State& offset_goal) {
  auto n = spiral.size();
  auto delta_x = (offset_goal.location.x - spiral[n - 1].x);
  auto delta_y = (offset_goal.location.y - spiral[n - 1].y);
  auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
  return (dist < 0.1);
}

float MotionPlanner::calculate_cost(const std::vector<PathPoint>& spiral,
                                    const std::vector<State>& obstacles,
                                    const State& goal) {
  float cost = 0.0;
  cost += cf::collision_circles_cost_spiral(spiral, obstacles);
  cost += cf::close_to_main_goal_cost_spiral(spiral, goal);
  return cost;
}
