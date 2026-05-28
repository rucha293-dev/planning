/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file behavior_planner_FSM.cpp
 **/

#include "behavior_planner_FSM.h"

State BehaviorPlannerFSM::get_closest_waypoint_goal(
    const State& ego_state, const SharedPtr<cc::Map>& map,
    const float& lookahead_distance, bool& is_goal_junction) {
  // Nearest waypoint on the center of a Driving Lane.
  auto waypoint_0 = map->GetWaypoint(ego_state.location);

  if (_active_maneuver == DECEL_TO_STOP || _active_maneuver == STOPPED) {
    State waypoint;
    auto wp_transform = waypoint_0->GetTransform();
    waypoint.location = wp_transform.location;
    waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
    waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
    waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
    return waypoint;
  }

  // Waypoints at a lookahead distance
  auto lookahead_waypoints = waypoint_0->GetNext(lookahead_distance);
  auto n_wp = lookahead_waypoints.size();
  if (n_wp == 0) {
    State waypoint;
    return waypoint;
  }

  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];

  is_goal_junction = waypoint_0->IsJunction();
  auto cur_junction_id = waypoint_0->GetJunctionId();
  if (is_goal_junction) {
    if (cur_junction_id == _prev_junction_id) {
      is_goal_junction = false;
    } else {
      _prev_junction_id = cur_junction_id;
    }
  }
  State waypoint;
  auto wp_transform = waypoint_0->GetTransform();
  waypoint.location = wp_transform.location;
  waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
  waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
  waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
  return waypoint;
}

double BehaviorPlannerFSM::get_look_ahead_distance(const State& ego_state) {
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);

  // Lookahead distance = distance to stop from current speed using comfortable
  // deceleration: d = v^2 / (2 * a_max)
  auto look_ahead_distance = (velocity_mag * velocity_mag) / (2.0 * P_DECEL_MAX);

  look_ahead_distance =
      std::min(std::max(look_ahead_distance, _lookahead_distance_min),
               _lookahead_distance_max);

  return look_ahead_distance;
}

State BehaviorPlannerFSM::get_goal(const State& ego_state,
                                   SharedPtr<cc::Map> map) {
  // Get look-ahead distance based on Ego speed
  auto look_ahead_distance = get_look_ahead_distance(ego_state);

  // Nearest waypoint on the center of a Driving Lane.
  bool is_goal_in_junction{false};
  auto goal_wp = get_closest_waypoint_goal(ego_state, map, look_ahead_distance,
                                           is_goal_in_junction);

  string tl_state = "none";
  State goal =
      state_transition(ego_state, goal_wp, is_goal_in_junction, tl_state);

  return goal;
}

State BehaviorPlannerFSM::state_transition(const State& ego_state, State goal,
                                           bool& is_goal_in_junction,
                                           string tl_state) {
  goal.acceleration.x = 0;
  goal.acceleration.y = 0;
  goal.acceleration.z = 0;

  if (_active_maneuver == FOLLOW_LANE) {
    if (is_goal_in_junction) {
      _active_maneuver = DECEL_TO_STOP;

      // Put goal behind the stopping point by _stop_line_buffer,
      // in the opposite direction of the road heading
      auto ang = goal.rotation.yaw + M_PI;
      goal.location.x += _stop_line_buffer * std::cos(ang);
      goal.location.y += _stop_line_buffer * std::sin(ang);

      // At the stopping point we want speed = 0
      goal.velocity.x = 0.0;
      goal.velocity.y = 0.0;
      goal.velocity.z = 0.0;

    } else {
      // Nominal state: drive at the speed limit in the direction of travel
      goal.velocity.x = _speed_limit * std::cos(goal.rotation.yaw);
      goal.velocity.y = _speed_limit * std::sin(goal.rotation.yaw);
      goal.velocity.z = 0;
    }

  } else if (_active_maneuver == DECEL_TO_STOP) {
    // Maintain the same goal (stop line) while decelerating
    goal = _goal;

    auto distance_to_stop_sign =
        utils::magnitude(goal.location - ego_state.location);

    // Use distance to stop sign rather than speed (handles teleportation edge case)
    if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE) {
      _active_maneuver = STOPPED;
      _start_stop_time = std::chrono::high_resolution_clock::now();
    }
  } else if (_active_maneuver == STOPPED) {
    // Keep the same stop-line goal while stopped
    goal = _goal;

    long long stopped_secs =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - _start_stop_time)
            .count();

    if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0) {
      // Done waiting; resume lane following
      _active_maneuver = FOLLOW_LANE;
    }
  }
  _goal = goal;
  return goal;
}
