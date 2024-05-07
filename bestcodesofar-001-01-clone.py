import math
import numpy as np

def reward_function(params):
    # Extract parameters
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    is_left_of_center = params['is_left_of_center']
    heading = params['heading']
    waypoints = np.array(params['waypoints'])
    progress = params['progress']
    speed = params['speed']
    steering = params['steering_angle']

    # Constants
    MIN_REWARD = 1e-3
    FULL_REWARD_DISTANCE = 0.4 * track_width
    CENTER_WEIGHT = 0.5
    DIRECTION_WEIGHT = 0.3
    SPEED_WEIGHT = 0.3
    PROGRESS_WEIGHT = 0.2
    CURVATURE_WEIGHT = 0.2
    WAYPOINT_WEIGHT = 0.3
    SMOOTHNESS_WEIGHT = 0.1
    SPEED_MIN = 0.9
    SPEED_MAX = 1.0
    SPEED_PENALTY = 0.7

    # Initialize reward
    reward = MIN_REWARD

    if all_wheels_on_track:
        # Calculate deviation penalty
        deviation_penalty = CENTER_WEIGHT * distance_from_center
        reward += max(1 - deviation_penalty, 0)

        # Check if within full reward distance
        if distance_from_center <= FULL_REWARD_DISTANCE:
            reward += 1.0

        # Calculate angle from center
        angle_from_center = abs(heading - 90) if is_left_of_center else abs(heading + 270)

        # Calculate direction penalty
        direction_penalty = DIRECTION_WEIGHT * angle_from_center
        reward += max(1 - direction_penalty, 0)

        # Apply speed penalty
        if speed < SPEED_MIN:
            reward -= SPEED_WEIGHT * (SPEED_MIN - speed) * SPEED_PENALTY
        elif speed > SPEED_MAX:
            reward -= SPEED_WEIGHT * (speed - SPEED_MAX) * SPEED_PENALTY

        # Apply progress reward
        reward += PROGRESS_WEIGHT * progress

        # Calculate curvature deviation
        curvature_deviation = abs(steering) / 30.0
        reward -= CURVATURE_WEIGHT * curvature_deviation

        # Calculate distance to next waypoint
        next_waypoint_index = params['closest_waypoints'][1]
        next_waypoint = waypoints[next_waypoint_index]
        distance_to_next_waypoint = np.linalg.norm(np.array([params['x'], params['y']]) - next_waypoint)
        reward += WAYPOINT_WEIGHT * max(1 - (distance_to_next_waypoint / track_width), 0)

        # Apply smoothness reward
        if abs(steering) < 5:
            reward += SMOOTHNESS_WEIGHT

    return float(reward)
