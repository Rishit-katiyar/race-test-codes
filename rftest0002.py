import math

def reward_function(params):
    # Input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    is_left_of_center = params['is_left_of_center']
    heading = params['heading']
    waypoints = params['waypoints']
    progress = params['progress']
    speed = params['speed']
    steering = params['steering_angle']

    # Constants
    MIN_REWARD = 1e-3
    FULL_REWARD_DISTANCE = 0.4 * track_width
    CENTER_WEIGHT = 0.5
    DIRECTION_WEIGHT = 0.3
    SPEED_MIN = 0.9
    SPEED_MAX = 1.2
    SPEED_PENALTY = 0.7
    PROGRESS_WEIGHT = 0.2
    CURVATURE_WEIGHT = 0.2
    WAYPOINT_WEIGHT = 0.3
    SMOOTHNESS_WEIGHT = 0.1
    MAX_ANGLE = 360
    MAX_STEERING_ANGLE = 30.0
    CLOSENESS_THRESHOLD = 0.05
    STEERING_THRESHOLD = 5
    
    reward = MIN_REWARD
    
    if all_wheels_on_track:
        # Reward for staying close to the center line
        deviation_penalty = CENTER_WEIGHT * distance_from_center
        reward += max(1 - deviation_penalty, 0)
        reward += 1.0 if distance_from_center <= FULL_REWARD_DISTANCE else 0
        
        # Reward for steering in the correct direction
        angle_from_center = abs(heading - MAX_ANGLE / 4) if is_left_of_center else abs(heading + MAX_ANGLE * 3 / 4)
        direction_penalty = DIRECTION_WEIGHT * angle_from_center
        reward += max(1 - direction_penalty, 0)
        
        # Penalize for speed deviation from target range
        if speed < SPEED_MIN:
            reward -= SPEED_PENALTY * (SPEED_MIN - speed)
        elif speed > SPEED_MAX:
            reward -= SPEED_PENALTY * (speed - SPEED_MAX)
        
        # Reward for making progress along the track
        reward += PROGRESS_WEIGHT * progress
        
        # Penalize for high steering angle
        curvature_deviation = abs(steering) / MAX_STEERING_ANGLE
        reward -= CURVATURE_WEIGHT * curvature_deviation
        
        # Reward for following waypoints
        next_waypoint_index = params['closest_waypoints'][1]
        next_waypoint = waypoints[next_waypoint_index]
        distance_to_next_waypoint = math.sqrt((params['x'] - next_waypoint[0]) ** 2 + (params['y'] - next_waypoint[1]) ** 2)
        reward += WAYPOINT_WEIGHT * max(1 - (distance_to_next_waypoint / track_width), 0)
        
        # Additional reward for smooth steering
        reward += SMOOTHNESS_WEIGHT if abs(steering) < STEERING_THRESHOLD else 0
    
    return float(reward)
