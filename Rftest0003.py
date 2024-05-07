import math

def reward_function(params):
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    is_left_of_center = params['is_left_of_center']
    heading = params['heading']
    waypoints = params['waypoints']
    progress = params['progress']
    speed = params['speed']
    steering = params['steering_angle']

    MIN_REWARD = 1e-3
    FULL_REWARD_DISTANCE = 0.4 * track_width
    CENTER_WEIGHT = 0.5
    DIRECTION_WEIGHT = 0.3
    SPEED_WEIGHT = 0.3
    PROGRESS_WEIGHT = 0.2
    CURVATURE_WEIGHT = 0.2
    WAYPOINT_WEIGHT = 0.3
    SMOOTHNESS_WEIGHT = 0.1
    
    SPEED_MIN = 0.8
    SPEED_MAX = 1.0
    
    reward = MIN_REWARD
    
    if all_wheels_on_track:
        deviation_penalty = CENTER_WEIGHT * distance_from_center
        reward += max((1 - deviation_penalty), 0)
        
        if distance_from_center <= FULL_REWARD_DISTANCE:
            reward += 1.0
        
        if is_left_of_center:
            angle_from_center = abs(heading - 90)
        else:
            angle_from_center = abs(heading + 270)
        
        direction_penalty = DIRECTION_WEIGHT * angle_from_center
        reward += max((1 - direction_penalty), 0)
        
        if speed < SPEED_MIN:
            reward -= SPEED_WEIGHT * (SPEED_MIN - speed)
        elif speed > SPEED_MAX:
            reward -= SPEED_WEIGHT * (speed - SPEED_MAX)
        
        reward += PROGRESS_WEIGHT * progress
        
        curvature_deviation = abs(steering) / 30.0
        reward -= CURVATURE_WEIGHT * curvature_deviation
        
        next_waypoint_index = params['closest_waypoints'][1]
        next_waypoint = waypoints[next_waypoint_index]
        distance_to_next_waypoint = math.sqrt((params['x'] - next_waypoint[0]) ** 2 + (params['y'] - next_waypoint[1]) ** 2)
        reward += WAYPOINT_WEIGHT * max((1 - (distance_to_next_waypoint / track_width)), 0)
        
        if abs(steering) < 5:
            reward += SMOOTHNESS_WEIGHT
    
    else:
        reward = MIN_REWARD
    
    return float(reward)
