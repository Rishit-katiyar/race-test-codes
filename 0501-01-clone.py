import math

REWARD_MIN_SPEED = 0.8
REWARD_MAX_SPEED = 5.0
REWARD_SPEED_THRESHOLD = 3.0
REWARD_SPEED_WEIGHT = 2.0
REWARD_STEERING_THRESHOLD = 15.0
REWARD_STEERING_WEIGHT = 1.0
REWARD_CENTERLINE_WEIGHT = 1.5
REWARD_DISTANCE_WEIGHT = 1.0
REWARD_STEPS_WEIGHT = 0.5
REWARD_FAST_LAP_TIME_WEIGHT = 2.0
REWARD_AGGRESSIVE_STEERING_WEIGHT = 1.5
WAYPOINT_REWARD_WEIGHT = 0.8
REWARD_TRACK_PROGRESS_WEIGHT = 1.2
REWARD_PROGRESS_THRESHOLD = 0.3
REWARD_PROGRESS_WEIGHT = 0.5
REWARD_LANE_REWARD_WEIGHT = 1.5

def reward_function(params):
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    speed = params['speed']
    steering_angle = abs(params['steering_angle'])
    steps = params['steps']
    closest_waypoints = params['closest_waypoints']
    progress = params['progress']
    waypoints = params['waypoints']
    track_length = len(waypoints)
    track_progress = (closest_waypoints[1] + 1) / track_length

    waypoint_reward = 0.0
    for i in range(3):
        waypoint_idx = (closest_waypoints[1] + i) % track_length
        waypoint = waypoints[waypoint_idx]
        waypoint_distance = math.hypot(waypoint[0] - params['x'], waypoint[1] - params['y'])
        waypoint_reward += math.exp(-0.5 * waypoint_distance)

    reward = 0.0

    if all_wheels_on_track and (0.5 * track_width - distance_from_center) >= 0.05:
        reward += REWARD_SPEED_WEIGHT * (speed - REWARD_MIN_SPEED) / (REWARD_MAX_SPEED - REWARD_MIN_SPEED + 1e-6)
        reward -= REWARD_STEERING_WEIGHT * (steering_angle - REWARD_STEERING_THRESHOLD) / (90 - REWARD_STEERING_THRESHOLD + 1e-6)
        reward += REWARD_CENTERLINE_WEIGHT * math.exp(-0.5 * (distance_from_center / (0.5 * track_width))**2)
        reward += REWARD_DISTANCE_WEIGHT * (1.0 / (1.0 + math.exp(-10.0 * (track_progress - 0.5))))
        reward -= REWARD_STEPS_WEIGHT * (1.0 - math.exp(-steps / 200.0))
        reward += REWARD_FAST_LAP_TIME_WEIGHT * math.exp(-2.0 * (progress - 0.5)**2)
        reward -= REWARD_AGGRESSIVE_STEERING_WEIGHT / (1.0 + math.exp(-5.0 * (steering_angle - 10.0)))
        reward += WAYPOINT_REWARD_WEIGHT * waypoint_reward
        reward += REWARD_TRACK_PROGRESS_WEIGHT * math.exp(-0.5 * (track_progress - REWARD_PROGRESS_THRESHOLD)**2)
        reward += REWARD_LANE_REWARD_WEIGHT * math.log(1.0 - distance_from_center / (0.5 * track_width) + 1.0)

    return float(reward)
