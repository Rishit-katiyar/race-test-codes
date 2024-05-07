def reward_function(params):
    # Extract input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    steering = abs(params['steering_angle'])
    track_length = params['track_length']
    width_margin = 0.05 * track_width
    max_speed = 4.0  # Define maximum allowed speed

    # Initialize reward
    reward = 1e-3

    # Reward for staying on the track
    if all_wheels_on_track:
        # Calculate distance from the center line
        distance_from_center_ratio = distance_from_center / (track_width / 2.0)

        # Reward for staying close to the center line
        if distance_from_center_ratio <= 0.5:
            reward += 0.5  # Base reward for staying near the center
        else:
            reward += 0.1  # Smaller reward for being off-center but still on track

        # Progress-based reward
        reward += progress * 0.5

        # Speed-based reward
        reward += min(speed / max_speed, 1.0) * 0.3

        # Additional reward for maintaining high speed without going off track
        if speed >= max_speed:
            reward += 0.5

        # Penalize excessive steering
        reward -= steering * 0.1

        # Penalize slow speeds when steering is high
        if steering > 0.8 and speed < 2.0:
            reward -= 0.3

    # Penalize if the car goes off track
    else:
        reward -= 1.0

    # Additional reward for completing the track
    if progress == 100:
        reward += 10.0

    # Penalize if too many steps are taken
    if steps > track_length / 2:
        reward -= 2.0

    # Always return a float value
    return float(reward)
