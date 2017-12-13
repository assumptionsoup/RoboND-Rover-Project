import numpy as np
import math


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    elements = min(30, len(Rover.forward_obstacle_data.dists) - 1)
    emergency_stop = False
    if elements > 0 and Rover.rocks_angles is None:
        lowest_dist_idx = np.argpartition(Rover.forward_obstacle_data.dists, elements)[:elements]
        obstacle_dist = np.mean(Rover.forward_obstacle_data.dists[lowest_dist_idx])
        if obstacle_dist < 10:
            # Emergency stop, something is very close to the front of the rover
            Rover.mode = 'stop'
            emergency_stop = True

    need_pickup = Rover.near_sample and not Rover.picking_up
    if need_pickup:
        Rover.mode = 'stop'
        emergency_stop = False

    if Rover.low_speed > 30:
        Rover.mode = 'backup'
        Rover.low_speed = 0  # Give some time to try to accel



    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set

                    # Rover is going slowly, and we're not trying to
                    # pick up rocks.
                    if (Rover.vel < 0.01 and
                            Rover.rocks_angles is None and
                            not (Rover.near_sample or Rover.picking_up)):

                        Rover.low_speed += 1
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                if Rover.rocks_angles is not None:
                    angles = np.mean(Rover.rocks_angles * 180 / np.pi)
                    Rover.throttle *= 0.25  # slower accel towards rocks
                else:
                    angles = np.mean(Rover.terrain_data.angles * 180 / np.pi)

                    min_obst_dists = 0
                    if len(Rover.obstacle_data.dists):
                        min_obst_dists = np.min(Rover.obstacle_data.dists)
                    if min_obst_dists > 14 and len(Rover.obstacle_data.dists):
                        # Lean to the right!
                        angles -= 11

                Rover.steer = np.clip(angles, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            Rover.low_speed = 0
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2 and not need_pickup:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward or emergency_stop:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                    # if Rover.rocks_angles is not None:
                    #     Rover.steer = np.mean(Rover.rocks_angles * 180 / np.pi)
                    #     Rover.throttle = 0.05
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
        elif Rover.mode == 'backup':
            # Rover is stuck, try to back up, and hope it can
            # make a new route.
            Rover.throttle = -0.2
            Rover.steer = 0
            if Rover.prev_pos is not None and Rover.pos is not None:
                move = np.array(Rover.pos) - np.array(Rover.prev_pos)
                Rover.backup_dist += np.linalg.norm(move)
            if Rover.backup_dist > 0.4:
                Rover.backup_dist = 0.0
                Rover.mode = 'stop'



    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover

