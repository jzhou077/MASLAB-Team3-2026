import utils.constants as constants

class MotionProfile:
    def run(self, distance, elapsed_time, max_velocity=34.2, max_acceleration=68.4):
        print("Distance: " + str(distance))
        """
        Return the current reference position based on the given motion profile times, max acc, vel, and current time.
        """
        acceleration_dt = max_velocity / max_acceleration

        halfway_distance = distance/2
        acceleration_distance = 0.5 * max_acceleration * (acceleration_dt ** 2)

        if acceleration_distance > halfway_distance:
            acceleration_dt = (halfway_distance / (0.5 * max_acceleration))**0.5
        
        acceleration_distance = 0.5 * max_acceleration * (acceleration_dt ** 2)

        max_velocity = max_acceleration * acceleration_dt

        deceleration_dt = acceleration_dt

        cruise_distance = distance - 2 * acceleration_distance
        cruise_dt = cruise_distance / max_velocity if cruise_distance > 0 else 0
        deceleration_time = acceleration_dt + cruise_dt

        entire_dt = acceleration_dt + cruise_dt + deceleration_dt
        if (elapsed_time >= entire_dt):
            return distance
        
        if elapsed_time < acceleration_dt:
            return 0.5 * max_acceleration * (elapsed_time ** 2)
        elif elapsed_time < (deceleration_time):
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt ** 2)
            cruise_current_dt = elapsed_time - acceleration_dt
            return acceleration_distance + max_velocity * cruise_current_dt
        else:
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt ** 2)
            cruise_distance = max_velocity * cruise_dt
            deceleration_time = elapsed_time - deceleration_time
            
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * (deceleration_time ** 2)