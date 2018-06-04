
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

Kp_t = 1.0
Ki_t = 0.001
Kd_t = 3.0

MIN_THROTTLE = -1.0
MAX_THROTTLE = 1.0

Kp_s = 1.0
Ki_s = 0.001
Kd_s = 3.0

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, vehicle_mass, wheel_radius):
        # TODO: Implement
        self.throttle_pid = PID(Kp_t, Ki_t, Kd_t, MIN_THROTTLE, MAX_THROTTLE)
        
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.vehicle_mass  = vehicle_mass
        self.wheel_radius  = wheel_radius
        self.min_speed = 0.0
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        
        self.steering_pid = PID(Kp_s, Ki_s, Kd_s, -self.max_steer_angle, self.max_steer_angle)
        
        self.previous_timestamp = None
        
    def control(self, target_lv, target_av, current_lv, current_av, is_dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        if self.previous_timestamp is None or not is_dbw_enabled:
            self.previous_timestamp = rospy.get_time()
            return 0.0, 0.0, 0.0
        
        dt = rospy.get_time() - self.previous_timestamp
        
        error_v = min(target_lv.x, 30.0*ONE_MPH) - current_lv.x
        error_v = max(self.decel_limit*dt, min(self.accel_limit*dt, error_v))

        throttle = self.throttle_pid.step(error_v, dt)
        throttle = max(0.0, min(1.0, throttle))

        if error_v < 0:
            brake = -15.0*error_v   # Proportional braking
            brake = max(brake, 1.0)
            throttle = 0.0
        else:
            brake = 0.0

        if abs(target_v.x) < 0.1:
            brake = 100.0
            
        steering = self.yaw_controller.get_steering(target_lv, target_av, current_lv)
        steering = self.steering_pid.step(steering, dt)

        rospy.loginfo("velocity => target: %f  | current: %f ", target_lv, current_lv)
        rospy.loginfo("steering => target: %f  | current: %f ", target_av, current_av)
        rospy.loginfo("throttle: %f  | brake: %f  | Steer: %f", throttle, brake, steering)
        
        return throttle, brake, steer
