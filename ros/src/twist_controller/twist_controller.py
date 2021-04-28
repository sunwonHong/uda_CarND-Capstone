from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_base, wheel_radius, steer_ratio, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        kp = 1.5
        ki = 0.001
        kd = 0.
        mn = -5
        mx = 1.
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        self.vehicle_mass = vehicle_mass
        self.final_mass = (fuel_capacity * GAS_DENSITY) + vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = self.decel_limit * self.wheel_radius * self.final_mass
            
        elif throttle > 0:    
            brake = 0

        else:
            brake = -throttle * self.wheel_radius * self.final_mass
            throttle = 0
        
        return throttle, brake , steering
