from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, decel_limit):
        self.yaw_controller = None
        self.throttle_controller = None
        self.velocity_filter = LowPassFilter(.5, .02)

        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit

        self.last_time = rospy.get_time()
    
    def init_yaw(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(
            wheel_base,
            steer_ratio,
            min_speed, 
            max_lat_accel, 
            max_steer_angle
        )

    def init_throttle(self, kp, ki, kd, min_throttle, max_throttle):
        self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity):
        if self.yaw_controller is None or self.throttle_controller is None:
            return 0., 0., 0.

        current_linear_velocity = self.velocity_filter.filt(current_linear_velocity)

        steering = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

        delta_velocity = target_linear_velocity - current_linear_velocity
        
        current_time = rospy.get_time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(delta_velocity, delta_time)

        brake = 0.

        if target_linear_velocity == 0. and current_linear_velocity < 0.1:
            # Full stop
            throttle = 0.
            brake = 400
        elif throttle < .1 and delta_velocity < 0:
            # Slow deceleration
            throttle = 0.
            deceleration = max(delta_velocity, self.decel_limit)
            brake = abs(deceleration) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering

    def reset(self):
        self.throttle_controller.reset()