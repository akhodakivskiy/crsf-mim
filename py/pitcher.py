import math
import numpy as np

class Pitcher:
    """Alternative implementation with pure incremental logic and better error estimation."""

    def __init__(self):
        # Gains (tuned for incremental control)
        self.kp = 0.1  # Proportional gain on acceleration error
        self.kd = 0.005  # Derivative gain on velocity
        self.ki = 0.02  # Integral gain

        # Limits
        self.max_pitch_rate = 0.5
        self.integral_limit = 0.5

        # Filtering
        self.filter_alpha = 0.3

        # State
        self.prev_vz = 0.0
        self.a_cmd_filtered = 0.0
        self.accel_integral = 0.0

    def update(self, dt: float, vz: float, a_cmd: float, last_pitch: float) -> float:
        """
        Pure incremental control based on acceleration error.
        """
        if dt <= 0:
            return last_pitch

        # Filter command
        self.a_cmd_filtered = (1 - self.filter_alpha) * self.a_cmd_filtered + self.filter_alpha * a_cmd

        # Proportional increment
        pitch_delta_p = self.kp * self.a_cmd_filtered * dt

        # Derivative increment (based on velocity for damping)
        pitch_delta_d = -self.kd * (vz - self.prev_vz) * dt
        self.prev_vz = vz

        # Integral increment
        if abs(a_cmd) > 0.1:  # Only integrate for significant commands
            self.accel_integral += a_cmd * dt
            self.accel_integral = np.clip(
                self.accel_integral, 
                -self.integral_limit, 
                self.integral_limit)
        else:
            self.accel_integral *= 0.95

        pitch_delta_i = self.ki * self.accel_integral * dt

        # Total increment
        pitch_delta = pitch_delta_p + pitch_delta_d + pitch_delta_i

        # Rate limit the increment
        max_delta = self.max_pitch_rate * dt
        pitch_delta = np.clip(pitch_delta, -max_delta, max_delta)


        # New pitch command
        pitch_cmd = last_pitch + pitch_delta

        # Anti-windup
        if (pitch_cmd >= 1.0 or pitch_cmd <= -1.0):
            self.accel_integral *= 0.9

        return max(-1.0, min(1.0, pitch_cmd))

    def reset(self):
        """Reset the controller state."""
        self.a_cmd_filtered = 0.0
        self.accel_integral = 0.0
