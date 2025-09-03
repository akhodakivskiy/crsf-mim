import numpy as np
import math
from enum import Enum
from dataclasses import dataclass

from tracker import AccelTracker


class ProNavType(Enum):
    NONE = "NONE"
    PURSUIT = "PURSUIT"
    TPN = "TPN"

@dataclass
class State:
    timestamp: float
    lat: float
    lon: float
    alt: float
    vel_north: float
    vel_east: float
    vel_down: float

@dataclass
class Command:
    nav_type: ProNavType
    pitch_cmd: float
    roll_cmd: float
    accel_lat: float
    accel_vert: float
    range: float

class ProportionalNavigation:
    def __init__(self, nav_constant=3.0, max_pitch_deg=25, max_roll_deg=65):
        """
        Proportional Navigation Guidance System

        Args:
            nav_constant: Navigation constant (typically 3-5)
            earth_radius: Earth radius in meters
        """
        self.N = nav_constant
        self.R_earth = 6371000.0
        self.g = 9.81
        self.deg_to_rad = math.pi / 180.0
        self.max_pitch_rad = np.radians(max_pitch_deg)
        self.max_roll_rad = np.radians(max_roll_deg)
        self.tracker = AccelTracker()

    def lat_lon_to_ned(self, state_ref, state):
        """Convert lat/lon/alt to North-East-Down coordinates relative to reference point"""
        # Approximate conversion for small distances
        dlat = state.lat - state_ref.lat
        dlon = state.lon - state_ref.lon
        dalt = state.alt - state_ref.alt

        # Convert to meters
        north = dlat * self.deg_to_rad * self.R_earth
        east = dlon * self.deg_to_rad * self.R_earth * math.cos(state_ref.lat * self.deg_to_rad)
        down = -dalt  # NED convention: positive down

        return np.array([north, east, down])

    # True proportional navigation
    def compute_guidance_tpn(self, r, vel_i, vel_t):
        vel_i_norm = np.linalg.norm(vel_i)
        if vel_i_norm < 1e-6:
            return np.zeros(3)
        vel_i_unit = vel_i / vel_i_norm

        r_norm = np.linalg.norm(r)
        if r_norm < 1:
            return np.zeros(3)
        r_unit = r / r_norm

        v = vel_t - vel_i
        v_norm = np.linalg.norm(v)
        if v_norm < 1e-6:
            return np.zeros(3)

        omega = np.cross(r, v) / (r_norm**2)

        return (- self.N * v_norm * np.cross(vel_i_unit, omega))

    def compute_guidance_pursuit(self, r, vel_i):
        # LOS and velocity vectors
        r_norm = np.linalg.norm(r)

        if r_norm < 1:
            return np.zeros(3)

        vel_i_norm = np.linalg.norm(vel_i)
        if vel_i_norm < 1e-6:
            return np.zeros(3)

        # Project LOS onto velocity plane to get desired heading
        r_unit = r / r_norm
        vel_i_unit = vel_i / vel_i_norm

        # Component of LOS perpendicular to current velocity
        r_perp = r_unit - np.dot(r_unit, vel_i_unit) * vel_i_unit
        r_perp_norm = np.linalg.norm(r_perp)

        if r_perp_norm < 1e-6:
            return np.zeros(3)

        # Acceleration direction (perpendicular to velocity, toward target)
        accel_direction = r_perp / r_perp_norm

        # Angular rate needed to align with target
        angular_error = np.arcsin(np.clip(r_perp_norm, -1, 1))

        # Required lateral acceleration (v²/r = v*ω)
        accel_norm = self.N * vel_i_norm * angular_error

        return accel_direction * accel_norm

    def compute_guidance_accel(self, r, vel_i, vel_t):
        r_norm = np.linalg.norm(r)
        v = vel_t - vel_i
        closing_speed = -np.dot(v, r / r_norm)

        print(f"{r}, {v}, {closing_speed}")

        if closing_speed < 0 and r_norm > 100:
            return ProNavType.PURSUIT, self.compute_guidance_pursuit(r, vel_i)
        else:
            return ProNavType.TPN, self.compute_guidance_tpn(r, vel_i, vel_t)

    def compute_guidance(self, ic_state: State, tg_state: State) -> Command:
        """
        Compute proportional navigation guidance commands

        Args:
            interceptor_state: dict with 'lat', 'lon', 'alt', 'v_north', 'v_east', 'v_down'
            target_state: dict with 'lat', 'lon', 'alt', 'v_north', 'v_east', 'v_down'
            interceptor_speed: constant speed of interceptor (m/s)

        Returns:
            dict with 'pitch_cmd' and 'roll_cmd' in range [-1, 1]
        """
        # Convert to NED coordinates (using interceptor as reference)
        range = self.lat_lon_to_ned(ic_state, tg_state)


        # Velocity vectors in NED
        vel_i = np.array([ic_state.vel_north, 
                          ic_state.vel_east, 
                          ic_state.vel_down])
        vel_t = np.array([tg_state.vel_north, 
                          tg_state.vel_east, 
                          tg_state.vel_down])

        nav_type, accel = self.compute_guidance_accel(range, vel_i, vel_t)

        if nav_type == ProNavType.TPN:
            metric = self.tracker.update(vel_i, accel, ic_state.timestamp)
            if metric is not None:
                print(f"accel error: {metric.error_instant}")

        vel_horizontal = np.array([vel_i[0], vel_i[1], 0])
        down_unit = np.array([0, 0, 1])

        lateral_unit = np.cross(down_unit, vel_horizontal)
        lateral_unit_norm = np.linalg.norm(lateral_unit)
        if lateral_unit_norm < 1e-6:
            return Command(ProNavType.NONE, 0, 0, 0, 0, 0)
        lateral_unit /= lateral_unit_norm

        a_lateral = np.dot(accel, lateral_unit)
        a_vertical = np.dot(accel, down_unit)

        roll_rad = np.arctan2(a_lateral, self.g)
        pitch_rad = -np.arcsin(np.clip(a_vertical / self.g, -1.0, 1.0))

        roll_cmd = np.clip(roll_rad / self.max_roll_rad, -1.0, 1.0)
        pitch_cmd = np.clip(pitch_rad / self.max_pitch_rad, -1.0, 1.0)

        print(f"accel: {accel}, lat={a_lateral}, ver={a_vertical}")
        print(f"roll_rad={roll_rad}, pitch_rad={pitch_rad}")
        print(f"roll_cmd={roll_cmd}, pitch_cmd={pitch_cmd}")

        return Command(
            nav_type = nav_type,
            pitch_cmd = pitch_cmd,
            roll_cmd = roll_cmd,
            accel_lat = a_lateral,
            accel_vert = a_vertical,
            range = float(np.linalg.norm(range)),
        )

# Example usage
def test_command(i_vel_north):
    """Example of how to use the proportional navigation system"""
    pn = ProportionalNavigation(nav_constant=3.0, max_roll_deg=35, max_pitch_deg=25)

    # Example interceptor state
    interceptor = State(
        timestamp= 0,
        lat= 30.0,
        lon= 50.0,
        alt= 1000.0,
        vel_north= i_vel_north,
        vel_east= 0.0,
        vel_down= 0.0
    )

    # Example target state
    target = State(
        timestamp= 0,
        lat= 30.1,
        lon= 50.1,
        alt= 1500.0,
        vel_north= 10.0,
        vel_east= 0.0,
        vel_down= 0.0
    )

    print(pn.lat_lon_to_ned(interceptor, target))

    command = pn.compute_guidance(interceptor, target)

    print(f"type: {command.nav_type}")

    return command

def test_guidance():
    pn = ProportionalNavigation(nav_constant=3.0)

    print("pronav: " + str(pn.compute_guidance_tpn(np.array([100, 0, 0]), np.array([10, 0, 0]), np.array([5, 0, 0]))))
    print("pronav: " + str(pn.compute_guidance_tpn(np.array([100, 10, 0]), np.array([10, 0, 0]), np.array([5, 0, 0]))))

    print("pursuit: " + str(pn.compute_guidance_pursuit(np.array([100, 0, 0]), np.array([10, 0, 0]))))
    print("pursuit: " + str(pn.compute_guidance_pursuit(np.array([100, 10, 0]), np.array([10, 0, 0]))))
    print("pursuit: " + str(pn.compute_guidance_pursuit(np.array([100, -10, 0]), np.array([10, 0, 0]))))


if __name__ == "__main__":
    test_command(100.0)
    test_command(5.0)
    #test_guidance()


