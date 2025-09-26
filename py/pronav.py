import numpy as np
import math
from enum import Enum
from dataclasses import dataclass

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
    vel_up: float

@dataclass
class Command:
    nav_type: ProNavType
    accel_lon: float = 0
    accel_lat: float = 0
    accel_vert: float = 0
    range: float = 0
    time_to_go: float = 0
    zero_effort_miss: float = 0

@dataclass
class NavCtx:
    range: np.ndarray
    range_unit: np.ndarray
    range_norm: float

    vel_i: np.ndarray
    vel_i_unit: np.ndarray
    vel_i_norm: float

    vel_t: np.ndarray
    vel_t_unit: np.ndarray
    vel_t_norm: float

    vel_rel: np.ndarray
    vel_rel_unit: np.ndarray
    vel_rel_norm: float

    vel_closing: float

class ProNav:
    def __init__(self, 
                 nav_constant=3.0, 
                 max_pitch_deg=25, 
                 max_roll_deg=35, 
                 cone_bias=0.1, 
                 cone_angle_deg=20):
        """
        Proportional Navigation Guidance System

        Args:
            nav_constant: Navigation constant (typically 3-5)
            earth_radius: Earth radius in meters
        """
        self.N = nav_constant
        self.cone_bias = cone_bias
        self.cone_angle_rad = np.radians(cone_angle_deg)
        self.R_earth = 6371000.0
        self.g = 9.81
        self.deg_to_rad = math.pi / 180.0
        self.max_pitch_rad = np.radians(max_pitch_deg)
        self.max_roll_rad = np.radians(max_roll_deg)
        self.accel_lat_max = self.g * np.tan(self.max_roll_rad)

    def state_to_ned(self, state_ref, state) -> np.ndarray:
        """Convert lat/lon/alt to North-East-Down coordinates relative to reference point"""
        # Approximate conversion for small distances
        dlat = state.lat - state_ref.lat
        dlon = state.lon - state_ref.lon
        dalt = state.alt - state_ref.alt

        lat_avg = (state.lat + state_ref.lat) / 2
        radius = self.R_earth + (state.alt + state_ref.alt) / 2

        # Convert to meters
        north = dlat * self.deg_to_rad * radius
        east = dlon * self.deg_to_rad * radius * math.cos(lat_avg * self.deg_to_rad)
        down = -dalt  # NED convention: positive down

        return np.array([north, east, down])

    def state_to_ctx(self, state_ic: State, state_tg: State) -> NavCtx:
        range = self.state_to_ned(state_ic, state_tg)

        vel_i = np.array([state_ic.vel_north, 
                          state_ic.vel_east, 
                          -state_ic.vel_up])
        vel_t = np.array([state_tg.vel_north, 
                          state_tg.vel_east, 
                          -state_tg.vel_up])

        vel_rel = vel_t - vel_i

        return NavCtx(
            range=range,
            range_unit = range / np.linalg.norm(range),
            range_norm = float(np.linalg.norm(range)),

            vel_i=vel_i,
            vel_i_unit = vel_i / np.linalg.norm(vel_i),
            vel_i_norm = float(np.linalg.norm(vel_i)),

            vel_t=vel_t,
            vel_t_unit = vel_t / np.linalg.norm(vel_t),
            vel_t_norm = float(np.linalg.norm(vel_t)),

            vel_rel=vel_rel,
            vel_rel_unit = vel_rel / np.linalg.norm(vel_rel),
            vel_rel_norm = float(np.linalg.norm(vel_rel)),

            vel_closing = -np.dot(vel_rel, range) / np.linalg.norm(range)
        )

    def compute_tail_bias(self, ctx: NavCtx) -> np.ndarray:
        tail_angle = np.arccos(np.clip(np.dot(ctx.range_unit, ctx.vel_t_unit), -1, 1))

        in_cone = tail_angle <= self.cone_angle_rad

        if not in_cone:
            rotation_axis = np.cross(ctx.range_unit, ctx.vel_t_unit)
            rotation_axis_norm = np.linalg.norm(rotation_axis)

            if rotation_axis_norm > 0.001:
                rotation_axis /= np.linalg.norm(rotation_axis)

                tail_angle_delta = tail_angle - self.cone_angle_rad

                #Rodrigues rotation formula
                k = rotation_axis
                theta = tail_angle_delta
                v = ctx.range_unit
                v_rot = (v * np.cos(theta) +
                    np.cross(k, v) * np.sin(theta) + 
                    k * np.dot(k, v) * (1 - np.cos(theta)))

                los_error = v_rot - v

                a_tail_bias = - (self.cone_bias * ctx.vel_i_norm * los_error)

                return a_tail_bias

        return np.zeros(3)

    # True proportional navigation
    def compute_guidance_pronav(self, ctx: NavCtx) -> np.ndarray:
        omega_los = np.cross(ctx.range_unit, ctx.vel_rel) / (ctx.range_norm)

        return self.N * ctx.vel_rel_norm * np.cross(omega_los, ctx.vel_i_unit)

    def compute_guidance_pursuit(self, ctx: NavCtx) -> np.ndarray:
        # Component of LOS perpendicular to current velocity
        r_lat = ctx.range_unit - np.dot(ctx.range_unit, ctx.vel_i_unit) * ctx.vel_i_unit
        r_lat_norm = np.linalg.norm(r_lat)

        if r_lat_norm < 1e-6:
            return np.zeros(3)

        # Acceleration direction (perpendicular to velocity, toward target)
        r_lat_unit = r_lat / r_lat_norm

        # Angular rate needed to align with target
        angular_error = np.arcsin(np.clip(r_lat_norm, -1, 1))

        # Required lateral acceleration (v²/r = v*ω)
        accel_norm = self.N * ctx.vel_i_norm * angular_error / self.g

        return r_lat_unit * accel_norm

    def compute_guidance_accel(self, ctx: NavCtx):
        if ctx.vel_closing < 0 and ctx.range_norm > 100:
            t = ProNavType.PURSUIT
            a = self.compute_guidance_pursuit(ctx)
        else:
            t = ProNavType.TPN
            a = self.compute_guidance_pronav(ctx)

        a_tail_bias = self.compute_tail_bias(ctx)

        return t, a + a_tail_bias

    def compute_tgo_zem(self, ctx: NavCtx):
        if ctx.range_norm < 1 or ctx.vel_rel_norm < 1e-6 or ctx.vel_closing < 0.0:
            t_go = 0.0
        elif ctx.vel_closing < 1e-6:
            t_go = np.dot(ctx.range, ctx.vel_rel) / math.pow(ctx.vel_rel_norm, 2)
        else:
            t_go = ctx.range_norm / ctx.vel_closing

        if t_go > 0.0:
            zem_vec = ctx.range + ctx.vel_rel * t_go
            zem_mag = float(np.linalg.norm(zem_vec))
            return t_go, zem_mag
        else:
            return 0.0, 0.0

    def compute_guidance(self, state_ic: State, state_tg: State) -> Command:
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
        ctx: NavCtx = self.state_to_ctx(state_ic, state_tg)

        if ctx.range_norm < 1:
            print("NAV: low range")

        if ctx.vel_i_norm < 1:
            print("NAV: low speed")

        nav_type, accel = self.compute_guidance_accel(ctx)

        unit_down = np.array([0, 0, 1])
        unit_lon = ctx.vel_i_unit
        unit_lat = np.cross(unit_down, unit_lon)
        unit_lat /= np.linalg.norm(unit_lat)

        #unit_ver = np.cross(ctx.vel_i, unit_lat)
        #unit_ver_norm = np.linalg.norm(unit_ver)
        #unit_ver /= unit_ver_norm

        a_lon = np.dot(accel, unit_lon)
        a_lat = np.clip(np.dot(accel, unit_lat), 
                        -self.accel_lat_max, self.accel_lat_max)
        a_ver = -np.dot(accel, unit_down)

        tgo, zem = self.compute_tgo_zem(ctx)

        return Command(
            nav_type = nav_type,
            accel_lon = a_lon,
            accel_lat = a_lat,
            accel_vert = a_ver,
            range = ctx.range_norm,
            time_to_go = tgo,
            zero_effort_miss = zem,
        )


if __name__ == "__main__":
    import unittest
    import test_pronav

    suite = unittest.TestLoader().loadTestsFromModule(test_pronav)
    unittest.TextTestRunner(verbosity=2).run(suite)


