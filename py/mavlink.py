#!/usr/bin/env python

import collections
collections.MutableMapping = collections.abc.MutableMapping
from collections import deque

import time
import argparse
import math
import numpy as np
import traceback
from dronekit import connect, Vehicle, VehicleMode
from pronav import ProNavType, ProNav, State, Command
from pitcher import Pitcher

def init_vehicle(port, vehicle_name):
    """Init individual vehicle"""
    connection_string = f'udp:127.0.0.1:{port}'

    try:
        print(f"Connecting to {vehicle_name} on port {port}...")
        vehicle = connect(connection_string, wait_ready=True)

        print(f"Connected to {vehicle_name}: {vehicle.version}, SYSID: {vehicle._master.target_system}")

        # Wait for vehicle to be armable
        print(f"Waiting for {vehicle_name} to be armable...")
        while not vehicle.is_armable:
            time.sleep(1)

        print(f"{vehicle_name} is armable")

        return vehicle
    except Exception as e:
        print(f"Error initializing {vehicle_name}: {e}")
        return None

def setup_vehicle(vehicle, vehicle_id, vehicle_name):
    """Setup individual vehicle"""

    try:
        # Set mode to GUIDED
        if not vehicle.armed:

            print(f"settings mode=GUIDED for {vehicle_name}")
            vehicle.mode = VehicleMode("GUIDED")

            while vehicle.mode != "GUIDED":
                print(f"mode={vehicle.mode}")
                time.sleep(1)

            # Arm vehicle
            print(f"Arming {vehicle_name}...")
            vehicle.armed = True

            while not vehicle.armed:
                print(f"Waiting for {vehicle_name} arming...")
                time.sleep(1)

            print(f"{vehicle_name} armed!")

            return vehicle, vehicle_name

    except Exception as e:
        print(f"Error setting up {vehicle_name}: {e}")
        return None, vehicle_name

def run_pronav(ic: Vehicle, tg: Vehicle, delay: float):
    """Monitor all vehicles"""
    pronav = ProNav(nav_constant=4, 
                    max_roll_deg=35, 
                    max_pitch_deg=25,
                    cone_bias=0.01,
                    cone_angle_deg=20)
    pitcher = Pitcher()

    queue = deque()
    time_last = time.time()
    time_last_log = time_last
    pitch_cmd = 0

    while True:
        time.sleep(0.3)

        time_now = time.time()
        state_ic_new = State(
            timestamp = time_now,
            lat = float(ic.location._lat),
            lon = float(ic.location._lon),
            alt = float(ic.location._alt),
            vel_north=float(ic._vx),
            vel_east=float(ic._vy),
            vel_up=-float(ic._vz))

        state_tg_new = State(
            timestamp = time_now,
            lat = float(tg.location._lat),
            lon = float(tg.location._lon),
            alt = float(tg.location._alt),
            vel_north=float(tg._vx),
            vel_east=float(tg._vy),
            vel_up=-float(tg._vz))

        queue.append((state_ic_new, state_tg_new))

        if (time_now - time_last) >= delay:
            (state_ic, state_tg) = queue.popleft()
            dt = state_ic.timestamp - time_last
            time_last = state_ic.timestamp

            command = pronav.compute_guidance(state_ic, state_tg)
            pitch_cmd = pitcher.update(dt, state_ic.vel_up, 
                                       command.accel_vert, pitch_cmd)
            roll_cmd = np.arctan2(command.accel_lat, pronav.g) / pronav.max_roll_rad

            roll = np.clip(int(roll_cmd * 500 + 1500), 900, 2100)
            pitch = np.clip(int((-pitch_cmd) * 500 + 1500), 900, 2100)

            if time_now - time_last_log > 1:
                print(f"{command.nav_type}, range_nav={command.range:.2f}, tgo={command.time_to_go:.2f}, zem={command.zero_effort_miss:.2f}, vz={state_ic.vel_up:.4f}, ah={command.accel_lat:.4f}, av={command.accel_vert:.4f} pitch={pitch_cmd:.4f}, pitch_pwm={pitch}")
                time_last_log = time_now

                print(f"pitch a={pitcher.a_cmd_filtered:.4f}, err_int={pitcher.accel_integral:.4f}")

            if ic.mode == "FBWA" or ic.mode == "FBWB" or ic.mode == "CRUISE":
                ic.channels.overrides['1'] = roll 
                ic.channels.overrides['2'] = pitch 
                ic.channels.overrides['3'] = 2000
            else:
                pitcher.reset()


def main(delay: float):

    vehicles_info = []

    try:
        # Connect to both vehicles (ports increment by 10)
        vehicle1 = init_vehicle(14560, "interceptor")  # First vehicle
        vehicle2 = init_vehicle(14570, "target")  # Second vehicle

        if vehicle1 is None or vehicle2 is None:
            print("Failed to setup one or both vehicles")
            return

        print("setting up vehicles")

        setup_vehicle(vehicle1, 0, "interceptor")
        setup_vehicle(vehicle2, 0, "target")

        print("Both vehicles are now executing their missions!")

        # Monitor both vehicles
        run_pronav(vehicle1, vehicle2, delay)

    except KeyboardInterrupt:
        print("\nScript interrupted by user")
    except Exception as e:
        traceback.print_exception(e)
        print(f"Error: {e}")
    finally:
        # Clean up
        for vehicle, name in vehicles_info:
            try:
                if vehicle:
                    vehicle.close()
                    print(f"Disconnected from {name}")
            except:
                pass

        print("SITL terminated")

def parse_arguments():
    parser = argparse.ArgumentParser(description="Parse IP address and port")

    parser.add_argument(
        "--delay", "-d",
        type=float,
        required=True,
        help="Delay (seconds) of the position stream"
    )

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    main(args.delay)
