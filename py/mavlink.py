#!/usr/bin/env python

import collections
collections.MutableMapping = collections.abc.MutableMapping

import time
from dronekit import connect, Vehicle, VehicleMode
from pronav import ProNavType, ProportionalNavigation, State, Command

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

def run_pronav(ic: Vehicle, tg: Vehicle):
    """Monitor all vehicles"""
    pronav = ProportionalNavigation(nav_constant=3, 
                                    max_roll_deg=35, 
                                    max_pitch_deg=25)

    last_log_time = time.time()
    while True:
        try:
            state_ic = State(
                timestamp = time.time(),
                lat = float(ic.location._lat),
                lon = float(ic.location._lon),
                alt = float(ic.location._alt),
                vel_north=float(ic._vx),
                vel_east=float(ic._vy),
                vel_down=float(ic._vz))

            state_tg = State(
                timestamp = time.time(),
                lat = float(tg.location._lat),
                lon = float(tg.location._lon),
                alt = float(tg.location._alt),
                vel_north=float(tg._vx),
                vel_east=float(tg._vy),
                vel_down=float(tg._vz))

            command = pronav.compute_guidance(state_ic, state_tg)

            roll = int(command.roll_cmd * 500 + 1500)
            pitch = int(command.pitch_cmd * 500 + 1500)

            if time.time() - last_log_time > 1:
                print(f"{command.nav_type}, range={command.range:.2f} a_lat={command.accel_lat:.4f}, roll={command.roll_cmd:.4f}/{roll}, accel_ver={command.accel_vert:.4f} pitch={command.pitch_cmd:.4f}/{pitch}")
                last_log_time = time.time()

            if ic.mode == "FBWA" or ic.mode == "FBWB" or ic.mode == "CRUISE":
                ic.channels.overrides['1'] = roll 
                ic.channels.overrides['2'] = pitch 
                ic.channels.overrides['3'] = 2000

            time.sleep(1)
        except KeyboardInterrupt:
            break


def main():

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
        run_pronav(vehicle1, vehicle2)

    except KeyboardInterrupt:
        print("\nScript interrupted by user")
    except Exception as e:
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

if __name__ == "__main__":
    main()
