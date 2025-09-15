#!/usr/bin/env python

import collections

collections.MutableMapping = collections.abc.MutableMapping

import time
import traceback
import skymap_pb2
import socket
import argparse
import ipaddress
from dronekit import connect, Vehicle, VehicleMode
from google.protobuf.internal.encoder import _VarintBytes

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

def make_estimate(v: Vehicle, name: str):
    e = skymap_pb2.TargetEstimate()
    e.timestamp.seconds = int(time.time_ns() / 1000000000)
    e.timestamp.nanos = int(time.time_ns() % 1000000000)
    e.precise_timestamp = False
    e.target_id = name
    e.position.latitude_deg = v.location._lat
    e.position.longitude_deg = v.location._lon
    e.position.altitude_msl_m = v.location._alt
    e.velocity.east_ms = v._vx
    e.velocity.north_ms = v._vy
    e.velocity.up_ms = v._vz

    return e

def serialize_server_message(m):
    data = m.SerializeToString()
    prefix = _VarintBytes(len(data))

    return prefix + data

def run_radar(ip, port, ic: Vehicle, ic_name: str, tg: Vehicle, tg_name: str):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    header = bytes.fromhex("54545001")

    last_mav_name = ic_name
    last_mav = None
    last_log_time = time.time()
    while True:
        if last_mav_name == ic_name:
            last_mav = tg
            last_mav_name = tg_name
        else:
            last_mav = ic
            last_mav_name = ic_name

        est = make_estimate(last_mav, last_mav_name)

        if est is not None:
            m = skymap_pb2.ServerMessage()

            if last_mav_name == ic_name:
                m.interceptor_estimate.CopyFrom(est)
            else:
                m.target_estimate.CopyFrom(est)

            print(f"sending {last_mav_name}: {m}")

            data = serialize_server_message(m)

            sock.sendto(header + data, (ip, port))


        time.sleep(1)


def main(ip, port):

    vehicles_info = []

    try:
        # Connect to both vehicles (ports increment by 10)
        
        ic_name = "interceptor"
        ic_v = init_vehicle(14560, ic_name)  # First vehicle

        tg_name = "target"
        tg_v = init_vehicle(14570, tg_name)  # Second vehicle

        if ic_v is None or tg_v is None:
            print("Failed to setup one or both vehicles")
            return

        run_radar(ip, port, ic_v, ic_name, tg_v, tg_name)

    except KeyboardInterrupt:
        print("\nScript interrupted by user")
    except Exception as e:
        traceback.print_exception(e)
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

def validate_ip(ip_string):
    """Validate IP address (IPv4 or IPv6)"""
    try:
        ipaddress.ip_address(ip_string)
        return ip_string
    except ValueError:
        raise argparse.ArgumentTypeError(f"Invalid IP address: {ip_string}")

def validate_port(port_string):
    """Validate port number (1-65535)"""
    try:
        port = int(port_string)
        if 1 <= port <= 65535:
            return port
        else:
            raise ValueError
    except ValueError:
        raise argparse.ArgumentTypeError(f"Invalid port: {port_string}. Must be 1-65535")

def parse_arguments():
    parser = argparse.ArgumentParser(description="Parse IP address and port")

    parser.add_argument(
        "--ip", "-i",
        type=validate_ip,
        required=True,
        help="IP address (IPv4 or IPv6)"
    )

    parser.add_argument(
        "--port", "-p",
        type=validate_port,
        required=True,
        help="Port number (1-65535)"
    )

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    main(args.ip, args.port)
