#!/usr/bin/env python

import collections
collections.MutableMapping = collections.abc.MutableMapping

import io
import time
import traceback
import socket
import argparse
import ipaddress
from dronekit import connect, Vehicle, VehicleMode
import skymap_pb2 as sm
import delimited_protobuf as dp
import tatep

def handle_client_message(m: sm.ClientMessage):
    pass

def run_skymap_target(period: float, 
                      ip_skymap: str, 
                      port_skymap: int, 
                      vehicle: Vehicle):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(period)

    last_est_time = 0
    last_ping_time = 0 
    while True:
        if float(vehicle.last_heartbeat or 10) > 5:
            print("no heartbeat in 5 seconds...")
            time.sleep(2)
            continue

        # attempt to receive client message
        data, sender_addr = None, None
        try:
            data, sender_addr = sock.recvfrom(1024)
            m = tatep.parse_client_message(data)
            if m is not None:
                handle_client_message(m)
        except socket.timeout as e:
            pass

        # server message
        t = time.time()
        if t - last_ping_time > 5:
            last_ping_time = t

            m = tatep.make_ping()
            print(f"sending ping to {ip_skymap}, {port_skymap}")
        elif t - last_est_time > 0.5:
            last_est_time = t

            m = tatep.make_target_estimate(vehicle)
            print(f"sending {m.WhichOneof('message')} to {ip_skymap}, {port_skymap}")
        else:
            continue

        data = tatep.serialize_server_message(m)

        try:
            sock.sendto(data, (ip_skymap, port_skymap))
        except Exception:
            print(f"failed to send message to {ip_skymap}, {port_skymap}: {m}")



def init_vehicle(url):
    print(f"Connecting to {url}")

    vehicle = connect(url, wait_ready=True)

    print(f"Connected {vehicle.version}, SYSID: {vehicle._master.target_system}")

    # Wait for vehicle to be armable
    while not vehicle.is_armable:
        print(f"Waiting for vehicle to be armable...")
        time.sleep(1)

    return vehicle

def main(period: float, mavlink_url: str, ip_skymap: str, port_skymap: int):
    try:
        # Connect to both vehicles (ports increment by 10)
        vehicle = init_vehicle(mavlink_url)  # First vehicle

        run_skymap_target(period, ip_skymap, port_skymap, vehicle)

    except KeyboardInterrupt:
        print("\nScript interrupted by user")
    except Exception as e:
        traceback.print_exception(e)

def validate_period(period_str):
    """Validate port number (1-65535)"""
    period = float(period_str)
    if period > 0 and period <= 10:
        return period
    else:
        raise argparse.ArgumentTypeError(f"Invalid period: {period_str}. Must be in [0-1]")

def validate_ip(ip_string):
    """Validate IP address (IPv4 or IPv6)"""
    try:
        ipaddress.ip_address(ip_string)
        return ip_string
    except ValueError:
        raise argparse.ArgumentTypeError(f"Invalid IP address: {ip_string}")

def validate_port(port_string):
    """Validate port number (1-65535)"""
    port = int(port_string)
    if 1 <= port <= 65535:
        return port
    else:
        raise argparse.ArgumentTypeError(f"Invalid port: {port_string}. Must be 1-65535")

def parse_arguments():
    parser = argparse.ArgumentParser(description="Parse IP address and port")

    parser.add_argument(
        "--period", "-p",
        type=validate_period,
        required=True,
        help="IP address (IPv4 or IPv6)"
    )

    parser.add_argument(
        "--skymap-ip",
        type=validate_ip,
        required=True,
        help="IP address (IPv4 or IPv6)"
    )

    parser.add_argument(
        "--skymap-port",
        type=validate_port,
        default=8888,
        help="Port number (1-65535)"
    )

    parser.add_argument(
        "--mavlink-url",
        type=str,
        default='udp:127.0.0.1:14550',
        help="Port number (1-65535)"
    )

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    main(args.period, args.mavlink_url, args.skymap_ip, args.skymap_port)
