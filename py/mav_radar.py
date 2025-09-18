#!/usr/bin/env python

import collections

from numpy import isin

collections.MutableMapping = collections.abc.MutableMapping

import io
import time
import traceback
import socket
import argparse
import ipaddress
from dataclasses import dataclass
from dronekit import connect, Vehicle, VehicleMode
import skymap_pb2 as sm
import delimited_protobuf as dp

class MavRadar:
    def __init__(self, ip: str, port_skymap: int, v_ceptor: Vehicle, v_target: Vehicle):
        self.ip = ip
        self.port_skymap = port_skymap

        self.vehicle_ceptor = v_ceptor
        self.vehicle_target = v_target

        self.tatep_header = bytes.fromhex("54545001")

    def serialize_server_message(self, m):
        stream = io.BytesIO()
        stream.write(self.tatep_header)

        dp.write(stream, m)

        return stream.getbuffer()


    def parse_client_message(self, data):
        stream = io.BytesIO(data)

        if stream.read(4) != self.tatep_header:
            print('unexpected header')
            return None

        return dp.read(stream, sm.ClientMessage)

    def make_estimate(self, v: Vehicle, name: str):
        e = sm.TargetEstimate()
        e.timestamp.seconds = int(time.time_ns() / 1000000000)
        e.timestamp.nanos = int(time.time_ns() % 1000000000)
        e.precise_timestamp = False
        e.target_id = name
        e.position.latitude_deg = v.location._lat
        e.position.longitude_deg = v.location._lon
        e.position.altitude_msl_m = v.location._alt
        e.velocity.north_ms = v._vx or 0
        e.velocity.east_ms = v._vy or 0
        e.velocity.up_ms = -(v._vz or 0)

        return e

    def handle_client_message(self, m):
        field_name = m.WhichOneof('message')

        if field_name == 'channels':
            if self.vehicle_ceptor.mode == "FBWA":
                self.vehicle_ceptor.channels.overrides['1'] = m.channels.ch1
                self.vehicle_ceptor.channels.overrides['2'] = m.channels.ch2
                self.vehicle_ceptor.channels.overrides['3'] = m.channels.ch3
                self.vehicle_ceptor.channels.overrides['4'] = m.channels.ch4

            print(f"channels {m.channels.ch1} {m.channels.ch2} {m.channels.ch3} {m.channels.ch4}")
        elif field_name == 'status':
            print(f"Skymap status")
        else:
            print(f"unhandled Skymap message {m}")
            return None



    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.1)

        last_est_time = 0
        last_ping_time = 0 
        vehicle = None
        while True:
            # attempt to receive client message
            data, sender_addr = None, None
            try:
                data, sender_addr = sock.recvfrom(1024)
                m = self.parse_client_message(data)
                if m is not None:
                    self.handle_client_message(m)
            except socket.timeout as e:
                pass

            # send server message

            m = sm.ServerMessage()

            t = time.time()
            if t - last_ping_time > 5:
                last_ping_time = t

                m.ping.CopyFrom(sm.Ping())
                #print(f"sending ping to {self.ip}, {self.port_skymap}")
            elif t - last_est_time > 0.5:
                last_est_time = t

                vehicle_name = None
                if vehicle == self.vehicle_ceptor:
                    vehicle = self.vehicle_target

                    est = self.make_estimate(vehicle, "target")
                    m.target_estimate.CopyFrom(est)
                else:
                    vehicle = self.vehicle_ceptor

                    est = self.make_estimate(vehicle, "interceptor")
                    m.interceptor_estimate.CopyFrom(est)
                #print(f"sending {m.WhichOneof('message')} to {self.ip}, {self.port_skymap}")
            else:
                continue

            data = self.serialize_server_message(m)

            try:
                sock.sendto(data, (self.ip, self.port_skymap))
            except Exception:
                print(f"failed to send message to {self.ip}, {self.port_skymap}: {m}")



def init_vehicle(port, vehicle_name):
    """Init individual vehicle"""
    connection_string = f'udp:127.0.0.1:{port}'

    try:
        vehicle = connect(connection_string, wait_ready=True)

        print(f"Connected to {vehicle_name}: {vehicle.version}, SYSID: {vehicle._master.target_system}")

        # Wait for vehicle to be armable
        while not vehicle.is_armable:
            print(f"Waiting for {vehicle_name} to be armable...")
            time.sleep(1)

        if not vehicle.armed:
            print(f"settings mode=TAKEOFF for {vehicle_name}")
            vehicle.mode = VehicleMode("TAKEOFF")

            while vehicle.mode != "TAKEOFF":
                print(f"mode={vehicle.mode}")
                time.sleep(1)

            vehicle.armed = True

            while not vehicle.armed:
                print(f"Waiting for {vehicle_name} arming...")
                time.sleep(1)

        return vehicle
    except Exception as e:
        print(f"Error initializing {vehicle_name}: {e}")
        return None

def main(ip, port):

    vehicles = []

    try:
        # Connect to both vehicles (ports increment by 10)
        vehicle_ceptor = init_vehicle(14560, "interceptor")  # First vehicle
        vehicle_target = init_vehicle(14570, "target")  # Second vehicle

        if vehicle_ceptor is None or vehicle_target is None:
            print("Failed to setup one or both vehicles")
            return

        vehicles.append(vehicle_ceptor)
        vehicles.append(vehicle_target)

        radar = MavRadar(ip, port, vehicle_ceptor, vehicle_target)

        radar.run()

    except KeyboardInterrupt:
        print("\nScript interrupted by user")
    except Exception as e:
        traceback.print_exception(e)
    finally:
        # Clean up
        for vehicle in vehicles:
            try:
                if vehicle:
                    vehicle.close()
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
