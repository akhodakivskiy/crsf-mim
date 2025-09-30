from io import BytesIO
import time
from dronekit import Vehicle
import skymap_pb2 as sm
import delimited_protobuf as dp

header = bytes.fromhex("54545001")

def serialize_server_message(m):
    stream = BytesIO()
    stream.write(header)

    dp.write(stream, m)

    return stream.getbuffer()

def parse_client_message(data):
    stream = BytesIO(data)

    if stream.read(4) != header:
        print('unexpected header')
        return None

    return dp.read(stream, sm.ClientMessage)

def make_ping() -> sm.ServerMessage:
    m = sm.ServerMessage()
    m.ping.CopyFrom(sm.Ping())
    return m

def make_estimate(v: Vehicle, name: str) -> sm.TargetEstimate:
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

def make_target_estimate(v: Vehicle) -> sm.ServerMessage:
    m = sm.ServerMessage()
    est = make_estimate(v, "target")
    m.target_estimate.CopyFrom(est)
    return m

def make_interceptor_estimate(v: Vehicle) -> sm.ServerMessage:
    m = sm.ServerMessage()
    est = make_estimate(v, "interceptor")
    m.interceptor_estimate.CopyFrom(est)
    return m
