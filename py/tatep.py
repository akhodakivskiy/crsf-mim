from io import BytesIO
import time
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

def make_estimate_from_global_position_int(msg, name: str) -> sm.TargetEstimate:
    e = sm.TargetEstimate()

    e.timestamp.seconds = int(time.time_ns() / 1000000000)
    e.timestamp.nanos = int(time.time_ns() % 1000000000)
    e.precise_timestamp = False
    e.target_id = name
    e.position.latitude_deg = msg.lat / 1e7
    e.position.longitude_deg = msg.lon / 1e7
    e.position.altitude_msl_m = msg.alt / 1000
    e.velocity.north_ms = (msg.vx / 100) or 0
    e.velocity.east_ms = (msg.vy / 100) or 0
    e.velocity.up_ms = (-msg.vz / 100) or 0

    return e

def make_target_estimate(est: sm.TargetEstimate) -> sm.ServerMessage:
    m = sm.ServerMessage()
    m.target_estimate.CopyFrom(est)
    return m
