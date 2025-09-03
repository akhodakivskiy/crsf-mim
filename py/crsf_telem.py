from typing import Container
import serial
import argparse
import sys
import time
import crsf_parser

def pkt_handler(frame: Container, status: crsf_parser.PacketValidationStatus):
    print(f"{frame} - {status}")

def read_serial(port, baudrate=9600, timeout=1):
    """
    Read data from a serial port and display it.

    Args:
        port: COM port name (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
        baudrate: Communication speed (default: 9600)
        timeout: Read timeout in seconds (default: 1)
    """
    try:
        # Open serial connection
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )

        print(f"Connected to {port} at {baudrate} baud")
        print("Press Ctrl+C to exit\n")

        parser = crsf_parser.CRSFParser(pkt_handler)

        # Continuously read data
        while True:
            if ser.in_waiting > 0:
                # Read available data
                data = ser.read_all()

                if data is None:
                    continue
                if len(data) > 0:
                    # If decoding fails, print raw bytes
                    input = bytearray(data)
                    parser.parse_stream(input)
                    print(f"Received (raw): {data.hex()}")

            time.sleep(0.01)  # Small delay to prevent CPU overuse

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}")
        print(f"Details: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"Closed connection to {port}")

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description='Read data from a serial COM port'
    )
    parser.add_argument(
        'port',
        help='COM port to read from (e.g., COM3, /dev/ttyUSB0)'
    )
    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        default=9600,
        help='Baud rate (default: 9600)'
    )
    parser.add_argument(
        '-t', '--timeout',
        type=float,
        default=1.0,
        help='Read timeout in seconds (default: 1.0)'
    )

    # Parse arguments
    args = parser.parse_args()

    # Start reading serial data
    read_serial(args.port, args.baudrate, args.timeout)

if __name__ == "__main__":
    main()
