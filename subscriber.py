import struct
import zmq
import json
import time
from cflib.crtp import init_drivers
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander

# Initialize Crazyflie drivers
init_drivers()

# Crazyflie URI
URI = 'radio://0/80/2M/E7E7E7E7E7'

# Create a ZeroMQ context and subscriber socket
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Subscribe to all messages

def receive_pose():
    """Receive position data from the publisher and yield it."""
    while True:
        try:
            # Receive raw binary data (12 bytes, representing 3 float values)
            pose_data = socket.recv()

            # Unpack the 12-byte binary data into 3 float values (x, y, z)
            x, y, z = struct.unpack('fff', pose_data)

            # Yield the position data to be used by the Crazyflie
            yield x, y, z
        except zmq.ZMQError as e:
            print(f"Error receiving data: {e}")
            break

def main():
    counter = 0
    x0, y0, z0 = None, None, None  # Initial coordinates (starting point)

    # Connect to the Crazyflie
    with SyncCrazyflie(URI) as scf:
        phlc = PositionHlCommander(scf)
        
        # Take off and hover in place
        print("Taking off...")
        phlc.take_off()
        time.sleep(1)  # Give the drone time to take off
        print("Hovering...")

        try:
            # Receive position data and set the position of the Crazyflie
            while True:
                for x, y, z in receive_pose():
                    print(f"Received position: x={x}, y={y}, z={z}")

                    # If the initial coordinates are not set, use the first received coordinates as reference
                    if counter == 0:
                        x0, y0, z0 = x, y, z
                        print(f"Initial coordinates set: x0={x0}, y0={y0}, z0={z0}")
                    else:
                        # Subtract the initial coordinates from the current pose to get relative coordinates
                        dx, dy, dz = x - x0, y - y0, z - z0

                        # Print the relative position change
                        print(f"Relative position: dx={dx}, dy={dy}, dz={dz}")

                        # If the difference is small enough, stop the drone and land
                        if abs(dx) < 0.05 and abs(dy) < 0.05 and abs(dz) < 0.05:
                            print("Target position reached (difference is small).")
                            print("Drone is hovering. Waiting for manual interruption...")
                            # Keep hovering until manually stopped
                            while True:
                                time.sleep(1)  # Drone stays in hover mode, no landing yet.
                            break  # This break will never be reached, as the loop runs indefinitely

                        # Move the Crazyflie to the new relative position
                        phlc.go_to(x0 + dx, y0 + dy, z0 + dz, 0.55)

                    counter += 1

                time.sleep(0.1)  # Adjust this for the frequency of receiving the pose data

        except KeyboardInterrupt:
            print("Manual interruption detected. Landing...")
            phlc.land()

if __name__ == "__main__":
    main()
