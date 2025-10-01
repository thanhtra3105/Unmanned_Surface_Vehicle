import time
from pymavlink import mavutil

# MAVLink connection settings
udp_ip = "localhost"
udp_port = 12345
serial_port = '/dev/serial0'
baud_rate = 38400

# Establish UDP connection to receive MAVLink messages
udp_conn = mavutil.mavlink_connection(f'udp:{udp_ip}:{udp_port}')

# Establish serial connection to forward MAVLink messages
serial_conn = mavutil.mavlink_connection(serial_port, baud=baud_rate)

# Function to receive and forward MAVLink messages
def receive_and_forward():
    while True:
        # Receive a message from the UDP connection
        msg = udp_conn.recv_msg()
        if msg:
            # Print received message
            print(f"Received message: {msg}")

            # Forward the message via serial connection
            serial_conn.mav.send(msg)
            print(f"Forwarded message: {msg}")

if __name__ == "__main__":
    try:
        receive_and_forward()
    except KeyboardInterrupt:
        print("Terminating script.")