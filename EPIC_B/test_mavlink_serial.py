# import time
# from pymavlink import mavutil

# # MAVLink connection settings
# udp_ip = "localhost"
# udp_port = 12345
# serial_port = '/dev/serial0'
# baud_rate = 38400

# # Establish UDP connection to receive MAVLink messages
# udp_conn = mavutil.mavlink_connection(f'udp:{udp_ip}:{udp_port}')

# # Establish serial connection to forward MAVLink messages
# serial_conn = mavutil.mavlink_connection(serial_port, baud=baud_rate)

# # Function to receive and forward MAVLink messages
# def receive_and_forward():
#     while True:
#         # Receive a message from the UDP connection
#         msg = udp_conn.recv_msg()
#         if msg:
#             # Print received message
#             print(f"Received message: {msg}")

#             # Forward the message via serial connection
#             serial_conn.mav.send(msg)
#             print(f"Forwarded message: {msg}")

# if __name__ == "__main__":
#     try:
#         receive_and_forward()
#     except KeyboardInterrupt:
#         print("Terminating script.")


from pymavlink import mavutil
import time

serial_port = "/dev/ttyAMA0"
serial_baud = 57600

master = mavutil.mavlink_connection(serial_port, serial_baud)
print(f"Connected to port: {serial_port} with baud: {serial_baud}")

def send_waypoint(lat, lon, alt):
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    master.mav.mission_count_send(master.target_system, master.target_component, 1)
    time.sleep(0.5)
    master.mav.mission_item_int_send(
        master.target_system, 
        master.target_component, 
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
        0,1,
        0,0,0,0,
        int(lat*1e7), int(lon*1e7), alt*1e7
    )
    print(f"Sent waypoint:{lat}, {lon}, {alt}")
    time.sleep(1)
    msg = master.recv_match(type="MISSION_ACK", timeout=10)
    if msg:
        print("missison upload complete")


def start_mission():
    try:
        master.set_mode('AUTO')
        print("Set mode to AUTO")
        
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 
            1,0,0,0,0,0,0
        )
        print('ARMED')

        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0,0,0,0,0,0,0
        )
        print("Started mission")
    except Exception as e:
        print("Error: e")

while(1):
    hb = master.wait_heartbeat()
    try:
        if not hb:
            raise Exception("Khong nhan duoc heartbeat")
        else:
            print(f"Da nhan duoc heartbeat: {master.target_system}, commopnent: {master.target_component}")
            # msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            # print(msg)
            # if msg:
            #     if msg.get_type() == "GLOBAL_POSITION_INT":
            #         print(f"lat: {msg.lat}, lon: {msg.lon}, alt: {msg.alt}")
            #         msg_dict = msg.to_dict()
            #         print(f" {msg_dict['lat']}, {msg_dict['lon']}, {msg_dict['alt']}")
            #         # time.sleep(2)
            #         # master.mav.send(msg)
            #         # print(f"Da gui lai msg: {msg}")
            send_waypoint(16.0668138, 108.1608325, 0)
            start_mission()
    except Exception as e:
        print(f"Error: {e}")

    
