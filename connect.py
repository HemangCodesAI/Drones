import time
from pymavlink import mavutil
from math import sqrt, atan2, degrees

telemetry_port = 'COM10'
baud_rate = 57600
FOLLOW_DISTANCE = 10  # Desired follow distance in meters
KP = 0.5  # Proportional gain for velocity control


def request_message(master, message_id: int):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0, # confirmation
        message_id, 0, 0, 0, 0, 0, 0)

def scan_for_drones(connection, timeout=10):
    print("Scanning for drones...")
    detected_drones = set()
    start_time = time.time()

    while time.time() - start_time < timeout:
        msg = connection.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            system_id = msg.get_srcSystem()
            if system_id not in detected_drones:
                detected_drones.add(system_id)
                print(f"Drone detected with system ID: {system_id}")

    if not detected_drones:
        print("No drones detected.")
    else:
        print(f"Total drones detected: {len(detected_drones)}")

    return detected_drones

def connect_to_drone(telemetry_port, baud_rate=57600):
    print(f"Connecting to drone on {telemetry_port} at {baud_rate} baud...")
    connection = mavutil.mavlink_connection(telemetry_port, baud=baud_rate)
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print("Heartbeat received! Connected to the drone.")
    return connection

def arm_drone(connection,sys_id):
    
    print("Arming the drone...")
    connection.target_system=sys_id
    connection.mav.command_long_send(
        connection.target_system,    
        connection.target_component, 
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  
        0,                           
        1, 0, 0, 0, 0, 0, 0          
    )
    connection.motors_armed_wait()
    print("Drone is now armed.")

def disarm_drone(connection,sys_id):
    print("Disarming the drone...")
    connection.target_system=sys_id
    connection.mav.command_long_send(
        connection.target_system,    
        connection.target_component, 
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  
        0,                           
        0, 0, 0, 0, 0, 0, 0          
    )
    connection.motors_disarmed_wait()
    print("Drone is now disarmed.")
    return None

def get_global_position(connection):
    request_message(connection,33)
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    print(msg)
    # msg = connection.messages['GLOBAL_POSITION_INT'].alt
    # print(msg)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        return lat, lon, alt
    return None


def get_battery(connection):
    request_message(connection,1)
    msg = connection.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if msg:
        voltage = msg.voltage_battery / 1000.0  # Convert millivolts to volts
        # current = msg.current_battery / 100.0   # Convert centiamps to amps
        # remaining = msg.battery_remaining       # Percentage of battery remaining
        # print(f"Battery Voltage: {voltage}V, Current: {current}A, Remaining: {remaining}%")
        return voltage
    else:
        print("Failed to receive battery data.")
        return None

def get_altitude(connection):
    request_message(connection,33)
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        altitude = msg.alt / 1000.0  # Convert from millimeters to meters
        print(f"Altitude: {altitude} meters")
        return altitude
    else:
        print("Failed to receive altitude data.")
        return None

def get_throttle(connection):
    request_message(connection,35)
    msg = connection.recv_match(type='RC_CHANNELS', blocking=True, timeout=5)
    if msg:
        throttle = msg.chan3_raw  # Assuming channel 3 is throttle (common setup)
        print(f"Throttle: {throttle}")
        return throttle
    else:
        print("Failed to receive throttle data.")
        return None

def get_rel_altitude(connection):
    request_message(connection,33)
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        rel_altitude = msg.relative_alt / 1000.0  # Convert from millimeters to meters
        print(f"Relative Altitude: {rel_altitude} meters")
        return rel_altitude
    else:
        print("Failed to receive relative altitude data.")
        return None

def get_status(connection):
    request_message(connection,0)
    msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        system_status = mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name
        print(f"System Status: {system_status}")
        return system_status
    else:
        print("Failed to receive status data.")
        return None

def get_prearm(connection):
    request_message(connection,253)
    msg = connection.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
    if msg and "PreArm" in msg.text:
        print(f"PreArm message: {msg.text}")
        return msg.text
    else:
        print("No PreArm status received.")
        return None

def get_velocity(connection):
    request_message(connection,33)
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        vx = msg.vx / 100.0  # cm/s to m/s
        vy = msg.vy / 100.0
        vz = msg.vz / 100.0
        print(f"Velocity: ({vx}, {vy}, {vz}) m/s")
        return vx, vy, vz 

def follow_drone(connection, lead_drone_sysid):
    print(f"Following drone with system ID {lead_drone_sysid}...")
    while True:
        # Get follower and lead drone positions
        follower_pos = get_global_position(connection)
        lead_pos = get_global_position(lead_drone_sysid)

        if follower_pos and lead_pos:
            # Calculate distance between the two drones
            dist_x = lead_pos[0] - follower_pos[0]
            dist_y = lead_pos[1] - follower_pos[1]
            distance = sqrt(dist_x**2 + dist_y**2)

            # Calculate heading to the lead drone
            heading = degrees(atan2(dist_y, dist_x))

            # Calculate velocity commands to follow
            velocity_x = KP * dist_x
            velocity_y = KP * dist_y

            # Send velocity commands to the follower drone
            connection.mav.set_position_target_local_ned_send(
                0,  # timestamp
                connection.target_system,  # target system
                connection.target_component,  # target component
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
                0b0000111111000111,  # type_mask: ignore position, yaw, and altitude
                0, 0, 0,  # position (not used)
                velocity_x, velocity_y, 0,  # velocity in m/s
                0, 0, 0,  # acceleration (not used)
                heading, 0)  # heading and yaw rate

            print(f"Distance to lead drone: {distance:.2f} meters, Heading: {heading:.2f}°")
            
            if distance < FOLLOW_DISTANCE:
                print("At desired follow distance.")
                break
        else:
            print("Failed to retrieve position data.")

        time.sleep(1)  # Adjust as needed


def main():
    try:
        connection = connect_to_drone(telemetry_port, baud_rate)
        while True:
            command = input("Enter a command : ").strip().lower()
            parts=command.split()
            if len(parts)==0:
                continue
            command=parts[0]
            tar_system_id=int(parts[1][2:]) if len(parts) > 1 and parts[1].startswith('--') else None
            if tar_system_id is not None:
                    connection.target_system = tar_system_id
            if command == 'get_position':
                print(get_global_position(connection))
            elif command == 'arm':
                arm_drone(connection)
            elif command == 'disarm':
                disarm_drone(connection)
            elif command == 'scan':
                drones = scan_for_drones(connection)
                print(drones)
            elif command == 'follow':
                lead_drone_id = int(input("Enter the lead drone's system ID: "))
                follow_drone(connection, lead_drone_id)
            elif command=="velocity":
                get_velocity(connection)
            elif command=="prearm":
                get_prearm(connection)
            elif command=="status":
                get_status(connection)
            elif command=="throttle":
                get_throttle(connection)
            elif command=="alt":
                get_altitude(connection)
            elif command=="rel":
                get_rel_altitude(connection)
            elif command=="battery":
                get_battery(connection)
            elif command == 'exit':
                print("Exiting...")
                break
            else:
                print("Unknown command.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        connection.close()
        print("Connection closed.")

if __name__ == "__main__":
    main()
