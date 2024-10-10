import time
from pymavlink import mavutil

telemetry_port = 'COM10'
baud_rate = 57600  
def connect_to_drone():
    print(f"Connecting to drone on {telemetry_port} at {baud_rate} baud...")
    connection = mavutil.mavlink_connection(telemetry_port, baud=baud_rate)
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print("Heartbeat received! Connected to the drone.")
    return connection

def get_global_location(connection):
    print("Waiting for global position data...")
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        latitude = msg.lat / 1e7    # Convert from degrees * 1E7 to degrees
        longitude = msg.lon / 1e7   # Convert from degrees * 1E7 to degrees
        altitude = msg.alt / 1000.0 # Convert from millimeters to meters
        relative_altitude = msg.relative_alt / 1000.0 # Convert from millimeters to meters
        heading = msg.hdg / 100.0   # Convert from centi-degrees to degrees
        print(f"Latitude: {latitude}°")
        print(f"Longitude: {longitude}°")
        print(f"Altitude: {altitude} meters (MSL)")
        print(f"Relative Altitude: {relative_altitude} meters")
        print(f"Heading: {heading}°")
    else:
        print("Failed to receive GPS data. Timeout.")
def arm_drone(connection):
    print("Arming the drone...")
    connection.mav.command_long_send(
        connection.target_system,    # Target system
        connection.target_component, # Target component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command
        0,                           # Confirmation
        1, 0, 0, 0, 0, 0, 0          # Parameters: 1 to arm the drone
    )
    connection.motors_armed_wait()
    print("Drone is now armed.")

def disarm_drone(connection):
    print("Disarming the drone...")
    connection.mav.command_long_send(
        connection.target_system,    # Target system
        connection.target_component, # Target component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command
        0,                           # Confirmation
        0, 0, 0, 0, 0, 0, 0          # Parameters: 0 to disarm the drone
    )
    connection.motors_disarmed_wait()
    print("Drone is now disarmed.")

def main():
    connection = connect_to_drone()
    try:
        while True:
            command = input("Enter a command (get_position, arm, disarm, exit): ").strip().lower()
            if command == 'get_position':
                get_global_location(connection)
            elif command == 'arm':
                arm_drone(connection)
            elif command == 'disarm':
                disarm_drone(connection)
            elif command == 'exit':
                print("Exiting...")
                break
            else:
                print("Unknown command. Available commands: get_position, arm, disarm, exit")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        connection.close()
        print("Connection closed.")
if __name__ == "__main__":
    main()
