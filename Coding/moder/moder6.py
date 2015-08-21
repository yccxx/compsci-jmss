from __future__ import print_function

from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
from px4flow import PX4Flow
import time, math, re, datetime

import socket
import sys
import thread
import smbus

api = local_connect()
vehicle = api.get_vehicles()[0]

class Lidar_Lite():
    def __init__(self):
        self.address = 0x62
        self.distWriteReg = 0x00
        self.distWriteVal = 0x04
        self.distReadReg1 = 0x8f
        self.distReadReg2 = 0x10
        self.velWriteReg = 0x04
        self.velWriteVal = 0x08
        self.velReadReg = 0x09

    def connect(self, bus):
        try:
              self.bus = smbus.SMBus(bus)
              time.sleep(0.5)
              return 0
        except Exception:
              return -1

    def writeAndWait(self, register, value):
        self.bus.write_byte_data(self.address, register, value);
        time.sleep(0.02)

    def readAndWait(self, register):
        res = self.bus.read_byte_data(self.address, register)
        time.sleep(0.02)
        return res

    def getDistance(self):
        self.writeAndWait(self.distWriteReg, self.distWriteVal)
        dist1 = self.readAndWait(self.distReadReg1)
        dist2 = self.readAndWait(self.distReadReg2)
        return (dist1 << 8) + dist2

    def getVelocity(self):
        self.writeAndWait(self.distWriteReg, self.distWriteVal)
        self.writeAndWait(self.velWriteReg, self.velWriteVal)
        vel = self.readAndWait(self.velReadReg)
        return self.signedInt(vel)

    def signedInt(self, value):
        if value > 127:
            return (256-value) * (-1)
        else:
            return value

def arm_and_takeoff(aTargetAltitude):
    # Arm vehicle and fly to aTargetAltitude.
    print("Basic pre-arm checks")
    # Don't let the user try to fly while autopilot is booting
    if vehicle.mode.name == "INITIALISING":
        print("Waiting for vehicle to initialise")
        time.sleep(1)
    while vehicle.gps_0.fix_type < 2:
        print("Waiting for GPS...:", vehicle.gps_0.fix_type)
        time.sleep(1)
        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    vehicle.flush()

    while not vehicle.armed and not api.exit:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.commands.takeoff(aTargetAltitude) # Take off to target altitude
    vehicle.flush()

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.commands.takeoff will execute immediately).
    while not api.exit:
        print(" Altitude: ", vehicle.location.alt)
        if vehicle.location.alt>=aTargetAltitude*0.95: #Just below target, in case of undershoot.
            print("Reached target altitude")
            break;
        time.sleep(1)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,            # confirmation
        heading,      # param 1, yaw in degrees
        0,            # param 2, yaw speed deg/s
        1,            # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)      # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def set_roi(location):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, # command
        0,           # confirmation
        0, 0, 0, 0,  # params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def set_speed(speed):
    # create the MAV_CMD_DO_CHANGE_SPEED command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
        0,   # confirmation
        0,   # param 1
        speed,  # speed
        0, 0, 0, 0, 0  # param 3 - 7
        )

    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
def set_home(aLocation, aCurrent=1):
    # create the MAV_CMD_DO_SET_HOME command: 
    # http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_home
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,   # command
        0,   # confirmation
        aCurrent,   # param 1: 1 to use current position, 2 to use the entered values.
        0, 0, 0,    # params 2-4
        aLocation.lat,
        aLocation.lon,
        aLocation.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def get_location_metres(original_location, dNorth, dEast):
    earth_radius=6378137.0   # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return Location(newlat, newlon, original_location.alt, original_location.is_relative)


def get_distance_metres(aLocation1, aLocation2):
    dlat        = aLocation2.lat - aLocation1.lat
    dlong       = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;

def goto_position_target_global_int(aLocation):
    print('goto_target_globalint_position')
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame      
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt,  # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above
                        # terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def goto(dNorth, dEast, gotoFunction = vehicle.commands.goto):
    currentLocation = vehicle.location
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    vehicle.flush()

    while not api.exit and vehicle.mode.name == "GUIDED": # Stop action if we are no longer in guided mode.
        remainingDistance = get_distance_metres(vehicle.location, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance <= targetDistance*0.01: # Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_global_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame      
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
           # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def trigger_drop(chstring):
    # Let's assume the drop servo is bound to channel 8 through the Pixhawk
    # Override channel 8 to 1800; this should trigger drop
    vehicle.channel_override = {chstring: 1800}
    time.sleep(0.5)
    # Cancel the override, returning the servo to normal
    vehicle.channel_override = {chstring: 0}

class PX4FlowReader(PX4Flow):
    def __init__(self, port):

        PX4Flow.__init__(self, port)

        # No readings yet
        self.SensorX, self.SensorY = None, None
        self.X, self.Y = None, None
        self.H = None
        self.Quality = None

        # Create logfile named by current date / time
        filename = 'px4flow_' + time.strftime('%d_%b_%Y_%H_%M_%S') + '.csv'
        self.logfile = open(filename, 'w')
        self.write_and_flush('Time (sec), Ground Dist (m), Flow X, Flow Y, Flow Comp X (m), Flow Comp Y '
                             '(m), Quality (/255),,')
        self.write_and_flush('X accum (m), Y accum (m)\n')

        # These will get the accumulated X,Y distances in meters
        self.timeSecPrev = None
        self.X_accum = 0
        self.Y_accum = 0

        self.count = 0

    # Implements missing method in parent class
    def update(self):

        # Grab raw sensor X,Y
        self.SensorX, self.SensorY = self.getFlow()

        # Grab computed X,Y in meters
        self.X, self.Y = self.getFlowComp()

        # Grab ground distance (height) in meters
        self.H = self.getGroundDistance()

        # Grab quality in percent
        self.Quality = self.getQuality()

        # Time in seconds
        timeSec = self.getTime() / 1e6

        # Computed flow in meters per second
        flowCompX, flowCompY = self.getFlowComp()

        self.write_and_flush('%6.3f, %+3.3f, %d, %d, %+3.3f, %+3.3f, %d' % \
                     (timeSec, self.H, self.SensorX, self.SensorY, self.X, self.Y, self.Quality))

        # After first iteration, compute accumulated distances
        if self.count:

            # Compute distance if elapsed time available
            if self.timeSecPrev:

                elapsedSec = timeSec - self.timeSecPrev

                # Elapsed time should never be more than a small fraction of a second
                if elapsedSec < 0.1:
                    self.X_accum += flowCompX * elapsedSec
                    self.Y_accum += flowCompY * elapsedSec

                self.timeSecPrev = timeSec

                self.write_and_flush(',,%+3.3f, %+3.3f' % (self.X_accum, self.Y_accum))

        self.write_and_flush('\n')

        # Update count for speed reporting
        self.count += 1
        self.timeSecPrev = timeSec

    def write_and_flush(self, s):

        self.logfile.write(s)
        self.logfile.flush()

class Carrier:
    def __init__(self, altitude, velocity, heading, hurdle_time):
        self.altitude = altitude
        self.velocity = velocity
        self.heading = heading
        self.hurdle_time = hurdle_time

    def infostring(self):
        return str(datetime.now()) + ":alt=" + str(c.altitude) + ",vel=" + str(c.velocity) + ",head=" +\
               str(c.heading) + "hur=" + str(c.hurdle_time) + ";\n"

# Broadcasts and listens on the socket
def sock_server(conn, c, flow_object=None):
    while True:
        try:
            # Sending message to connected client
            conn.send(c.infostring())
            # send_data = str(flow_object.update())
            # conn.sendall(send_data)
            data = conn.recv(1024)
            if data:
                print(data)
                if "exit" in str(data):
                    break
                if "drop" in str(data):
                    thread.start_new_thread(trigger_drop, ("8", ))
            else:
                continue
            time.sleep(0.01)
        except Exception as e:
            if '35' in str(e):
                pass
            else:
                print(e)
                break
    # came out of loop
    conn.close()

def read_bearing(filename):
    with open(filename) as infile:
        return int(infile.readline())

def sock_loop(s, c):
    while True:
        # obtain socket connection
        # wait to accept a connection - blocking call
        conn, addr = s.accept()
        if conn and addr:
            print('Connected with ' + addr[0] + ':' + str(addr[1]))
            thread.start_new_thread(sock_server, (conn, c))

def hurdle_detector(ld, c):
    last_100 = []
    while True:
        current_dist = int(ld.getDistance())
        if len(last_100) < 100:
            last_100.append(current_dist)
        else:
            last_100.pop(0)
            last_100.append(current_dist)
        average_dist = sum(last_100) / len(last_100)
        if (current_dist * 3) < average_dist:
            c.hurdle = 1
        elif c.hurdle > 0:
            c.hurdle -= 0.1
        else:
            c.hurdle = 0
        time.sleep(0.01)

FLOW_ENABLED = False
BEARING = 40             # degrees
TARGET_VELOCITY = 6      # m/s
TARGET_ALTITUDE = 4.5    # metres
ALTITUDE_FEEDBACK_GAIN = 0.4
ALTITUDE_FEEDBACK_POWER_GAIN_NONLINEAR = 0.7
ROLL_CORRECTION_GAIN = 0.05
RC1_CENTRE_CONSTANT = 1508
ROLL_BOUNDING_DEGREES = 35
MAX_LIDAR_ALTITUDE_METRES = 15
MIN_LIDAR_ALTITUDE_METRES = 0.2

try:
    BEARING = read_bearing("bearing.txt")
except Exception as e:
    print("Error reading bearing file")

# Initialise the LIDAR
lidar = Lidar_Lite()
if lidar.connect(1):
    # Returned -1: something bad happened.
    print("LIDAR connection failed. Check wiring.")
else:
    # Returned 0. Connection success.
    print("LIDAR connected")

# Take off to an altitude of 10 metres
arm_and_takeoff(10)

# Fly the vehicle in a straight line along a bearing, at a constant velocity of 6 metres per second.
# Face in the direction specified.
condition_yaw(BEARING)
bearing_radians = math.radians(BEARING)
x_comp = TARGET_VELOCITY * math.cos(bearing_radians)
y_comp = TARGET_VELOCITY * math.sin(bearing_radians)
send_global_velocity(x_comp, y_comp, 0)

if FLOW_ENABLED:
    flow_sensor = PX4FlowReader("/dev/ttyACM0")

# Socket-related code
HOST = ''     # Symbolic name meaning all available interfaces
PORT = 7898   # Arbitrary non-privileged port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')
# Bind socket to local host and port
try:
    s.bind((HOST, PORT))
except socket.error, msg:
    print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
    sys.exit()
print('Socket bind complete')
s.setblocking(False)
# Start listening on socket
s.listen(10)
print('Socket now listening')

# Create the carrier, leaving it empty for now
c = Carrier(None, None, None, None)

# Start listening continuously
thread.start_new_thread(sock_loop, (s, c))

# Start the hurdle detector
thread.start_new_thread(hurdle_detector, (lidar, c))

PRINT_STATEMENTS = False

# Main loop
while True:
    condition_yaw(BEARING)
    # Rangefind
    if PRINT_STATEMENTS:
        print(str(vehicle.location))
        print(re.findall(r"alt=[0-9]+\.[0-9]+", str(vehicle.location)))
    try:
        current_altitude = float(re.findall(r"alt=[0-9]+\.[0-9]+", str(vehicle.location))[0][4:])
        ground_distance = lidar.getDistance()
        if MIN_LIDAR_ALTITUDE_METRES < current_altitude < MAX_LIDAR_ALTITUDE_METRES:
            current_altitude = float(ground_distance) / 100
    except IndexError:
        # Let's assume you can't have negative altitude
        current_altitude = 0.0
    print("Altitude is", current_altitude)
    c.altitude = current_altitude
    if PRINT_STATEMENTS:
        print("Target altitude is", TARGET_ALTITUDE)

    # Calculate throttle correction
    altitude_diff = TARGET_ALTITUDE - current_altitude
    if PRINT_STATEMENTS:
        print("Altitude diff is", altitude_diff)
    z_comp = math.copysign(math.pow(abs(altitude_diff), ALTITUDE_FEEDBACK_POWER_GAIN_NONLINEAR), altitude_diff) \
        * -1 * ALTITUDE_FEEDBACK_GAIN
    if PRINT_STATEMENTS:
        print("Correction is", z_comp)

    # Calculate roll corrections
    rc1 = vehicle.channel_readback['1'] - RC1_CENTRE_CONSTANT
    if -10 < rc1 < 10:
        rc1 = 0
    rc_corr = rc1 * ROLL_CORRECTION_GAIN
    if rc_corr >= ROLL_BOUNDING_DEGREES:
        rc_corr = ROLL_BOUNDING_DEGREES
    elif rc_corr <= -1 * ROLL_BOUNDING_DEGREES:
        rc_corr = -1 * ROLL_BOUNDING_DEGREES

    if PRINT_STATEMENTS:
        print("Bearing correction is", rc_corr)

    bearing_radians = math.radians(BEARING + rc_corr)
    adjusted_velocity = abs(TARGET_VELOCITY / math.sin(math.radians(90 - rc_corr)))

    if PRINT_STATEMENTS:
        print("Adjusted velocity is", adjusted_velocity)

    x_comp = adjusted_velocity * math.cos(bearing_radians)
    y_comp = adjusted_velocity * math.sin(bearing_radians)
    c.heading = math.degrees(bearing_radians)
    
    # Apply correction
    send_global_velocity(x_comp, y_comp, z_comp)
    c.velocity = math.sqrt(x_comp**2 + y_comp**2)

    # Update the sensor
    if FLOW_ENABLED:
        flow_sensor.update()

    # Show sensor properties
    if FLOW_ENABLED:
        print(flow_sensor.X, flow_sensor.Y)

    # Short sleep to prevent weirdness
    time.sleep(0.05)
