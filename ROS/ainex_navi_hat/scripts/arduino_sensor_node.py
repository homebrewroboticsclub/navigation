#!/usr/bin/env python3
import rospy
import serial
from sensor_msgs.msg import NavSatFix, MagneticField
from std_msgs.msg import Header

def parse_line(line):
    """
    Parse a line from Arduino.
    Expected formats:
    GPS,lat,lon,alt
    MAG,x,y,z,a
    Refused ROS Serial due to unstable operation and higher resource consumption on Arduino
    """
    parts = line.strip().split(',')
    if not parts:
        return None, None

    if parts[0] == 'GPS' and len(parts) == 4:
        try:
            lat = float(parts[1])
            lon = float(parts[2])
            alt = float(parts[3])
            return 'GPS', (lat, lon, alt)
        except ValueError:
            return None, None

    elif parts[0] == 'MAG' and len(parts) == 5:
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            a = float(parts[4])         
            return 'MAG', (x, y, z, a)
        except ValueError:
            return None, None

    return None, None

def main():
    rospy.init_node('arduino_sensor_node')

    gps_pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
    mag_pub = rospy.Publisher('/mag', MagneticField, queue_size=10)

    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baud = rospy.get_param('~baudrate', 115200)

    try:
        ser = serial.Serial(port, baud, timeout=1)
        rospy.loginfo(f"Opened serial port {port} at {baud} baud")
    except serial.SerialException as e:
        rospy.logerr(f"Cannot open serial port: {e}")
        return

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                sensor_type, data = parse_line(line)

                if sensor_type == 'GPS':
                    lat, lon, alt = data
                    msg = NavSatFix()
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.latitude = lat
                    msg.longitude = lon
                    msg.altitude = alt
                    # GPS status (0 = fix)
                    msg.status.status = 0
                    msg.status.service = 1  # GPS
                    # Positional accuracy can be added if available
                    gps_pub.publish(msg)

                elif sensor_type == 'MAG':
                    x, y, z, a = data
                    msg = MagneticField()
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.magnetic_field.x = x
                    msg.magnetic_field.y = y
                    # Temporary solution for transmitting azimuth data to the standard sensor topic.
                    msg.magnetic_field.z = a
                    
                    mag_pub.publish(msg)

        except Exception as e:
            rospy.logwarn(f"Error reading serial data: {e}")

        rate.sleep()

if __name__ == '__main__':
    main()
