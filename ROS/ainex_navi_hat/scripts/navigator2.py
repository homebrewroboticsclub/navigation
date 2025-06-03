#!/usr/bin/env python3
# encoding: utf-8
import math
import rospy
from sensor_msgs.msg import NavSatFix, MagneticField
from ainex_kinematics.gait_manager import GaitManager

class NavigationNode:
    def __init__(self):
        rospy.init_node('gps_navigation_node')
        
        # Route parameters
        # TODO transfer route input from UI
        self.initial_point = [43.109165, 131.878811]  # Latitude, longitude
        self.waypoints = [
            [43.109255, 131.878197],
            [43.109255, 131.878493],
            [43.109455, 131.878992]
        ]
        self.tolerance = 3.0  # Allowable distance to target (m)
        self.angle_threshold = 15.0  # Threshold for turning (degrees)
        self.max_rotation_step = 10.0  # Max. rotation per step (degrees)
        self.current_goal_index = 0
        
        # Current data
        self.current_lat = self.initial_point[0]
        self.current_lon = self.initial_point[1]
        self.current_heading = 0.0  # Azimuth in degrees [0, 360)
        self.got_fix = False
        self.got_mag = False
        
        # GaitManager initialization
        self.gait_manager = GaitManager()
        rospy.loginfo("GaitManager initialized")
        
        # Topic subscriptions
        rospy.Subscriber('/fix', NavSatFix, self.fix_callback)
        rospy.Subscriber('/mag', MagneticField, self.mag_callback)
        rospy.loginfo("Subscriptions created")

    def fix_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        if not self.got_fix:
            rospy.loginfo(f"First GPS data: {self.current_lat}, {self.current_lon}")
            self.got_fix = True

    def mag_callback(self, msg):
        # Azimuth conversion [-180, 180] → [0, 360)
        heading = msg.magnetic_field.z
        self.current_heading = heading + 360 if heading < 0 else heading
        if not self.got_mag:
            rospy.loginfo(f"First compass data: {self.current_heading}°")
            self.got_mag = True

    @staticmethod
    def calculate_bearing_and_distance(lat1, lon1, lat2, lon2):
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Coordinate differences
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        
        # Distance calculation (haversine formula)
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth radius in meters
        
        # Azimuth calculation
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.atan2(y, x)
        bearing = math.degrees(bearing) % 360
        
        return bearing, distance

    @staticmethod
    def normalize_angle_diff(diff):
        """Normalize angle difference to the range [-180, 180]."""
        return (diff + 180) % 360 - 180

    def wait_for_topics(self, timeout=30.0):
        """Wait for topics to appear with a timeout. If data does not arrive, use default values."""
        # TODO pull up calibration values for azimuth (magnetometer and GPS, add local coordinate system to INS, for error correction after manual refinement)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.got_fix and self.got_mag:
                return True
                
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn(f"Topic wait timeout: GPS={self.got_fix}, magnetometer={self.got_mag}")
                return False
                
            rospy.sleep(0.1)

    def run(self):
        rospy.loginfo("Start of navigation operation")
        
        # Waiting for data
        if not self.wait_for_topics():
            rospy.logerr("Failed to get initial data. DEF.")
           

        rate = rospy.Rate(5)  # 5 HZ (frequency of navigation commands)
        
        while not rospy.is_shutdown():
            # Check route completion
            if self.current_goal_index >= len(self.waypoints):
                self.gait_manager.stop()
                rospy.loginfo("Route completed!")
                rospy.signal_shutdown("Route completed")
                return

            # Current target
            goal = self.waypoints[self.current_goal_index]
            goal_lat, goal_lon = goal
            
            # Calculation of movement parameters
            bearing, distance = self.calculate_bearing_and_distance(
                self.current_lat, self.current_lon, goal_lat, goal_lon
            )
            
            # Logging every 2 seconds
            if rospy.Time.now().to_sec() % 2 < 0.1:
                rospy.loginfo(f"Target #{self.current_goal_index}: Distance: {distance:.2f} m, Azimuth: {bearing:.1f}°")
            
            # Check target achievement
            if distance <= self.tolerance:
                rospy.loginfo(f"Target #{self.current_goal_index} reached!")
                self.current_goal_index += 1
                continue
            
            # Calculation of rotation angle
            angle_diff = self.normalize_angle_diff(bearing - self.current_heading)
            
            # Motion control
            if abs(angle_diff) > self.angle_threshold:
                # Turn in place
                x_step = 0.0
                yaw = math.copysign(min(self.max_rotation_step, abs(angle_diff)), angle_diff)
                if rospy.Time.now().to_sec() % 2 < 0.1:
                    rospy.loginfo(f"Turn: {yaw:.1f}°")
            else:
                # Forward movement with correction
                x_step = 0.01
                yaw = math.copysign(min(5.0, abs(angle_diff)), angle_diff)  # Soft correction
                if rospy.Time.now().to_sec() % 2 < 0.1:
                    rospy.loginfo(f"Movement: step {x_step:.3f} m, correction {yaw:.1f}°")
            
            # Sending motion command
            self.gait_manager.move(2, x_step, 0, yaw)
            rate.sleep()
            # TODO add event loop to handle camera events, possibly SLAM based on camera. Add reaction to falls, for auto-recovery and continuation
            # TODO Refine walking algorithm, on OS from IMU.

if __name__ == '__main__':
    try:
        node = NavigationNode()
        node.run()
        rospy.spin()  # Main message processing loop
    except rospy.ROSInterruptException:
        rospy.loginfo("Node finished")
