#!/usr/bin/env python
import rospy
import time
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import SetBool


class my_node:
    def __init__(self):
        self.map_pub = rospy.Publisher('/ecte477/map', OccupancyGrid, queue_size=10)
        self.e_path_pub = rospy.Publisher('/ecte477/e_path', Path, queue_size=10)
        self.r_path_pub = rospy.Publisher('/ecte477/r_path', Path, queue_size=10)
        self.explore_service = rospy.ServiceProxy('/explore/explore_service', SetBool)
        self.move_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.frontiers_sub = rospy.Subscriber('/explore/frontiers', MarkerArray, self.callback_frontiers)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.explore_started = False
        self.exploration_finished = False
        self.e_path = Path()
        self.e_path.header.frame_id = 'odom'
        self.r_path = Path()
        self.r_path.header.frame_id = 'odom'
        self.prev_pose = None

    def callback_map(self, data):
        self.map_pub.publish(data)

    def callback_odom(self, data):
        if not self.explore_started:
            self.explore_started = True
            time.sleep(8)
            self.explore_service(True)
        else:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose = data.pose.pose
            self.e_path.poses.append(pose)
            self.e_path_pub.publish(self.e_path)

          
            if self.exploration_finished:
                if self.prev_pose is not None:
                    dist = ((data.pose.pose.position.x - self.prev_pose.position.x) ** 2 + (data.pose.pose.position.y - self.prev_pose.position.y) ** 2) ** 0.5
                    if dist > 0.1:
                        self.r_path.poses.append(pose)
                        self.r_path_pub.publish(self.r_path)
                self.prev_pose = data.pose.pose

    def callback_frontiers(self, frontiers):
        if not self.exploration_finished and len(frontiers.markers) == 0:
            self.exploration_finished = True
            time.sleep(5)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            self.move_pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)
    rospy.loginfo("Starting My Node!")
    mn = my_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down My Node!")


	

