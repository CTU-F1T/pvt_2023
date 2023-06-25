#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from transforms3d.euler import quat2euler
from geometry_msgs.msg import PoseStamped
from drive_api_msgs.msg import DriveApiValues
from sensor_msgs_py import point_cloud2 as pc2
import threading


def get_angle(Va, Vb, Vn):
    """returns oriented angle of rotation from Va to Vb"""
    # https://stackoverflow.com/a/33920320
    return float(np.arctan2(np.matmul(np.cross(Va, Vb, axis=0).T, Vn), np.matmul(Va.T, Vb)))

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        drive_topic = '/drive_api/command'
        qos_profile = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.drive_publisher = self.create_publisher(DriveApiValues, drive_topic, qos_profile=1)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/cartographer/tracked_pose',
            self.tracked_pose_cb,
            qos_profile=qos_profile)

        self.subscription = self.create_subscription(
            Marker,
            '/lookahead_point',
            self.lookahead_point_cb,
            qos_profile=qos_profile)

        self.lock = threading.RLock()
        self.points = None

        self.marker_pose = None
        self.sum = 0
        self.Kp = 0.6 # P constant
        self.Ki = 0.  # I constant
        self.speed = 0.2

        self.subscription = self.create_subscription(PointCloud2, '/path', self.path_callback, qos_profile=1)

    def path_callback(self, msg):
        if self.lock.acquire(blocking=True, timeout=0.2):
            points = pc2.read_points_list(msg)
            self.points = np.array(points).T  #3*N
            self.lock.release()

    def lookahead_point_cb(self, msg):
        self.marker_pose = np.array([[msg.pose.position.x], [msg.pose.position.y], [0.0]])

    def tracked_pose_cb(self, msg):
        if self.marker_pose is None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y

        # check whether DFS returned correctly and stop the car if not
        if np.allclose(self.marker_pose[0,0],-1000.) and np.allclose(self.marker_pose[1,0], -1000.):
            self.get_logger().fatal("STOP, invalid point")
            dv_msg = DriveApiValues()
            dv_msg.steering = 0.
            dv_msg.right = True
                
            dv_msg.velocity = 0.
            dv_msg.forward = True
            self.drive_publisher.publish(dv_msg)
            return

        q = (
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        )

        # quaternion to euler
        euler = quat2euler(q)
        roll, pitch, yaw = euler  

        # transform point to base_footprint
        c = np.cos(-yaw)
        s = np.sin(-yaw)
        mat = np.array([[c,-s,0,-x*c+y*s],[s,c,0,-x*s-y*c],[0,0,1,0],[0,0,0,1]])
        point_tf = np.matmul(mat, np.concatenate((self.marker_pose, np.array([[1]]))))[0:3,:]

        # get current angle
        angle = get_angle(np.array([[1],[0],[0]]), point_tf, np.array([[0],[0],[1]]))

        # Publish drive command
        dv_msg = DriveApiValues()
        if angle < 0:
            dv_msg.steering = float(-angle)
            dv_msg.right = True
        else:
            dv_msg.steering = float(angle)
            dv_msg.right = False

        # PI controller
        self.sum += self.Ki*dv_msg.steering
        self.sum = np.clip(self.sum, -0.2, 0.2)
        dv_msg.steering = self.Kp*dv_msg.steering+self.sum

        # steering saturation
        dv_msg.steering = np.clip(dv_msg.steering, -1, 1)
        dv_msg.velocity = self.speed*np.exp(-4*angle**2)

        # velocity saturation
        dv_msg.velocity = np.clip(dv_msg.velocity, 0.1, 0.2)
        dv_msg.forward = True
        self.drive_publisher.publish(dv_msg)

        
def main(args=None):
    rclpy.init(args=args)
    pp = PurePursuit()
    rclpy.spin(pp)
    pp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
