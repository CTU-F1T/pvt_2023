#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from transforms3d.euler import quat2euler
from geometry_msgs.msg import PoseStamped
import csv
from drive_api_msgs.msg import DriveApiValues
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
import threading

la = 1.0
k = 0.6
ki = 0.
PUBLISH_PATH = False

def get_angle(Va, Vb, Vn):
    """returns oriented angle of rotation from Va to Vb"""
    # https://stackoverflow.com/a/33920320
    return float(np.arctan2(np.matmul(np.cross(Va, Vb, axis=0).T, Vn), np.matmul(Va.T, Vb)))

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        drive_topic = '/drive_api/command'
        qos_profile = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # self.drive_publisher = self.create_publisher(DriveApiValues, drive_topic, qos_profile=qos_profile)
        # self.way_pub = self.create_publisher(Marker, 'lookahead', qos_profile=qos_profile)
        # self.path_publisher = self.create_publisher(PointCloud2, '/path_tf', qos_profile=qos_profile)
        self.drive_publisher = self.create_publisher(DriveApiValues, drive_topic, qos_profile=1)
        # self.way_pub = self.create_publisher(Marker, 'lookahead', qos_profile=1)
        # self.path_publisher = self.create_publisher(PointCloud2, '/path_tf', qos_profile=1)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/cartographer/tracked_pose',
            self.callback5,
            qos_profile=qos_profile)

        self.subscription = self.create_subscription(
            Marker,
            '/lookPoint',
            self.callback4,
            qos_profile=qos_profile)

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self, qos = qos_profile)
        # self.tf_listener = TransformListener(self.tf_buffer, self, qos = 1)

        self.lock = threading.RLock()
        self.points = None

        # self.marker = Marker()
        # self.marker.header.frame_id = "base_footprint"
        # self.marker.type = Marker.SPHERE
        # self.marker.action = Marker.ADD
        # self.marker.pose.position.z = 0.
        # self.marker.scale.x = 0.3
        # self.marker.scale.y = 0.3
        # self.marker.scale.z = 0.3
        # self.marker.color.a = 1.0
        # self.marker.color.r = 0.0
        # self.marker.color.g = 1.0
        # self.marker.color.b = 0.0
        self.marker_pose = None
        self.sum = 0

        # self.subscription = self.create_subscription(PointCloud2, '/path', self.path_callback, qos_profile=qos_profile)
        self.subscription = self.create_subscription(PointCloud2, '/path', self.path_callback, qos_profile=1)

        #self.timer = self.create_timer(0.05, self.control)

    def control(self):
        # Extract position and orientation from tf tree
        if self.points is None:
            return
        
        self.lock.acquire(blocking=True,timeout=-1)
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(e)
            return
        q = (
            trans.transform.rotation.w,
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z
        )
        euler = quat2euler(q)
        roll, pitch, yaw = euler  
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        
        # transform path to base_footprint
        c = np.cos(-yaw)
        s = np.sin(-yaw)
        mat = np.array([[c,-s,0,-x*c+y*s],[s,c,0,-x*s-y*c],[0,0,1,0],[0,0,0,1]])
        points_tf = np.matmul(mat, np.concatenate((self.points, np.ones((1,self.points.shape[1])))))[0:3,:]

        self.lock.release()

        if PUBLISH_PATH:
            h = Header()
            h.stamp = rclpy.clock.Clock().now().to_msg()
            h.frame_id = "base_footprint"
            cloud = pc2.create_cloud_xyz32(h, points_tf.T.tolist())
            self.path_publisher.publish(cloud)

        # find point in front of the car that is closest to lookahead
        val = np.abs(np.linalg.norm(points_tf,axis=0)-la)
        val[points_tf[0,:]<0.1] += 100000  #set some really high distance to discard all points behind the car
        i_min = np.argmin(val)

        angle = get_angle(np.array([[1],[0],[0]]), points_tf[:,i_min:i_min+1], np.array([[0],[0],[1]]))

        self.get_logger().info(str(points_tf[:,i_min]))

        # Publish visualization marker for lookahead point
        self.marker.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.marker.pose.position.x = points_tf[0,i_min]
        self.marker.pose.position.y = points_tf[1,i_min]
        self.way_pub.publish(self.marker)

        # Publish drive command
        dv_msg = DriveApiValues()
        if angle < 0:
            dv_msg.steering = float(-angle)
            dv_msg.right = True
        else:
            dv_msg.steering = float(angle)
            dv_msg.right = False
        self.sum += ki*dv_msg.steering
        dv_msg.steering = k*dv_msg.steering+self.sum
        if dv_msg.steering > 1:
            self.sum = self.sum - (dv_msg.steering-1)
            dv_msg.steering = 1.
        elif dv_msg.steering < -1:
            self.sum = self.sum - (dv_msg.steering-1)
            dv_msg.steering = -1.
        dv_msg.velocity = 0.3
       	if abs(dv_msg.steering) > 0.2:
            dv_msg.velocity = 0.15
        dv_msg.forward = True
        self.drive_publisher.publish(dv_msg)
        self.get_logger().info('published msg %s' % (str((dv_msg.velocity,dv_msg.steering))))

    def path_callback(self, msg):
        if self.lock.acquire(blocking=True, timeout=0.2):
            points = pc2.read_points_list(msg)
            self.points = np.array(points).T  #3*N
            self.lock.release()

    def callback3(self, msg):
        #t = rclpy.clock.Clock().now()
        #t2 = msg.header.stamp
        #self.get_logger().info("pose received, now: %s, stamp: %s" % (str(t), str(t2))
        self.lock.acquire(blocking=True,timeout=-1)
        if self.points is None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y

        q = (
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        )

        euler = quat2euler(q)
        roll, pitch, yaw = euler  

        # transform path to base_footprint
        c = np.cos(-yaw)
        s = np.sin(-yaw)
        mat = np.array([[c,-s,0,-x*c+y*s],[s,c,0,-x*s-y*c],[0,0,1,0],[0,0,0,1]])
        points_tf = np.matmul(mat, np.concatenate((self.points, np.ones((1,self.points.shape[1])))))[0:3,:]

        self.lock.release()

        if PUBLISH_PATH:
            h = Header()
            h.stamp = rclpy.clock.Clock().now().to_msg()
            h.frame_id = "base_footprint"
            cloud = pc2.create_cloud_xyz32(h, points_tf.T.tolist())
            self.path_publisher.publish(cloud)

        # find point in front of the car that is closest to lookahead
        val = np.abs(np.linalg.norm(points_tf,axis=0)-la)
        val[points_tf[0,:]<0.1] += 100000  #set some really high distance to discard all points behind the car
        i_min = np.argmin(val)

        angle = get_angle(np.array([[1],[0],[0]]), points_tf[:,i_min:i_min+1], np.array([[0],[0],[1]]))

        self.get_logger().info(str(points_tf[:,i_min]))

        # Publish visualization marker for lookahead point
        self.marker.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.marker.pose.position.x = points_tf[0,i_min]
        self.marker.pose.position.y = points_tf[1,i_min]
        self.way_pub.publish(self.marker)

        # Publish drive command
        dv_msg = DriveApiValues()
        if angle < 0:
            dv_msg.steering = float(-angle)
            dv_msg.right = True
        else:
            dv_msg.steering = float(angle)
            dv_msg.right = False
        self.sum += ki*dv_msg.steering
        self.sum = np.clip(self.sum, -0.2, 0.2)
        dv_msg.steering = k*dv_msg.steering+self.sum
        # if dv_msg.steering > 1:
        #     self.sum = self.sum - (dv_msg.steering-1)
        #     dv_msg.steering = 1.
        # elif dv_msg.steering < -1:
        #     self.sum = self.sum - (dv_msg.steering+1)
        #     dv_msg.steering = -1.
        dv_msg.steering = np.clip(dv_msg.steering, -1, 1)
            
        dv_msg.velocity = 0.25*np.exp(-4*angle**2)
        dv_msg.velocity = np.clip(dv_msg.velocity, 0.1, 100.)
       	#if abs(dv_msg.steering) > 0.2:
            #dv_msg.velocity = 0.15
        dv_msg.forward = True
        self.drive_publisher.publish(dv_msg)
        self.get_logger().info('published msg %s' % (str((dv_msg.velocity,dv_msg.steering))))

    def callback4(self, msg):
        self.marker_pose = np.array([[msg.pose.position.x], [msg.pose.position.y], [0.0]])

    def callback5(self, msg):
        if self.marker_pose is None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y

        q = (
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        )

        euler = quat2euler(q)
        roll, pitch, yaw = euler  

        # transform point to base_footprint
        c = np.cos(-yaw)
        s = np.sin(-yaw)
        mat = np.array([[c,-s,0,-x*c+y*s],[s,c,0,-x*s-y*c],[0,0,1,0],[0,0,0,1]])
        point_tf = np.matmul(mat, np.concatenate((self.marker_pose, np.array([[1]]))))[0:3,:]

        angle = get_angle(np.array([[1],[0],[0]]), point_tf, np.array([[0],[0],[1]]))

        # Publish drive command
        dv_msg = DriveApiValues()
        if angle < 0:
            dv_msg.steering = float(-angle)
            dv_msg.right = True
        else:
            dv_msg.steering = float(angle)
            dv_msg.right = False
        self.sum += ki*dv_msg.steering
        self.sum = np.clip(self.sum, -0.2, 0.2)
        dv_msg.steering = k*dv_msg.steering+self.sum
        dv_msg.steering = np.clip(dv_msg.steering, -1, 1)
            
        dv_msg.velocity = 0.2*np.exp(-4*angle**2)
        dv_msg.velocity = np.clip(dv_msg.velocity, 0.1, 0.2)
        dv_msg.forward = True
        self.drive_publisher.publish(dv_msg)
        self.get_logger().info('published msg %s' % (str((dv_msg.velocity,dv_msg.steering))))

        
def main(args=None):
    rclpy.init(args=args)
    pp = PurePursuit()
    rclpy.spin(pp)
    pp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
