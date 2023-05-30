#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
from skimage.morphology import skeletonize, medial_axis, thin
from scipy.spatial import distance as d
from scipy.spatial.distance import cdist
from scipy import ndimage
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import time
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import quat2euler
import torch

SAVE_PATH=False
DROP_EVERY_N = 4


class CenterLine(Node):

    def __init__(self):
        super().__init__('centerline')

        self.get_logger().info('init done')
        qos_profile = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.publisher = self.create_publisher(PointCloud2, '/path', 1)
        self.publisher2 = self.create_publisher(PointCloud2, '/scan2cloud', 1)
        self.publisher3 = self.create_publisher(OccupancyGrid, '/grid_augmented', 1)
        # qos_profile = QoSProfile(depth=1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=qos_profile)

        self.get_logger().info('publishers started')

        time.sleep(0.5)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/cartographer/tracked_pose',
            self.callback3,
            qos_profile=qos_profile)

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/cartographer/map',
            self.callback, 1)
        self.subscription2 = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback2,
            qos_profile=qos_profile)
        self.subscription  # prevent unused variable warning
        self.laser = None
        self.scans_received = 0
        self.map = None
        self.last_pose = None

        self.get_logger().info('node started')

        time.sleep(5.0)

        self.timer = self.create_timer(0.1, self.trajectory_timer)

    def callback(self, map):
        self.map = map

    def trajectory_timer(self):
        if self.map is None or self.laser is None or self.last_pose is None:
            return
        t0 = time.time()
        #Prevedeni 1D pole na 2D
        map = self.map
        l = self.map.data
        meta = self.map.info
        
        # prevedeni laser scanu do souradnic mapy #############################
        origin = meta.origin.position
        #get the direction
        direction = self.laser[0:2,:]-np.array([[origin.x],[origin.y]])
        #rotate it by theta (from grid origin)
        #the grid is in the x-y plane, the rotation is therefore around z axis
        #the quaternion looks like [cos(t/2),0,0,sin(t/2)]
        theta = 2*np.arccos(meta.origin.orientation.w)
        c = np.cos(theta)
        s = np.sin(theta)
        R = np.array([[c,-s],[s,c]])
        direction = np.matmul(R,direction)
        #from the known grid resolution, convert direction to coordinates
        coords = direction//meta.resolution
        coords = coords.astype(int)
        #######################################################################

        #unpack the list to numpy array
        arr = np.array(l)
        grid = arr.reshape((meta.height,meta.width)).T

        #Uprava mapy
        image = np.zeros_like(grid)
        image[np.logical_or(grid>=20, grid==-1)] = 1

        #pridej body ze scanu do mapy
        obstacles = np.zeros_like(image)
        coords = coords[:, np.logical_and(coords[0,:]<obstacles.shape[0], coords[1,:]<obstacles.shape[1])]
        try: 
            obstacles[coords[0,:],coords[1,:]] = 1
            obstacles = ndimage.binary_dilation(obstacles, iterations=4).astype(obstacles.dtype)
            image = np.clip(image+obstacles,0,1)
        except:
            self.get_logger().error('index error')
            self.get_logger().error(str(image.shape)) 
            self.get_logger().error(str([np.min(coords[0,:]), np.max(coords[0,:])])) 
            self.get_logger().error(str([np.min(coords[1,:]), np.max(coords[1,:])])) 

        #Dilatace prekazek
        image = ndimage.binary_dilation(image, iterations=4).astype(image.dtype)

        aug_list = (99*image).T.reshape([meta.height*meta.width]).tolist()
        o = OccupancyGrid()
        o.header.stamp = rclpy.clock.Clock().now().to_msg()
        o.header.frame_id = "map"
        o.info = meta
        o.data = aug_list
        self.publisher3.publish(o)
        
        #Vytvoreni skeletonu
        image = np.logical_not(image)
        skeleton = skeletonize(image).astype(np.uint16).T

        #smooth-out the skeleton and remove small branches
        #skeleton = ndimage.binary_dilation(skeleton, iterations=15).astype(image.dtype)
        #skeleton = skeletonize(skeleton).astype(np.uint16)
        # skeleton = ndimage.binary_dilation(skeleton, iterations=15).astype(image.dtype)
        # skeleton = skeletonize(skeleton).astype(np.uint16).T

        #Priprava skeletonu pro nasledne serazeni
        id = np.nonzero(skeleton)
        id = np.concatenate((id[1][None,:],id[0][None,:]),axis=0)

        #Prevedeni z indexu pole do souradneho systemu mapy
        id_h = np.concatenate((id, np.ones((1, id.shape[1]))))
        tf = np.array([[map.info.resolution,0,map.info.origin.position.x], [0,map.info.resolution,map.info.origin.position.y], [0,0,1]])
        id_tf = np.matmul(tf, id_h)[0:2,:]

        id_tf = np.concatenate((id_tf, np.zeros((1,id_tf.shape[1]))))
        h = Header()
        h.stamp = rclpy.clock.Clock().now().to_msg()
        h.frame_id = "map"
        cloud = pc2.create_cloud_xyz32(h, id_tf.T.tolist())
        self.publisher.publish(cloud)

        if SAVE_PATH:
            np.savetxt('track.txt', id_tf.T)

        t1 = time.time()
        # self.get_logger().info("Done! %f" % (t1-t0))

    def callback2(self, msg):
        if self.last_pose is None:
            return
        t0 = time.time()
        #self.scans_received += 1
        #if self.scans_received != DROP_EVERY_N:
        #    return
        #self.scans_received = 0
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        c = np.cos(angles)
        s = np.sin(angles)
        rot = np.array([[c,-s],[s,c]])  #2*2*N
        rot = np.swapaxes(np.swapaxes(rot,0,1),0,2)
        x_axis = np.repeat(np.array([[[1.],[0.]]]), len(ranges), axis=0)
        # points_2d = np.zeros((2,len(ranges)))
        # for i in range(len(ranges)):
        #     points_2d[:, i:i+1] = ranges[i]*np.matmul(rot[:,:,i], x_axis)
        rot_t = torch.from_numpy(rot)
        x_axis_t = torch.from_numpy(x_axis)
        points_2d = ranges*(torch.matmul(rot_t, x_axis_t).numpy().reshape((len(ranges),2)).T)
        
        points = np.concatenate((points_2d, np.zeros_like(ranges)[None,:]))

        h = Header()
        h.stamp = rclpy.clock.Clock().now().to_msg()
        h.frame_id = "laser"
        cloud = pc2.create_cloud_xyz32(h, points.T.tolist())
        self.publisher2.publish(cloud)

        # try:
        #     # trans = self.tf_buffer.lookup_transform("map", "laser", rclpy.time.Time())
        #     trans = self.tf_buffer.lookup_transform("map", "laser", msg.header.stamp)
        # except TransformException as e:
        #     self.get_logger().error(str(e))
        #     return
        # q = (
        #     trans.transform.rotation.w,
        #     trans.transform.rotation.x,
        #     trans.transform.rotation.y,
        #     trans.transform.rotation.z
        # )
        # euler = quat2euler(q)
        # roll, pitch, yaw = euler
        # c = np.cos(yaw)
        # s = np.sin(yaw)
        # x = trans.transform.translation.x
        # y = trans.transform.translation.y

        q = (
            self.last_pose.orientation.w,
            self.last_pose.orientation.x,
            self.last_pose.orientation.y,
            self.last_pose.orientation.z
        )
        euler = quat2euler(q)
        roll, pitch, yaw = euler
        c = np.cos(yaw)
        s = np.sin(yaw)
        x = self.last_pose.position.x
        y = self.last_pose.position.y

        tf = np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,0],[0,0,0,1]])
        points_h = np.concatenate((points, np.ones((1,points.shape[1]))))
        self.laser = np.matmul(tf,points_h)
        t1 = time.time()
        self.get_logger().info("Done! %f" % (t1-t0))

    def callback3(self, msg):
        self.last_pose = msg.pose


def DistMatrix(x): #vypocet matice urcujici vzdalenosti mezi jednotlivymi body
    dim = len(x)
    M = np.zeros((dim, dim))

    for i in range(dim):
        for j in range(i+1, dim):
            M[i][j] = d.euclidean(x[i], x[j])
            M[j][i] = M[i][j]

    return M


def sortCenterLine(v): #serazeni bodu stredove cary
    new = np.zeros((len(v), 2))
    v = np.array(v)
    idx = list(v[:, 0]).index(min(i for i in v[:, 0] if i >= 0))
    # D = DistMatrix(v)
    D = cdist(v, v)
    minimum = list(D[idx]).index(np.min(D[idx][np.nonzero(D[idx])]))
    D = np.delete(D, idx, 0)
    D = np.delete(D, idx, 1)
    new[0][0] = v[idx][0]
    new[0][1] = v[idx][1]
    v = np.delete(v, idx, axis=0)
    if idx < minimum:
        idx = minimum - 1
    else:
        idx = minimum

    for i in range(1, len(new) - 1):
        minimum = list(D[idx]).index(np.min(D[idx][np.nonzero(D[idx])]))
        new[i][0] = v[idx][0]
        new[i][1] = v[idx][1]
        v = np.delete(v, idx, axis=0)
        D = np.delete(D, idx, 0)
        D = np.delete(D, idx, 1)
        if idx < minimum:
            idx = minimum - 1
        else:
            idx = minimum
    new[len(new) - 1][0] = v[0][0]
    new[len(new) - 1][1] = v[0][1]

    return new


def main(args=None):
    rclpy.init(args=args)
    print("Creating the centerline ...")
    centerline = CenterLine()
    rclpy.spin(centerline)
    centerline.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    '''
    Tento program vytvori v mape stredovou caru
    '''
    main()
