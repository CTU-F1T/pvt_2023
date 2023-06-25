#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
from skimage.morphology import skeletonize
from scipy import ndimage
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import time
from sensor_msgs_py import point_cloud2 as pc2
from transforms3d.euler import quat2euler
import torch
from collections import deque
from visualization_msgs.msg import Marker

SAVE_PATH = False
PUBLISH_DEBUG = False


class CenterLine(Node):

    def __init__(self):
        super().__init__('centerline')

        qos_profile = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.publisher1 = self.create_publisher(PointCloud2, '/path', 1)
        self.publisher2 = self.create_publisher(OccupancyGrid, '/grid_augmented', 1)
        self.publisher3 = self.create_publisher(Marker, '/lookahead_point', 1)

        self.get_logger().info('publishers started')

        time.sleep(0.5)

        self.tracked_pose_sub = self.create_subscription(
            PoseStamped,
            '/cartographer/tracked_pose',
            self.tracked_pose_cb,
            qos_profile=qos_profile)

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/cartographer/map',
            self.callback, 1)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            qos_profile=qos_profile)
        self.laser = None
        # self.scans_received = 0
        self.map = None
        self.last_pose = None

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.z = 0.
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0

        time.sleep(5.0)

        self.timer = self.create_timer(0.1, self.trajectory_timer)
        self.get_logger().info('node started')

    def callback(self, map):
        self.map = map

    def trajectory_timer(self):
        if self.map is None or self.laser is None or self.last_pose is None:
            return

        # 1D array -> 2D array
        map = self.map
        l = self.map.data
        meta = self.map.info
        
        # convert laser coords to map frame #############################
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

        # convert -1 to 99 to 0-1
        image = np.zeros_like(grid)
        image[np.logical_or(grid>=20, grid==-1)] = 1

        # add LIDAR scan points to map
        obstacles = np.zeros_like(image)
        coords = coords[:, np.logical_and(coords[0,:]<obstacles.shape[0], coords[1,:]<obstacles.shape[1])]
        coords = coords[:, np.logical_and(coords[0,:]>=0, coords[1,:]>=0)]
        try: 
            obstacles[coords[0,:],coords[1,:]] = 1
            obstacles = ndimage.binary_dilation(obstacles, iterations=6).astype(obstacles.dtype)
            image = np.clip(image+obstacles,0,1)
        except:
            self.get_logger().error('index error')
            self.get_logger().error(str(image.shape)) 
            self.get_logger().error(str([np.min(coords[0,:]), np.max(coords[0,:])])) 
            self.get_logger().error(str([np.min(coords[1,:]), np.max(coords[1,:])])) 

        # Obstacles diletation
        image = ndimage.binary_dilation(image, iterations=5).astype(image.dtype)

        if PUBLISH_DEBUG:
            aug_list = (99*image).T.reshape([meta.height*meta.width]).tolist()
            o = OccupancyGrid()
            o.header.stamp = rclpy.clock.Clock().now().to_msg()
            o.header.frame_id = "map"
            o.info = meta
            o.data = aug_list
            self.publisher2.publish(o)
        
        # Create skeleton
        image = np.logical_not(image)
        skeleton = skeletonize(image).astype(np.uint16).T

        # Get current position of the car
        direction = np.array([[self.last_pose.position.x],[self.last_pose.position.y]])-np.array([[origin.x],[origin.y]])
        #rotate it by theta (from grid origin)
        #the grid is in the x-y plane, the rotation is therefore around z axis
        #the quaternion looks like [cos(t/2),0,0,sin(t/2)]
        theta = 2*np.arccos(meta.origin.orientation.w)
        c = np.cos(theta)
        s = np.sin(theta)
        R = np.array([[c,-s],[s,c]])
        direction = np.matmul(R,direction)
        #from the known grid resolution, convert direction to coordinates
        posRob = direction//meta.resolution
        posRob = [int(posRob[0]), int(posRob[1])]

        moves=[[1,0],[0,1],[0,-1],[-1,0]]
        NearBod=None
        visited = np.zeros_like(skeleton)
        if(posRob[0]<0 or posRob[0]>=skeleton.shape[1] or posRob[1]<0 or posRob[1]>=skeleton.shape[0]):
            return
        

        # BFS - find the nearest point on map
        q=deque()
        pozice=[posRob[0],posRob[1]]
        q.append(pozice)
        visited[posRob[1],posRob[0]]=1
        while(q):
            x,y=q.popleft()
            if(NearBod != None):
                break
            for mv in moves:
                xNew=mv[0]+x
                yNew=mv[1]+y
                if xNew>=0 and xNew<skeleton.shape[1] and yNew>=0 and yNew<skeleton.shape[0]:
                    if(visited[yNew,xNew]==0):
                        q.append([xNew,yNew])
                        visited[yNew, xNew]=1
                        if(skeleton[yNew][xNew]==1):
                            NearBod=[yNew, xNew]
                            break
                
        # get car orienation in map frame
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
        
        move=[[-1,0],[-1,-1],[0,-1],[1,-1],[1,0],[1,1],[0,1],[-1,1]]
        angle=yaw
        angle+=np.pi
        angle*=(8/(2*np.pi))
        angle=np.round(angle)
        index=angle%8
        
        # DFS overhead
        stack=[]
        visited2 = np.zeros_like(skeleton)
        visited2[NearBod[0],NearBod[1]]=1

        # find first point for DFS
        for i in range(int(index-3),int(index+4)):
            i=i%8
            if(skeleton[NearBod[0]+move[i][1],NearBod[1]+move[i][0]]==1):
                stack.append([NearBod[1]+move[i][0],NearBod[0]+move[i][1],0])
                visited2[NearBod[0]+move[i][1],NearBod[1]+move[i][0]]=1

        # DFS implementation
        found=False
        while(len(stack)!=0):
            x,y,delka=stack.pop()
            if(delka>=40):
                self.marker.header.stamp = rclpy.clock.Clock().now().to_msg()
                self.marker.pose.position.y = float(y*meta.resolution+origin.y)
                self.marker.pose.position.x = float(x*meta.resolution+origin.x)
                self.publisher3.publish(self.marker)
                found=True
                break
            for i in range(int(index-3),int(index+4)):
                i=i%8
                xNew=x+move[i][0]
                yNew=y+move[i][1]
                if(skeleton[yNew][xNew]==1):
                    if(visited2[yNew][xNew]==0):
                        stack.append([xNew,yNew,delka+1])
                        visited2[yNew,xNew] = 1

        if not found:
            self.get_logger().fatal("no point, stop")
            self.marker.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.marker.pose.position.y = -1000.
            self.marker.pose.position.x = -1000.
            self.publisher3.publish(self.marker)

        skeleton[NearBod[0]]
        # Prepare skeleton for ordering
        id = np.nonzero(skeleton)
        id = np.concatenate((id[1][None,:],id[0][None,:]),axis=0)

        # Transfer from array index to map frame coords
        id_h = np.concatenate((id, np.ones((1, id.shape[1]))))
        tf = np.array([[map.info.resolution,0,map.info.origin.position.x], [0,map.info.resolution,map.info.origin.position.y], [0,0,1]])
        id_tf = np.matmul(tf, id_h)[0:2,:]

        id_tf = np.concatenate((id_tf, np.zeros((1,id_tf.shape[1]))))
        h = Header()
        h.stamp = rclpy.clock.Clock().now().to_msg()
        h.frame_id = "map"
        cloud = pc2.create_cloud_xyz32(h, id_tf.T.tolist())
        self.publisher1.publish(cloud)

        if SAVE_PATH:
            np.savetxt('track.txt', id_tf.T)

    # convert laser to point
    def scan_cb(self, msg):
        if self.last_pose is None:
            return
        t0 = time.time()
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        c = np.cos(angles)
        s = np.sin(angles)
        rot = np.array([[c,-s],[s,c]])  #2*2*N
        rot = np.swapaxes(np.swapaxes(rot,0,1),0,2)
        x_axis = np.repeat(np.array([[[1.],[0.]]]), len(ranges), axis=0)
        rot_t = torch.from_numpy(rot)
        x_axis_t = torch.from_numpy(x_axis)
        points_2d = ranges*(torch.matmul(rot_t, x_axis_t).numpy().reshape((len(ranges),2)).T)
        
        points = np.concatenate((points_2d, np.zeros_like(ranges)[None,:]))

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

    # get last car position
    def tracked_pose_cb(self, msg):
        self.last_pose = msg.pose


def main(args=None):
    rclpy.init(args=args)
    print("Creating the centerline ...")
    centerline = CenterLine()
    rclpy.spin(centerline)
    centerline.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    '''
    Create the centerline in map
    '''
    main()
