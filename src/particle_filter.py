#!/usr/bin/env python

# packages
import rospy
import numpy as np
import range_libc
import time
from threading import Lock
import tf.transformations
import tf
import utils as Utils
import utils_TA
from scipy.misc import imread

# messages
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap

# visualization packages
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

'''
These flags indicate several variants of the sensor model. Only one of them is used at a time.
'''
VAR_NO_EVAL_SENSOR_MODEL = 0
VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1
VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3


class ParticleFiler():
    '''
    This class implements Monte Carlo Localization based on odometry and a laser scanner.
    '''

    def __init__(self):
        # parameters
        self.ANGLE_STEP        = int(rospy.get_param("~angle_step"))
        self.MAX_PARTICLES     = int(rospy.get_param("~max_particles"))
        self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles"))
        self.INV_SQUASH_FACTOR = 1.0 / float(rospy.get_param("~squash_factor"))
        self.MAX_RANGE_METERS  = float(rospy.get_param("~max_range"))
        self.THETA_DISCRETIZATION = int(rospy.get_param("~theta_discretization"))
        self.WHICH_RM          = rospy.get_param("~range_method", "cddt").lower()
        self.RANGELIB_VAR      = int(rospy.get_param("~rangelib_variant", "3"))
        self.SHOW_FINE_TIMING  = bool(rospy.get_param("~fine_timing", "0"))
        self.PUBLISH_ODOM      = bool(rospy.get_param("~publish_odom", "1"))
        self.DO_VIZ            = bool(rospy.get_param("~viz"))

        # various data containers used in the MCL algorithm
        self.MAX_RANGE_PX = None
        self.odometry_data = np.array([0.0,0.0,0.0])
        self.laser = None
        self.iters = 0
        self.map_info = None
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.last_pose = None
        self.laser_angles = None
        self.downsampled_angles = None
        self.range_method = None
        self.last_time = None
        self.last_stamp = None
        self.first_sensor_update = True
        self.goal_set = False
        self.last_inferred_pose = None
        self.pose_settled = False
        self.init_pose = False
        self.state_lock = Lock()
        self.m_to_ft = 3.28084

        self.avoid_dist = float(rospy.get_param("avoid_dist"))

        # variables for checking if cone is in acceptable region
        self.map2worldScale  = 0.0504
        self.world2mapScale  = 1/float(self.map2worldScale)
        self.w2mRotation     = np.array([-1, 1]) * self.world2mapScale
        self.w2mTranslation  = np.array([25.9,16.5]) * self.world2mapScale
        cone_map = imread("/home/racecar/racecar-ws/src/race_0/maps/cone_map_flap.png")
        cone_map_dilated = imread("/home/racecar/racecar-ws/src/race_0/maps/cone_map_dilated.png")
        self.acceptable_cone_coordinates = load_map(cone_map_dilated)
        self.acceptable_waypoint_coordinates = load_map(cone_map)

        # cache this to avoid memory allocation in motion model
        self.local_deltas = np.zeros((self.MAX_PARTICLES, 3))

        # cache this for the sensor model computation
        self.queries = None
        self.ranges = None
        self.tiled_angles = None
        self.sensor_model_table = None

        # particle poses and weights
        self.inferred_pose = None
        self.particle_indices = np.arange(self.MAX_PARTICLES)
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        # initialize the state
        self.smoothing = Utils.CircularArray(10)
        self.timer = Utils.Timer(10)
        self.get_omap()
        self.precompute_sensor_model()
        self.initialize_global()

        # these topics are for visualization
        self.pose_pub      = rospy.Publisher("/pf/viz/inferred_pose", PoseStamped, queue_size = 1)
        self.particle_pub  = rospy.Publisher("/pf/viz/particles", PoseArray, queue_size = 1)
        self.pub_fake_scan = rospy.Publisher("/pf/viz/fake_scan", LaserScan, queue_size = 1)
        self.rect_pub      = rospy.Publisher("/pf/viz/poly1", PolygonStamped, queue_size = 1)
        self.viz_namespace = "/cone_detect"
        self.cone_pub      = rospy.Publisher(self.viz_namespace + "/found_cone", Marker, queue_size = 1)
        self.cone_waypoint_pub1 = rospy.Publisher(self.viz_namespace + "/filter_waypoint1", Marker, queue_size = 1) 
        self.cone_waypoint_pub2 = rospy.Publisher(self.viz_namespace + "/filter_waypoint2", Marker, queue_size = 1)
        # self.map_goal_pub  = rospy.Publisher("/pf/viz/goal_point",PoseStamped, queue_size=1)
        # topic for path planner
        self.map_pose_pub = rospy.Publisher("/infered_pose_map", PoseStamped, queue_size = 1)
        self.map_goal_pub = rospy.Publisher("/goal_point",PoseStamped, queue_size=1)
        # oublishing cones as points
        self.cone_location_pub     = rospy.Publisher("/cone_location", Float32MultiArray, queue_size = 1)

        if self.PUBLISH_ODOM:
            self.odom_pub      = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)

        # these topics are for coordinate space things
        self.pub_tf = tf.TransformBroadcaster()

        # these topics are to receive data from the racecar
        self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.lidarCB, queue_size=1)
        self.odom_sub  = rospy.Subscriber(rospy.get_param("~odometry_topic", "/odom"), Odometry, self.odomCB, queue_size=1)
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose, queue_size=1)
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=1)
        self.cone_sub = rospy.Subscriber("pursuit_control_input",Float32MultiArray, self.cone_viz2, queue_size=1)
        # self.goal_sub = rospy.Subscriber("/goal_point_click", PoseStamped, self.clicked_pose, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.clicked_pose, queue_size=1)
        print "In Lab 6"

        print "Finished initializing, waiting on messages..."

    def get_omap(self):
        '''
        Fetch the occupancy grid map from the map_server instance, and initialize the correct
        RangeLibc method. Also stores a matrix which indicates the permissible region of the map
        '''
        # this way you could give it a different map server as a parameter
        map_service_name = rospy.get_param("~static_map", "static_map")
        print("getting map from service: ", map_service_name)
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

        self.map_info = map_msg.info
        print self.map_info
        oMap = range_libc.PyOMap(map_msg)
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        # initialize range method
        print "Initializing range method:", self.WHICH_RM
        if self.WHICH_RM == "bl":
            self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
        elif "cddt" in self.WHICH_RM:
            self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            if self.WHICH_RM == "pcddt":
                print "Pruning..."
                self.range_method.prune()
        elif self.WHICH_RM == "rm":
            self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == "rmgpu":
            self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == "glt":
            self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
        print "Done loading map"

         # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255==0] = 1
        self.map_initialized = True

    def publish_tf(self,pose, stamp=None):
        """ Publish a tf for the car. This tells ROS where the car is with respect to the map. """
        if stamp == None:
            stamp = rospy.Time.now()

        # this may cause issues with the TF tree. If so, see the below code.
        self.pub_tf.sendTransform((pose[0],pose[1],0),tf.transformations.quaternion_from_euler(0, 0, pose[2]), 
               stamp , "/laser", "/map")

        # also publish odometry to facilitate getting the localization pose
        if self.PUBLISH_ODOM:
            odom = Odometry()
            odom.header = Utils.make_header("/map", stamp)
            odom.pose.pose.position.x = pose[0]
            odom.pose.pose.position.y = pose[1]
            odom.pose.pose.orientation = Utils.angle_to_quaternion(pose[2])
            self.odom_pub.publish(odom)
        
        return # below this line is disabled

        """
        Our particle filter provides estimates for the "laser" frame
        since that is where our laser range estimates are measure from. Thus,
        We want to publish a "map" -> "laser" transform.

        However, the car's position is measured with respect to the "base_link"
        frame (it is the root of the TF tree). Thus, we should actually define
        a "map" -> "base_link" transform as to not break the TF tree.
        """

        # Get map -> laser transform.
        map_laser_pos = np.array( (pose[0],pose[1],0) )
        map_laser_rotation = np.array( tf.transformations.quaternion_from_euler(0, 0, pose[2]) )
        # Apply laser -> base_link transform to map -> laser transform
        # This gives a map -> base_link transform
        laser_base_link_offset = (0.265, 0, 0)
        map_laser_pos -= np.dot(tf.transformations.quaternion_matrix(tf.transformations.unit_vector(map_laser_rotation))[:3,:3], laser_base_link_offset).T

        # Publish transform
        self.pub_tf.sendTransform(map_laser_pos, map_laser_rotation, stamp , "/base_link", "/map")

    def visualize(self):
        '''
        Publish various visualization messages.
        '''
        if not self.DO_VIZ:
            return

        if self.pose_pub.get_num_connections() > 0 and isinstance(self.inferred_pose, np.ndarray):
            ps = PoseStamped()
            ps.header = Utils.make_header("map")
            ps.pose.position.x = self.inferred_pose[0]
            ps.pose.position.y = self.inferred_pose[1]
            ps.pose.orientation = Utils.angle_to_quaternion(self.inferred_pose[2])
            self.pose_pub.publish(ps)

        if self.particle_pub.get_num_connections() > 0:
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                # randomly downsample particles
                proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
                # proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES)
                self.publish_particles(self.particles[proposal_indices,:])
            else:
                self.publish_particles(self.particles)

        if self.pub_fake_scan.get_num_connections() > 0 and isinstance(self.ranges, np.ndarray):
            # generate the scan from the point of view of the inferred position for visualization
            self.viz_queries[:,0] = self.inferred_pose[0]
            self.viz_queries[:,1] = self.inferred_pose[1]
            self.viz_queries[:,2] = self.downsampled_angles + self.inferred_pose[2]
            self.range_method.calc_range_many(self.viz_queries, self.viz_ranges)
            self.publish_scan(self.downsampled_angles, self.viz_ranges)

    def cone_within_bounds(self, world_loc):
        map_loc = self.w2mRotation * world_loc + self.w2mTranslation # Convert to map coordinates
        map_loc = tuple(np.floor(map_loc))  # Floor and convert to tuple for hasing
        return map_loc in self.acceptable_cone_coordinates

    def waypoint_within_bounds(self, world_loc):
        map_loc = self.w2mRotation * world_loc + self.w2mTranslation # Convert to map coordinates
        map_loc = tuple(np.floor(map_loc))  # Floor and convert to tuple for hasing
        return map_loc in self.acceptable_waypoint_coordinates

    def cone_viz(self, msg):
        try:
            cone_type = msg.data[2]
            cone_pos_x = msg.data[0]
            cone_pos_y = msg.data[1]
            cone_theta = np.pi/2.0 - np.arctan2(cone_pos_y, cone_pos_x)

            self.state_lock.acquire()
            car_pos_x = self.inferred_pose[0]
            car_pos_y = self.inferred_pose[1]
            car_pos_t = self.inferred_pose[2]
            self.state_lock.release()
            cone_waypoint_x = cone_pos_x + self.avoid_dist+(-1 + 1*cone_type)
            cone_waypoint_theta = np.pi/2.0 - np.arctan2(cone_pos_y, cone_waypoint_x)
            
            relative_theta = car_pos_t - cone_theta
            relative_theta_way = car_pos_t - cone_waypoint_theta
            cone_rad = np.sqrt(cone_pos_x**2 + cone_pos_y**2)
            cone_way_rad = np.sqrt(cone_waypoint_x**2 + cone_pos_y**2)

            # cone_pos_map_x = car_pos_x + cone_pos_x
            # cone_pos_map_y = car_pos_y + cone_pos_y
            cone_pos_map_x = car_pos_x + np.cos(relative_theta)*cone_rad
            cone_pos_map_y = car_pos_y + np.sin(relative_theta)*cone_rad
            cone_pos_map_way_x = car_pos_x + np.cos(relative_theta_way)*cone_way_rad
            cone_pos_map_way_y = car_pos_y + np.sin(relative_theta_way)*cone_way_rad

            # print cone_pos_x*self.m_to_ft,cone_pos_y*self.m_to_ft,cone_type


            if cone_type == 0:
                x_avoid_shift = -1.0
                self.cone_point = [cone_pos_map_x, cone_pos_map_y]
                color = [1.0, 0, 0]
            elif cone_type == 1:
                x_avoid_shift = 1.0
                self.cone_point = [cone_pos_map_x, cone_pos_map_y]
                color = [0, 1.0, 0]

            self.cone_waypoint_pub.publish(utils_TA.make_circle_marker(np.array([cone_pos_map_way_x, cone_pos_map_way_y]), 0.5, [1.0,1.0,1.0], "/map", self.viz_namespace, 0, 5))
            self.cone_pub.publish(utils_TA.make_circle_marker(self.cone_point, 0.5, color, "/map", self.viz_namespace, 0, 5))
            if self.cone_within_bounds(np.array([cone_pos_map_x, cone_pos_map_y])) and self.waypoint_within_bounds(np.array([cone_pos_map_way_x, cone_pos_map_way_y])) : # Cone is in acceptable location
                self.cone_pub.publish(utils_TA.make_circle_marker(self.cone_point, 0.5, color, "/map", self.viz_namespace, 0, 5))
                cone_loc = Float32MultiArray()
                cone_loc.data = [cone_pos_map_x, cone_pos_map_y, cone_type]
                self.cone_location_pub.publish(cone_loc)
            else: # Cone is out of bounds
                print "Cone out of bounds"
                pass
        except:
            print "exception raised cone viz"


    def cone_viz2(self, msg):
        # try:
        cone_type = msg.data[2]
        cone_pos_x = msg.data[0]
        cone_pos_y = msg.data[1]
        cone_theta = np.pi/2.0 - np.arctan2(cone_pos_y, cone_pos_x)

        self.state_lock.acquire()
        car_pos_x = self.inferred_pose[0]
        car_pos_y = self.inferred_pose[1]
        car_pos_t = self.inferred_pose[2]
        self.state_lock.release()
        cone_waypoint_x1 = cone_pos_x + self.avoid_dist+(-1 + 1*cone_type)
        cone_waypoint_y1 = cone_pos_y - self.avoid_dist
        cone_waypoint_x2 = cone_pos_x + self.avoid_dist+(-1 + 1*cone_type)
        cone_waypoint_y2 = cone_pos_y + self.avoid_dist
        cone_waypoint_theta1 = np.pi/2.0 - np.arctan2(cone_waypoint_y1, cone_waypoint_x1)
        cone_waypoint_theta2 = np.pi/2.0 - np.arctan2(cone_waypoint_y2, cone_waypoint_x2)
        
        relative_theta = car_pos_t - cone_theta
        relative_theta_way1 = car_pos_t - cone_waypoint_theta1
        relative_theta_way2 = car_pos_t - cone_waypoint_theta2
        cone_rad = np.sqrt(cone_pos_x**2 + cone_pos_y**2)
        cone_way_rad1 = np.sqrt(cone_waypoint_x1**2 + cone_waypoint_y1**2)
        cone_way_rad2 = np.sqrt(cone_waypoint_x2**2 + cone_waypoint_y2**2)

        # cone_pos_map_x = car_pos_x + cone_pos_x
        # cone_pos_map_y = car_pos_y + cone_pos_y
        cone_pos_map_x = car_pos_x + np.cos(relative_theta)*cone_rad
        cone_pos_map_y = car_pos_y + np.sin(relative_theta)*cone_rad
        cone_pos_map_way_x1 = car_pos_x + np.cos(relative_theta_way1)*cone_way_rad1
        cone_pos_map_way_y1 = car_pos_y + np.sin(relative_theta_way1)*cone_way_rad1
        cone_pos_map_way_x2 = car_pos_x + np.cos(relative_theta_way2)*cone_way_rad2
        cone_pos_map_way_y2 = car_pos_y + np.sin(relative_theta_way2)*cone_way_rad2

        # print cone_pos_x*self.m_to_ft,cone_pos_y*self.m_to_ft,cone_type


        if cone_type == 0:
            x_avoid_shift = -1.0
            self.cone_point = [cone_pos_map_x, cone_pos_map_y]
            color = [1.0, 0, 0]
        elif cone_type == 1:
            x_avoid_shift = 1.0
            self.cone_point = [cone_pos_map_x, cone_pos_map_y]
            color = [0, 1.0, 0]

        self.cone_waypoint_pub1.publish(utils_TA.make_circle_marker(np.array([cone_pos_map_way_x1, cone_pos_map_way_y1]), 0.5, [1.0,1.0,1.0], "/map", self.viz_namespace, 0, 5))
        self.cone_waypoint_pub2.publish(utils_TA.make_circle_marker(np.array([cone_pos_map_way_x2, cone_pos_map_way_y2]), 0.5, [1.0,1.0,1.0], "/map", self.viz_namespace, 0, 5))
        self.cone_pub.publish(utils_TA.make_circle_marker(self.cone_point, 0.5, color, "/map", self.viz_namespace, 0, 5))
        if self.cone_within_bounds(np.array([cone_pos_map_x, cone_pos_map_y])) and self.waypoint_within_bounds(np.array([cone_pos_map_way_x1, cone_pos_map_way_y1])) \
            and self.waypoint_within_bounds(np.array([cone_pos_map_way_x2, cone_pos_map_way_y2])): # Cone is in acceptable location
            # self.cone_pub.publish(utils_TA.make_circle_marker(self.cone_point, 0.5, color, "/map", self.viz_namespace, 0, 5))
            cone_loc = Float32MultiArray()
            cone_loc.data = [cone_pos_map_way_x1, cone_pos_map_way_y1, cone_pos_map_way_x2, cone_pos_map_way_y2]
            self.cone_location_pub.publish(cone_loc)
        else: # Cone is out of bounds
            print "Cone out of bounds"
            pass
        # except:
            # print "exception raised cone viz"


    def publish_particles(self, particles):
        pa = PoseArray()
        pa.header = Utils.make_header("map")
        pa.poses = Utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)

    def publish_scan(self, angles, ranges):
        ls = LaserScan()
        ls.header = Utils.make_header("laser", stamp=self.last_stamp)
        ls.angle_min = np.min(angles)
        ls.angle_max = np.max(angles)
        ls.angle_increment = np.abs(angles[0] - angles[1])
        ls.range_min = 0
        ls.range_max = np.max(ranges)
        ls.ranges = ranges
        self.pub_fake_scan.publish(ls)

    def lidarCB(self, msg):
        '''
        Initializes reused buffers, and stores the relevant laser scanner data for later use.
        '''
        if not isinstance(self.laser_angles, np.ndarray):
            print "...Received first LiDAR message"
            self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.downsampled_angles = np.copy(self.laser_angles[0::self.ANGLE_STEP]).astype(np.float32)
            self.viz_queries = np.zeros((self.downsampled_angles.shape[0],3), dtype=np.float32)
            self.viz_ranges = np.zeros(self.downsampled_angles.shape[0], dtype=np.float32)
            print self.downsampled_angles.shape[0]

        self.downsampled_ranges = np.array(msg.ranges[::self.ANGLE_STEP])
        self.lidar_initialized = True
        # self.update()

    def odomCB(self, msg):
        '''
        Store deltas between consecutive odometry messages in the coordinate space of the car.

        Odometry data is accumulated via dead reckoning, so it is very inaccurate on its own.
        '''
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y])

        orientation = Utils.quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])

        if isinstance(self.last_pose, np.ndarray):
            rot = Utils.rotation_matrix(-self.last_pose[2])
            delta = np.array([position - self.last_pose[0:2]]).transpose()
            local_delta = (rot*delta).transpose()

            # changes in x,y,theta in local coordinate system of the car
            self.odometry_data = np.array([local_delta[0,0], local_delta[0,1], orientation - self.last_pose[2]])
            self.last_pose = pose
            self.last_stamp = msg.header.stamp
            self.odom_initialized = True
        else:
            print "...Received first Odometry message"
            self.last_pose = pose

        # this topic is slower than lidar, so update every time we receive a message
        self.update()

    def clicked_pose(self, msg):
        '''
        Receive pose messages from RViz and initialize the particle distribution in response.
        '''
        print msg
        if isinstance(msg, PointStamped):
            self.initialize_global()
        elif isinstance(msg, PoseWithCovarianceStamped):
            self.initialize_particles_pose(msg.pose.pose)
        elif isinstance(msg, PoseStamped):
            self.goal_point_pub(msg.pose)

    def goal_point_pub(self, pose):
        # Publishing goal_point in map/pixel coordinatess
        # if self.map_goal_pub.get_num_connections() > 0 and not self.goal_set:
        if self.map_goal_pub.get_num_connections() > 0:
            map_goal_point = Utils.world_to_map_slow(pose.position.x,pose.position.y,Utils.quaternion_to_angle(pose.orientation),self.map_info)
            ps = PoseStamped()
            ps.header = Utils.make_header("map")
            ps.pose.position.x = map_goal_point[0]
            ps.pose.position.y = self.map_info.height - map_goal_point[1]
            ps.pose.orientation = Utils.angle_to_quaternion(map_goal_point[2])

            # ps1 = PoseStamped()
            # ps1.header = Utils.make_header("map")
            # ps1.pose.position.x = pose.position.x
            # ps1.pose.position.y = pose.position.y
            # ps1.pose.orientation = pose.orientation
            self.map_goal_pub.publish(ps)
            # self.goal_set = True
            print "goal point printed", map_goal_point

    def initialize_particles_pose(self, pose):
        '''
        Initialize particles in the general region of the provided pose.
        '''
        print "SETTING POSE"
        print pose
        self.state_lock.acquire()
        self.init_pose = True
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        self.particles[:,0] = pose.position.x + np.random.normal(loc=0.0,scale=0.5,size=self.MAX_PARTICLES)
        self.particles[:,1] = pose.position.y + np.random.normal(loc=0.0,scale=0.5,size=self.MAX_PARTICLES)
        self.particles[:,2] = Utils.quaternion_to_angle(pose.orientation) + np.random.normal(loc=0.0,scale=0.4,size=self.MAX_PARTICLES)
        self.state_lock.release()

    def initialize_global(self):
        '''
        Spread the particle distribution over the permissible region of the state space.
        '''
        print "GLOBAL INITIALIZATION"
        # randomize over grid coordinate space
        self.state_lock.acquire()
        permissible_x, permissible_y = np.where(self.permissible_region == 1)
        indices = np.random.randint(0, len(permissible_x), size=self.MAX_PARTICLES)

        permissible_states = np.zeros((self.MAX_PARTICLES,3))
        permissible_states[:,0] = permissible_y[indices]
        permissible_states[:,1] = permissible_x[indices]
        permissible_states[:,2] = np.random.random(self.MAX_PARTICLES) * np.pi * 2.0

        Utils.map_to_world(permissible_states, self.map_info)
        self.particles = permissible_states
        self.weights[:] = 1.0 / self.MAX_PARTICLES
        self.state_lock.release()

    def precompute_sensor_model(self):
        '''
        Generate and store a table which represents the sensor model. For each discrete computed
        range value, this provides the probability of measuring any (discrete) range.

        This table is indexed by the sensor model at runtime by discretizing the measurements
        and computed ranges from RangeLibc.
        '''
        print "Precomputing sensor model"
        # sensor model constants
        z_short = 0.01
        z_max = 0.07
        z_rand = 0.12
        sigma_hit = 8.0
        z_hit = 0.75
        c_r = 0.01

        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width,table_width))

        t = time.time()
        # d is the computed range from RangeLibc
        for d in xrange(table_width):
            norm = 0.0
            sum_unkown = 0.0
            # r is the observed range from the lidar unit
            for r in xrange(table_width):
                prob = 0.0
                z = float(r-d)
                # reflects from the intended object
                prob += z_hit * np.exp(-(z*z)/(2.0*sigma_hit*sigma_hit)) / (sigma_hit * np.sqrt(2.0*np.pi))

                # observed range is less than the predicted range - short reading
                if r < d:
                    prob += 2.0 * z_short * (d - r) / float(d)

                # erroneous max range measurement
                if int(r) == int(self.MAX_RANGE_PX):
                    prob += z_max

                # random measurement
                if r < int(self.MAX_RANGE_PX):
                    prob += z_rand * 1.0/float(self.MAX_RANGE_PX)

                norm += prob
                self.sensor_model_table[int(r),int(d)] = prob

            # normalize
            self.sensor_model_table[:,int(d)] /= norm

        # upload the sensor model to RangeLib for ultra fast resolution
        if self.RANGELIB_VAR > 0:
            self.range_method.set_sensor_model(self.sensor_model_table)

        # code to generate various visualizations of the sensor model
        if False:
            # visualize the sensor model
            fig = plt.figure()
            ax = fig.gca(projection='3d')

            # Make data.
            X = np.arange(0, table_width, 1.0)
            Y = np.arange(0, table_width, 1.0)
            X, Y = np.meshgrid(X, Y)

            # Plot the surface.
            surf = ax.plot_surface(X, Y, self.sensor_model_table, cmap="bone", rstride=2, cstride=2,
                                   linewidth=0, antialiased=True)

            ax.text2D(0.05, 0.95, "Precomputed Sensor Model", transform=ax.transAxes)
            ax.set_xlabel('Ground truth distance (in px)')
            ax.set_ylabel('Measured Distance (in px)')
            ax.set_zlabel('P(Measured Distance | Ground Truth)')

            plt.show()
        elif False:
            plt.imshow(self.sensor_model_table * 255, cmap="gray")
            plt.show()
        elif False:
            plt.plot(self.sensor_model_table[:,140])
            plt.plot([139,139],[0.0,0.08], label="test")
            plt.ylim(0.0, 0.08)
            plt.xlabel("Measured Distance (in px)")
            plt.ylabel("P(Measured Distance | Ground Truth Distance = 140px)")
            plt.show()

    def motion_model(self, proposal_dist, action):
        '''
        The motion model applies the odometry to the particle distribution. Since there the odometry
        data is inaccurate, the motion model mixes in gaussian noise to spread out the distribution.

        Vectorized motion model. Computing the motion model over all particles is thousands of times
        faster than doing it for each particle individually due to vectorization and reduction in
        function call overhead
        
        TODO this could be better, but it works for now
            - fixed random noise is not very realistic
            - ackermann model provides bad estimates at high speed
        '''
        # rotate the action into the coordinate space of each particle
        # t1 = time.time()
        cosines = np.cos(proposal_dist[:,2])
        sines = np.sin(proposal_dist[:,2])

        self.local_deltas[:,0] = cosines*action[0] - sines*action[1]
        self.local_deltas[:,1] = sines*action[0] + cosines*action[1]
        self.local_deltas[:,2] = action[2]

        proposal_dist[:,:] += self.local_deltas
        add_rand = 0.05
        proposal_dist[:,0] += np.random.normal(loc=0.0,scale=add_rand,size=self.MAX_PARTICLES)
        proposal_dist[:,1] += np.random.normal(loc=0.0,scale=add_rand*0.5,size=self.MAX_PARTICLES)
        proposal_dist[:,2] += np.random.normal(loc=0.0,scale=0.25,size=self.MAX_PARTICLES)

    def sensor_model(self, proposal_dist, obs, weights):
        '''
        This function computes a probablistic weight for each particle in the proposal distribution.
        These weights represent how probable each proposed (x,y,theta) pose is given the measured
        ranges from the lidar scanner.

        There are 4 different variants using various features of RangeLibc for demonstration purposes.
        - VAR_REPEAT_ANGLES_EVAL_SENSOR is the most stable, and is very fast.
        - VAR_NO_EVAL_SENSOR_MODEL directly indexes the precomputed sensor model. This is slow
                                   but it demonstrates what self.range_method.eval_sensor_model does
        '''
        
        num_rays = self.downsampled_angles.shape[0]
        # only allocate buffers once to avoid slowness
        if self.first_sensor_update:
            if self.RANGELIB_VAR <= 1:
                self.queries = np.zeros((num_rays*self.MAX_PARTICLES,3), dtype=np.float32)
            else:
                self.queries = np.zeros((self.MAX_PARTICLES,3), dtype=np.float32)

            self.ranges = np.zeros(num_rays*self.MAX_PARTICLES, dtype=np.float32)
            self.tiled_angles = np.tile(self.downsampled_angles, self.MAX_PARTICLES)
            self.first_sensor_update = False

        if self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.queries[:,:] = proposal_dist[:,:]
            self.range_method.calc_range_repeat_angles_eval_sensor_model(self.queries, self.downsampled_angles, obs, self.weights)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR:
            if self.SHOW_FINE_TIMING:
                t_start = time.time()
            # this version demonstrates what this would look like with coordinate space conversion pushed to rangelib
            self.queries[:,:] = proposal_dist[:,:]
            if self.SHOW_FINE_TIMING:
                t_init = time.time()
            self.range_method.calc_range_repeat_angles(self.queries, self.downsampled_angles, self.ranges)
            if self.SHOW_FINE_TIMING:
                t_range = time.time()
            # evaluate the sensor model on the GPU
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            if self.SHOW_FINE_TIMING:
                t_eval = time.time()
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
            if self.SHOW_FINE_TIMING:
                t_squash = time.time()
                t_total = (t_squash - t_start) / 100.0

            if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
                print "sensor_model: init: ", np.round((t_init-t_start)/t_total, 2), "range:", np.round((t_range-t_init)/t_total, 2), \
                      "eval:", np.round((t_eval-t_range)/t_total, 2), "squash:", np.round((t_squash-t_eval)/t_total, 2)
        elif self.RANGELIB_VAR == VAR_CALC_RANGE_MANY_EVAL_SENSOR:
            # this version demonstrates what this would look like with coordinate space conversion pushed to rangelib
            # this part is inefficient since it requires a lot of effort to construct this redundant array
            self.queries[:,0] = np.repeat(proposal_dist[:,0], num_rays)
            self.queries[:,1] = np.repeat(proposal_dist[:,1], num_rays)
            self.queries[:,2] = np.repeat(proposal_dist[:,2], num_rays)
            self.queries[:,2] += self.tiled_angles

            self.range_method.calc_range_many(self.queries, self.ranges)

            # evaluate the sensor model on the GPU
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_NO_EVAL_SENSOR_MODEL:
            # this version directly uses the sensor model in Python, at a significant computational cost
            self.queries[:,0] = np.repeat(proposal_dist[:,0], num_rays)
            self.queries[:,1] = np.repeat(proposal_dist[:,1], num_rays)
            self.queries[:,2] = np.repeat(proposal_dist[:,2], num_rays)
            self.queries[:,2] += self.tiled_angles

            # compute the ranges for all the particles in a single functon call
            self.range_method.calc_range_many(self.queries, self.ranges)

            # resolve the sensor model by discretizing and indexing into the precomputed table
            obs /= float(self.map_info.resolution)
            ranges = self.ranges / float(self.map_info.resolution)
            obs[obs > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
            ranges[ranges > self.MAX_RANGE_PX] = self.MAX_RANGE_PX

            intobs = np.rint(obs).astype(np.uint16)
            intrng = np.rint(ranges).astype(np.uint16)

            # compute the weight for each particle
            for i in xrange(self.MAX_PARTICLES):
                weight = np.product(self.sensor_model_table[intobs,intrng[i*num_rays:(i+1)*num_rays]])
                weight = np.power(weight, self.INV_SQUASH_FACTOR)
                weights[i] = weight
        else:
            print "PLEASE SET rangelib_variant PARAM to 0-3"

    def MCL(self, a, o):
        '''
        Performs one step of Monte Carlo Localization.
            1. resample particle distribution to form the proposal distribution
            2. apply the motion model
            3. apply the sensor model
            4. normalize particle weights

        This is in the critical path of code execution, so it is optimized for speed.
        '''
        if self.SHOW_FINE_TIMING:
            t = time.time()
        # draw the proposal distribution from the old particles
        proposal_indices = np.random.choice(self.particle_indices, self.MAX_PARTICLES, p=self.weights)
        proposal_distribution = self.particles[proposal_indices,:]
        if self.SHOW_FINE_TIMING:
            t_propose = time.time()

        # compute the motion model to update the proposal distribution
        self.motion_model(proposal_distribution, a)
        if self.SHOW_FINE_TIMING:
            t_motion = time.time()

        # compute the sensor model
        self.sensor_model(proposal_distribution, o, self.weights)
        if self.SHOW_FINE_TIMING:
            t_sensor = time.time()

        # normalize importance weights
        self.weights /= np.sum(self.weights)
        if self.SHOW_FINE_TIMING:
            t_norm = time.time()
            t_total = (t_norm - t)/100.0

        if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
            print "MCL: propose: ", np.round((t_propose-t)/t_total, 2), "motion:", np.round((t_motion-t_propose)/t_total, 2), \
                  "sensor:", np.round((t_sensor-t_motion)/t_total, 2), "norm:", np.round((t_norm-t_sensor)/t_total, 2)

        # save the particles
        self.particles = proposal_distribution
    
    def expected_pose(self):
        # returns the expected value of the pose given the particle distribution
        return np.dot(self.particles.transpose(), self.weights)

    def inferred_pose_map(self):
        # Publishing inferred_pose in map/pixel coordinatess
        
        if self.map_pose_pub.get_num_connections() > 0 and isinstance(self.inferred_pose, np.ndarray):
            map_inferred_pose = Utils.world_to_map_slow(self.inferred_pose[0],self.inferred_pose[1],self.inferred_pose[2],self.map_info)
            ps = PoseStamped()
            ps.header = Utils.make_header("map")
            ps.pose.position.x = map_inferred_pose[0]
            ps.pose.position.y = self.map_info.height  - map_inferred_pose[1]
            ps.pose.orientation = Utils.angle_to_quaternion(map_inferred_pose[2])
            self.map_pose_pub.publish(ps)

    def start_point_flag(self):
        if self.last_inferred_pose is not None:
            pose_diff = np.subtract(self.inferred_pose, self.last_inferred_pose)
            if sum(pose_diff) <= 0.25:
                self.pose_settled = True
        self.last_inferred_pose = np.array(self.inferred_pose)


    def update(self):
        '''
        Apply the MCL function to update particle filter state. 

        Ensures the state is correctly initialized, and acquires the state lock before proceeding.
        '''
        if self.lidar_initialized and self.odom_initialized and self.map_initialized:
            if self.state_lock.locked():
                print "Concurrency error avoided"
            else:
                self.state_lock.acquire()
                self.timer.tick()
                self.iters += 1

                t1 = time.time()
                observation = np.copy(self.downsampled_ranges).astype(np.float32)
                action = np.copy(self.odometry_data)
                self.odometry_data = np.zeros(3)

                # run the MCL update algorithm
                self.MCL(action, observation)

                # compute the expected value of the robot pose
                self.inferred_pose = self.expected_pose()

                if not self.pose_settled and self.init_pose:
                    # Sets self.pose_settled if pose has settled after initializing localization
                    self.start_point_flag()
                elif self.pose_settled:
                    # Publishes inferred_pose in map coordinates once initial inferred pose has settled
                    self.inferred_pose_map()

                self.state_lock.release()
                t2 = time.time()

                # publish transformation frame based on inferred pose
                self.publish_tf(self.inferred_pose, self.last_stamp)

                # this is for tracking particle filter speed
                ips = 1.0 / (t2 - t1)
                self.smoothing.append(ips)
                if self.iters % 10 == 0:
                    pass
                    # print "iters per sec:", int(self.timer.fps()), " possible:", int(self.smoothing.mean())

                self.visualize()

import argparse
import sys
parser = argparse.ArgumentParser(description='Particle filter.')
parser.add_argument('--config', help='Path to yaml file containing config parameters. Helpful for calling node directly with Python for profiling.')

def load_map(map_img):
    cells = set()
    for i in range(len(map_img[0])):
        for j in range(len(map_img)):
          if np.all(map_img[i][j] == 255):
            cells.add((j,i))
    return cells

def load_params_from_yaml(fp):
    from yaml import load
    with open(fp, 'r') as infile:
        yaml_data = load(infile)
        for param in yaml_data:
            print "param:", param, ":", yaml_data[param]
            rospy.set_param("~"+param, yaml_data[param])

# this function can be used to generate flame graphs easily
def make_flamegraph(filterx=None):
    import flamegraph, os
    perf_log_path = os.path.join(os.path.dirname(__file__), "../tmp/perf.log")
    flamegraph.start_profile_thread(fd=open(perf_log_path, "w"),
                                    filter=filterx,
                                    interval=0.001)

if __name__=="__main__":
    rospy.init_node("particle_filter")

    args,_ = parser.parse_known_args()
    if args.config:
        load_params_from_yaml(args.config)

    # make_flamegraph(r"update")

    pf = ParticleFiler()
    rospy.spin()
