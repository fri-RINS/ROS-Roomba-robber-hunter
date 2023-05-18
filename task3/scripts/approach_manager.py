#!/usr/bin/python3

# vsebuje funkcijo, ki vrne koordinate ciljev, ki jih je potrebno doseci, da pregledamo celoten prostor
# se bo uporabil v brain.py na zacetku, da se pridobijo cilji

import rospy
import cv2
import numpy as np
import math

# for map loading
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Point, PointStamped, Vector3, Pose

import tf2_geometry_msgs
import tf2_ros

# for images additionally
from matplotlib import pyplot as plt

from tf import transformations as t

from map_msgs.msg import OccupancyGridUpdate

# weird ui
#import costmap_2d
from bresenham import bresenham

from tf.transformations import quaternion_from_euler

class ApproachManager:
    def __init__(self):
        # for storing a map
        self.map = None
        # for storing correct transformation
        self.map_transform = TransformStamped()
        self.map_resolution = None

        self.size_x = None
        self.size_y = None

        self.map_reference_frame = None
        #self.start_map_updates()

        # subscribe to costmap
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.map_callback)
        #rospy.sleep(2)

        # for updates
        # currently not working
        #rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, self.map_update_callback)

        # wait for map
        #while self.map_resolution == None:
        #while self.size_x == None:
        #    rospy.loginfo("Waiting for map in manager.")
        #    rospy.sleep(0.1)
        
        #rospy.Subscriber("/move_base/global_costmap/inflation_layer", OccupancyGrid, self.map_callback)


    def start_map_updates(self):
        """
        Starts updating the map. To be called after the goals were initialized.
        """
        rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, self.map_update_callback)

    def nearest_nonzero_to_point(self, a, x, y):
        """
        Return indices of nonzero element closest to point (x,y) in array a
        """
        r,c = np.nonzero(a)
        print("R:",r)
        print("C:",c)
        min_idx = ((r - y)**2 + (c - x)**2).argmin()
        return c[min_idx], r[min_idx]

    def get_nearest_accessible_point(self, x, y):
        """
        Return indices of accessible point closest to point (x,y) in costmap
        """
        # convert to map coords
        (c_x, c_y) = self.world_to_map_coords_simple(x, y)

        x_close, y_close = self.nearest_nonzero_to_point(self.accessible_costmap, c_x, c_y)

        (x_transformed, y_transformed) = self.map_to_world_coords(x_close, y_close)
        return x_transformed, y_transformed

    def map_update_callback(self, map_update):
        """
        Deals with map updates
        """
        #print("new map update")
        x = map_update.x
        y = map_update.y
        size_x = map_update.width
        size_y = map_update.height
        
        update_map = np.array(map_update.data).reshape((size_y, size_x))
        

        # deal with map
        update_map[update_map == -1] = 127
        update_map[update_map == 0] = 255
        update_map[update_map == 100] = 255
        
        self.map[y:(y+size_y), x:(x+size_x)] = update_map        

        # remember only accessible positions
        self.accessible_costmap = np.copy(self.map)
        

        # for old map
        # self.accessible_costmap[self.accessible_costmap != 255] = 0
        #threshold_available_map_point = 50
        threshold_available_map_point = 100
        self.accessible_costmap[self.accessible_costmap > threshold_available_map_point] = 0
        self.accessible_costmap[self.accessible_costmap > 0] = 255

        # erode accessible_costmap to make sure we get more central reachable points
        self.accessible_costmap = np.uint8(self.accessible_costmap)
        kernel = np.ones((5,5), np.uint8)
        #kernel = np.ones((5,5), np.uint8)
        self.accessible_costmap = cv2.erode(self.accessible_costmap, kernel)


        

    """
    Receives map from map_server and stores it in self.map
    """
    def map_callback(self, map_data):
        #rospy.loginfo(str(map_data.header))
        size_x = map_data.info.width
        size_y = map_data.info.height

        self.size_x = size_x
        self.size_y = size_y

        rospy.loginfo("Map size: x: %s, y: %s." % (str(size_x), str(size_y)))

        if size_x < 3 or size_y < 3:
            rospy.loginfo("Map size only: x: %s, y: %s. NOT running map to image conversion." % (str(size_x), str(size_y)))
            return
        
        self.map_resolution = map_data.info.resolution
        rospy.loginfo("Map resolution: %s" % str(self.map_resolution))

        self.map_reference_frame = map_data.header.frame_id

        self.map_transform.transform.translation.x = map_data.info.origin.position.x
        self.map_transform.transform.translation.y = map_data.info.origin.position.y
        self.map_transform.transform.translation.z = map_data.info.origin.position.z
        self.map_transform.transform.rotation = map_data.info.origin.orientation

        # deal with map
        #self.map = np.array(map_data.data, dtype = np.int8).reshape((size_y, size_x))
        self.map = np.array(map_data.data).reshape((size_y, size_x))

        # flip on the rows to get correct image (flip along y axis)
        # is it neccesary in our case - doe not seem to be because we will not transform y later at conversion
        # self.map = np.flip(self.map, 0)

        # get correct numbers
        self.map[self.map == 0] = 255
        self.map[self.map == -1] = 0
        self.map[self.map == 100] = 0

        # remember only accessible positions
        self.accessible_costmap = np.copy(self.map)
        self.accessible_costmap = np.uint8(self.accessible_costmap)

        # for old map
        # self.accessible_costmap[self.accessible_costmap != 255] = 0
        #threshold_available_map_point = 50
        # threshold_available_map_point = 128
        # self.accessible_costmap[self.accessible_costmap > threshold_available_map_point] = 255
        self.accessible_costmap[self.accessible_costmap < 255] = 0

        #erode accessible_costmap to make sure we get more central reachable points

        kernel = np.ones((1,1), np.uint8)
        #kernel = np.ones((5,5), np.uint8)
        self.accessible_costmap = cv2.erode(self.accessible_costmap, kernel)
        cv2.imshow("ImWindow", self.accessible_costmap)
        cv2.waitKey(0)



    """
    Returns euclidean distance between points.
    """
    def dist_euclidean(self, x1, y1, x2, y2):
        #rospy.loginfo("Distance called with: (%s, %s, %s, %s)" % (str(x1), str(y1), str(x2), str(y2)))
        return math.sqrt((x1 - x2) **2 + (y1 - y2)**2)

    """
    Retruns true, if x and y indices represent a cell in map array, false otherwise.
    """
    def in_map_bounds(self, x, y):
        if (x >= 0) and (y >= 0) and (x < self.size_x) and (y < self.size_y):
            return True
        else:
            return False

    """
    Returns inverse transform of self.map_transform
    """
    def get_inverse_transform(self):
        # https://answers.ros.org/question/229329/what-is-the-right-way-to-inverse-a-transform-in-python/
        # https://www.programcreek.com/python/example/96799/tf.transformations
        # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
        transform_tmp = t.concatenate_matrices(t.translation_matrix(np.array([self.map_transform.transform.translation.x, self.map_transform.transform.translation.y, self.map_transform.transform.translation.z])), t.quaternion_matrix(np.array([self.map_transform.transform.rotation.x, self.map_transform.transform.rotation.y, self.map_transform.transform.rotation.z])))
        inverse_transform = t.inverse_matrix(transform_tmp)
        translation = t.translation_from_matrix(inverse_transform)
        rotation = t.quaternion_from_matrix(inverse_transform)

        res = TransformStamped()
        res.transform.translation.x = translation[0]
        res.transform.translation.y = translation[1]
        res.transform.translation.z = translation[2]
        res.transform.rotation.x = rotation[0]
        res.transform.rotation.y = rotation[1]
        res.transform.rotation.z = rotation[2]
        res.transform.rotation.w = rotation[3]

        return res

    """
    Returns coordinates transformed from map to world.
    WARNING: do not use!
    """
    def map_to_world_coords_simple(self, x, y):
        w_x = x * self.map_resolution + self.map_transform.transform.translation.x
        w_y = y * self.map_resolution + self.map_transform.transform.translation.y
        return (w_x, w_y)

    """
    Returns coordinates transformed from map to world.
    """
    def map_to_world_coords(self, x, y):
        pt = PointStamped()
        pt.point.x = x * self.map_resolution
        pt.point.y = y * self.map_resolution
        pt.point.z = 0.0

        # transform to goal space
        transformed_pt = tf2_geometry_msgs.do_transform_point(pt, self.map_transform)

        return (transformed_pt.point.x, transformed_pt.point.y)

    """
    Returns tuple of coordinates transformed from world to map.
    WARNING: do not use!
    """
    def world_to_map_coords_simple(self, x, y):
        c_x = round((x - self.map_transform.transform.translation.x) / self.map_resolution)
        c_y = round((y - self.map_transform.transform.translation.y) / self.map_resolution)
        return (c_x, c_y)

    """
    Returns tuple of coordinates transformed from world to map.
    """
    def world_to_map_coords(self, x, y):
        inverse_transform = self.get_inverse_transform()

        rospy.loginfo("Inverse transform: %s" % str(inverse_transform))

        pt = PointStamped()
        pt.point.x = x
        pt.point.y = y
        pt.point.z = 0.0

        transformed_pt = tf2_geometry_msgs.do_transform_point(pt, inverse_transform)

        x_res = round(transformed_pt.point.x / self.map_resolution)
        y_res = round(transformed_pt.point.y / self.map_resolution)

        return (x_res, y_res)

    """
    Returns the map value at coordinates x, y in map.
    """
    def map_coord_cost(self, x, y):
        if not self.in_map_bounds(x, y):
            # wrong data
            rospy.logerr("Invalid map coordinates.")
            return None
        return self.map[y, x]
    
    """
    Returns the map value at coordinates x, y in world.
    """
    def world_coord_cost(self, x, y):
        (x_tmp, y_tmp) = self.world_to_map_coords(x, y)

        if not self.in_map_bounds(x_tmp, y_tmp):
            # wrong data
            rospy.logerr("Invalid map coordinates.")
            return None
        return self.map[y_tmp, x_tmp]

    """
    Returns True, if the robot can move to point x, y
    """
    def can_move_to(self, x, y):
        if self.map_coord_cost(x, y) == 127:
            # unknown
            return False
        elif self.map_coord_cost(x, y) == 99:
            # not far enough from obstacle
            return False
        elif self.map_coord_cost(x, y) == 0:
            # wall
            return False
        elif self.map_coord_cost(x, y) == 255:
            # empty space
            return True
        elif self.map_coord_cost(x, y) > 38:
            # wall
            return False

        return True

    def can_move_to_world_coord(self, x, y):
        """
        Returns true if we can move to point in world coords.
        """
        (x_m, y_m) = self.world_to_map_coords(x, y)
        return self.can_move_to(x_m, y_m)


    def get_move_to_map_array(self):
        """
        Returns a map where points that can be moved to are set to 255
        """

        costmap = np.copy(self.map)
        costmap[costmap != 255] = 0
        return costmap

    """
    Returns the coordinates of points which is closest to  point (x_ce, y_ce),
    is empty and far enough from the wall (read from cost_map).
    @return: (x_c, y_c)
    """
    def get_face_greet_location_candidates(self, x_ce, y_ce):
        candidates = []
        # d is parameter in what radius we search for closest points
        d = 10
        for r in range(y_ce - d, y_ce + d):
            for c in range(x_ce - d, x_ce + d):
                if self.in_map_bounds(c, r) and self.can_move_to(c, r):
                    candidates.append((c, r))

        return candidates

    """
    Returns the coordinates of points which is on perpendicular line to face plane and goes through (x_ce, y_ce),
    is empty and far enough from the wall (read from cost_map).
    @return: (x_c, y_c)
    """
    def get_face_greet_location_candidates_perpendicular_non_vector(self, x_ce, y_ce, fpose_left, fpose_right, d=15):
        x_left = fpose_left.position.x
        y_left = fpose_left.position.y
        x_right = fpose_right.position.x
        y_right = fpose_right.position.y

        # get the equation of perpendicular line
        k = (y_right - y_left) / (x_right - x_left)
        k_perp = -1.0 / k
        n = y_ce - k_perp * x_ce
        
        print("k: " + str(k) + " n: " + str(n) + " x_start: " + str(x_ce))
        # TODO: remove problems with lines perpendicular to x axis (problems because of numeric representation)
        # parameter
        #d = 7
        #d = 10

        x_start = x_ce - d
        y_start = round(k_perp * x_start + n)
        x_finish = x_ce + d
        y_finish = round(k_perp * x_finish + n)

        #candidates = list(bresenham(x_ce, y_ce, cnd_tmp[len(cnd_tmp)-1][0], cnd_tmp[len(cnd_tmp)-1][1]))
        candidates = list(bresenham(x_start, y_start, x_finish, y_finish))
        print("Candidates for:")
        print(candidates)

        #return candidates

        # go through candidates and check if they can be moved to
        candidates_reachable = []
        for candidate in candidates:
            c = candidate[0]
            r = candidate[1]
            if (self.in_map_bounds(c, r) and self.can_move_to(c, r)):
                candidates_reachable.append(candidate)

        print("Candidates reachable:")
        print(candidates_reachable)

        return candidates_reachable

    """
    Returns the coordinates of points which is on perpendicular line to face plane and goes through (x_ce, y_ce),
    is empty and far enough from the wall (read from cost_map).
    @return: (x_c, y_c)
    """
    #def get_face_greet_location_candidates_perpendicular(self, x_ce, y_ce, fpose_left, fpose_right, d=15):
    def get_face_greet_location_candidates_perpendicular(self, x_ce, y_ce, fpose_left, fpose_right, d=30):
        x_left = fpose_left.position.x
        y_left = fpose_left.position.y
        x_right = fpose_right.position.x
        y_right = fpose_right.position.y

        # with a vector to avoid problems with lines perpendicular to x axis

        # current vector
        dx = x_right - x_left
        dy = y_right - y_left

        # get normalized perpendicular vector
        perp_dx = -dy / ((dy*dy+dx*dx)**0.5)
        perp_dy = dx / ((dy*dy+dx*dx)**0.5)

        #x_start = round(- perp_dx * d + x_ce)
        #y_start = round(- perp_dy * d + y_ce)
        #x_finish = round(perp_dx * d + x_ce)
        #y_finish = round(perp_dy * d + y_ce)
        x_start = x_ce
        y_start = y_ce
        x_finish = round(- perp_dx * d + x_ce)
        y_finish = round(- perp_dy * d + y_ce)

        #candidates = list(bresenham(x_ce, y_ce, cnd_tmp[len(cnd_tmp)-1][0], cnd_tmp[len(cnd_tmp)-1][1]))
        candidates = list(bresenham(x_start, y_start, x_finish, y_finish))
        print("Candidates for:")
        print(candidates)

        #candidates.reverse()

        #return candidates

        # go through candidates and check if they can be moved to
        candidates_reachable = []
        for candidate in candidates:
            c = candidate[0]
            r = candidate[1]
            if (self.in_map_bounds(c, r) and self.can_move_to(c, r)):
                candidates_reachable.append(candidate)
                # if using central as start
                break

        print("Candidates reachable:")
        print(candidates_reachable)

        if len(candidates_reachable) == 0:
            # in case no candidates are on valid positions
            print("Searching for backup candidates")
            backup_candidate = candidates[3]
            x = backup_candidate[0]
            y = backup_candidate[1]

            x_close, y_close = self.nearest_nonzero_to_point(self.accessible_costmap, x, y)
            candidates_reachable.append((x_close, y_close))
            print(candidates_reachable)


        return candidates_reachable


    """
    Returns the coordinates of points which is closest to  point (x_ce, y_ce),
    is empty and far enough from the wall (read from cost_map)
    AND is closest to point (x_r, y_r) - robot at face detection
    (The idea being that this point will be on the correct side of the wall
    on which the face is mounted - closer to robot that was able to "see" it)
    @return: (x_point, y_point) in world coordinates used for setting goals.
    """
    def get_face_greet_location(self, x_c, y_c, x_r, y_r, fpose_left, fpose_right):

        # convert to map coordinates
        (x_c, y_c) = self.world_to_map_coords(x_c, y_c)
        (x_r, y_r) = self.world_to_map_coords(x_r, y_r)

        rospy.loginfo("Robot converted to map coordinates: (%s, %s)" % (str(x_r), str(y_r)))

        candidates = self.get_face_greet_location_candidates_perpendicular(x_c, y_c, fpose_left, fpose_right)


        min_dist = float('inf')
        res_point = None
        """
        for p in candidates:
            x = p[0]
            y = p[1]
            d = self.dist_euclidean(x, y, x_r, y_r)
            if d < min_dist:
                min_dist = d
                res_point = p
        """
        res_point = candidates[0]

        # convert to world coordinates and return res
        return self.map_to_world_coords(res_point[0], res_point[1])


    """
    Returns the coordinates of points which is closest to  point (x_ce, y_ce),
    is empty and far enough from the wall (read from cost_map)
    AND is closest to point (x_r, y_r) - robot at face detection
    (The idea being that this point will be on the correct side of the wall
    on which the face is mounted - closer to robot that was able to "see" it)
    @return: (x_point, y_point) in world coordinates used for setting goals.
    """
    def get_face_greet_location_old(self, x_c, y_c, x_r, y_r):

        # convert to map coordinates
        (x_c, y_c) = self.world_to_map_coords(x_c, y_c)
        (x_r, y_r) = self.world_to_map_coords(x_r, y_r)

        rospy.loginfo("Robot converted to map coordinates: (%s, %s)" % (str(x_r), str(y_r)))

        candidates = self.get_face_greet_location_candidates(x_c, y_c)


        min_dist = float('inf')
        res_point = None
        for p in candidates:
            x = p[0]
            y = p[1]
            d = self.dist_euclidean(x, y, x_r, y_r)
            if d < min_dist:
                min_dist = d
                res_point = p

        # convert to world coordinates and return res
        return self.map_to_world_coords(res_point[0], res_point[1])

    
    def quaternion_from_points(self, x1, y1, x2, y2):
        """
        Returns quaternion representing rotation so that the 
        robot will be pointing prom (x1,y1)to (x2,y2)
        """
        v1 = np.array([x2, y2, 0]) - np.array([x1, y1, 0])
        # in the direction of z axis
        v0 = [1, 0, 0]

        # compute yaw - rotation around z axis
        yaw = np.arctan2(v1[1], v1[0]) - np.arctan2(v0[1], v0[0])

        #rospy.loginfo("Yaw: %s" % str(yaw * 57.2957795))

        q = quaternion_from_euler(0, 0, yaw)

        #rospy.loginfo("Got quaternion: %s" % str(q))

        return q
    
    def get_object_greet_pose(self, x_obj, y_obj):
        """
        Returns pose with proper greet location and orientation 
        for ring / cylinder at x_obj, y_obj.
        """
        # compute position
        x_greet, y_greet = self.get_nearest_accessible_point(x_obj, y_obj)

        # compute orientation from greet point to object point
        q_dest = self.quaternion_from_points(x_greet, y_greet, x_obj, y_obj)

        # create pose for greet
        pose = Pose()
        pose.position.x = x_greet
        pose.position.y = y_greet
        pose.position.z = 0

        # so that there are no warnings
        pose.orientation.x = q_dest[0]
        pose.orientation.y = q_dest[1]
        pose.orientation.z = q_dest[2]
        pose.orientation.w = q_dest[3]

        return pose
    


def test():
    rospy.init_node('path_setter', anonymous=True)
    ps = ApproachManager()

    rospy.sleep(1) 
    """
    rospy.sleep(1)    
    x_in = 219
    y_in = 253
    plt.imshow(ps.map, interpolation='nearest')
    plt.plot(x_in, y_in, marker='o', markersize=3, color="red")
    plt.show()

    (x_out, y_out) = ps.map_to_world_coords(x_in, y_in)

    rospy.loginfo("Transformed point: x: %s, y: %s" % (str(x_out), str(y_out)))



    (x_orig, y_orig) = ps.world_to_map_coords(-0.55, -0.3666)

    plt.imshow(ps.accessible_costmap, interpolation='nearest')
    plt.plot(x_orig, y_orig, marker='o', markersize=3, color="red")
    plt.show()
    """
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():   
        #plt.imshow(ps.map, interpolation='nearest')  
        #plt.show()   
        rate.sleep()

    cv2.destroyAllWindows()
    

    #while not rospy.is_shutdown():
    #    rospy.sleep(0.1)

if __name__ == '__main__':
    test()
