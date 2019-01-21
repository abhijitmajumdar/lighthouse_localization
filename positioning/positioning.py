import numpy as np
import math
import scipy
import scipy.optimize
import cv2
import time
import itertools

from serial_data import Serial
from ekf import PositionEKF

'''
estimator:
    0   ->  PnP
    1   ->  Ray vector + optimizer
'''
class Position:
    def __init__(self,port,n_sensors,estimator=0,ekf=False,gauss_filter=1):
        self.port = Serial(port,swap_xy=False)
        self.ekf = PositionEKF() if ekf==True else None
        self.n_sensors = n_sensors
        # Assuming sensor 1 as the origin, sensor 0 towards y-axis and sensor 2 towards x-axis
        # If changed, pnp works, but might need to change the ray vector methods
        # self.sensor_positions = [[0,0.121,0],[0,0,0],[0.052,0,0],[0.052,0.121,0]]
        self.sensor_positions = [[0.05,0,0],[0,-0.05,0],[-0.05,0,0],[0,0.05,0]]
        self.estimate_pose = (self.estimate_pose_pnp,self.estimate_pose_rayvector)[estimator]
        self.lh_T = np.eye(4)
        self.object_T = np.eye(4)
        self.processed_T = np.eye(4)
        # For filtering outliers
        self.d_log = []
        self.d_log_n = gauss_filter
        self.old_p = None

    # PnP solution using OpenCV builtin methods
    def estimate_pose_pnp(self,sensor_angles,sensor_positions):
        # We assume the camera matrix to be identity
        camera_matrix = np.eye(3)
        # The real distances of the sensors in the object frame
        object_points = np.array(sensor_positions).reshape((self.n_sensors,-1,3))
        # The projection of the sensors on the virtual image plane is the tangent of angeles since fx=fy=1
        image_points = np.array([[math.tan(x),math.tan(y)] for x,y in sensor_angles]).reshape((self.n_sensors,-1,2))
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, None)
        if success==True:
            R, jacobian = cv2.Rodrigues(rvec)
            T = np.insert(np.insert(R,3,0,axis=1),3,0,axis=0)
            T[:3,3] = (tvec[0],tvec[1],tvec[2])
            T[3,3] = 1.0
            return T
        return None

    # Ray vector solution using optimization
    def F(self,x,*args):
        return sum([pow(pow(x[a],2) + pow(x[b],2) - 2*x[a]*x[b]*args[0][i] - pow(args[1][i],2),2) for i,(a,b) in enumerate(args[2])])

    def estimate_pose_rayvector(self,sensor_angles,sensor_positions):
        rays = []
        # Compute unit vectors in the respective directions
        for pt in sensor_angles:
            a = [math.cos(pt[0]), 0 , -math.sin(pt[0])]
            b = [0, math.cos(pt[1]), math.sin(pt[1])]
            ray = np.cross(b,a)
            # ray = np.cross(a,b)
            rays.append(ray/np.linalg.norm(ray))
        # Map indexes to pairs for consistancy across functions
        idx = list(itertools.combinations(range(self.n_sensors),2))
        # Compute angle between rays
        u = [math.cos(sensor_angles[a][0]-sensor_angles[b][0])*math.cos(sensor_angles[a][1]-sensor_angles[b][1]) for (a,b) in idx]
        # Compute distances between actual sensors
        r = [math.sqrt((sensor_positions[a][0]-sensor_positions[b][0])**2+(sensor_positions[a][1]-sensor_positions[b][1])**2+(sensor_positions[a][2]-sensor_positions[b][2])**2) for (a,b) in idx]
        # Initial approximation
        x0 = [1.0]*len(idx)
        sol = scipy.optimize.minimize(self.F, x0, args=(u,r,idx))
        if (sol.success==True):
            for i in range(self.n_sensors):
                rays[i] *= sol.x[i]
            p1,p2,p3 = np.array(rays[0]),np.array(rays[1]),np.array(rays[2])
            x = p3-p2
            v = p1-p2
            z = np.cross(x,v)
            x = x/float(np.linalg.norm(x))
            z = z/float(np.linalg.norm(z))
            y = np.cross(z,x)
            y = y/float(np.linalg.norm(y))
            # Compute rotation matrix from LH frame to body frame
            R = np.concatenate((x,y,z)).reshape((3,-1)).T
            # Transform matrix defining the rotation and translation
            T = np.insert(np.insert(R,3,0,axis=1),3,0,axis=0)
            T[:3,3] = rays[1]
            T[3,3] = 1.0
            return T
        return None

    def is_inlier(self,d_array, m=5):
        mean = np.mean(d_array[:-1])
        return True if abs(d_array[-1]-mean)<m else False

    def filter_pose(self,pose):
        x,y,z = pose[0,3],pose[1,3],pose[2,3]
        d = math.sqrt(x**2 + y**2 + z**2)
        self.d_log = (self.d_log + [d])[-self.d_log_n:] # last n logs
        if len(self.d_log)>2:
            if self.is_inlier(self.d_log)==False:
                print "Outlier detected!"
                self.d_log[-1] = self.d_log[-2]
                return self.old_p
        self.old_p = pose[:]
        return pose

    def process_pose(self):
        # Get raw data
        sample = self.port.read_data()
        # If invalid data
        if not len(sample)==self.n_sensors:
            return False
        T = self.estimate_pose(sample,self.sensor_positions)
        if T is None:
            return False
        T = self.filter_pose(T)
        self.processed_T = T.copy()
        return True

    def find_lighthouse(self):
        self.lh_T = self.processed_T.copy()

    def get_lighthouse(self):
        return self.lh_T.copy()

    def lock_lighthouse(self, iter=10):
        print "Initializing lighthouse position. Do not move the object!!!"
        time.sleep(2)
        lighthouse_pose_log = []
        n = iter
        while(n>0):
            if self.process_pose()==True:
                self.find_lighthouse()
                lighthouse_pose_log.append(self.get_lighthouse())
                n -= 1
                if n%(iter/10)==0: print n
                time.sleep(0.01)
        self.lh_T = np.mean(np.array(lighthouse_pose_log),axis=0)
        print "Initialization done! Computed location of lighthouse is:"
        print self.lh_T

    def find_object(self,disable_ekf=False):
        T = scipy.linalg.inv(self.processed_T)
        if (disable_ekf==False) and (self.ekf is not None):
            T[:3,3] = self.ekf.step(T[:3,3])
        self.object_T = T

    def get_object(self):
        return self.object_T.copy()

    def get_object_pose_world_frame(self):
        object_pose_lh_frame = self.get_object()
        lh_pose_world_frame = self.get_lighthouse()
        object_pose_world_frame = np.dot(scipy.linalg.inv(lh_pose_world_frame),scipy.linalg.inv(object_pose_lh_frame))
        return object_pose_world_frame

'''
References
- Pose from 3 points/vectors: https://www.mathworks.com/matlabcentral/answers/298940-how-to-calculate-roll-pitch-and-yaw-from-xyz-coordinates-of-3-planar-points
- General conversions of orientations: https://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/AN002.pdf?sfvrsn=19ee6b9_13
- Optimization coding(good!): http://hplgit.github.io/prog4comp/doc/pub/p4c-sphinx-Python/._pylight007.html
- Newton and Gauss-Newton:
    https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm
    https://en.wikipedia.org/wiki/Newton%27s_method#Nonlinear_systems_of_equations
- Tiny EKF: https://github.com/simondlevy/TinyEKF
- Lighthouse equations: https://trmm.net/Lighthouse
- Lighthouse OOTX decode:
    https://github.com/nairol/LighthouseRedox/blob/master/docs/Base%20Station.md#base-station-info-block
    https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md
- Rotation matrix to quaternion: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
- Summary of rotation and translation: http://www.math.tau.ac.il/~dcor/Graphics/cg-slides/geom3d.pdf
- Scipy opitmization bag: https://docs.scipy.org/doc/scipy/reference/optimize.html
- PnP:
    http://haralick-org.torahcode.us/journals/three_point_perspective.pdf
    https://github.com/imbinwang/posest/blob/master/p3p.c
    http://rpg.ifi.uzh.ch/docs/CVPR11_kneip.pdf
    https://en.wikipedia.org/wiki/Perspective-n-Point
    https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
    http://www.mmrc.iss.ac.cn/~xgao/paper/ieee.pdf
'''
