from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy

from util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.

        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        pass

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        pass

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).

        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.

        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        # remove the outliers, keep the densest particles

        # compare all the possible distances between particles and "set"
        # before there was a chance that a single particle is compared with itself

        distances = set()
        i = 0
        for p1 in self.particlecloud.poses:
            i = i + 1
            for p2 in self.particlecloud.poses[i:]:
                distance = numpy.sqrt(((p1.position.x - p2.position.x)**2) \
                                      + ((p1.position.y - p2.position.y)**2) \
                                      + ((p1.position.z - p2.position.z)**2))
                distances.add(distance)

        # sort the distances and keep the first third of them
        min_dist = sorted(distances)[:round(len(distances) / 3)]  # testing !!    !!!!!!!!!!!!!!!
        # calculate each particle's number of appearances in the min_dist
        counter = numpy.zeros(len(self.particlecloud))
        i = 0
        j = 0
        # how many particles is a specific particle close to?
        for p1 in self.particlecloud.poses:
            i = i + 1
            j = i - 1
            for p2 in self.particlecloud.poses[i:]:
                j = j + 1
                distance = numpy.sqrt(((p1.position.x - p2.position.x)**2) \
                                      + ((p1.position.y - p2.position.y)**2) \
                                      + ((p1.position.z - p2.position.z)**2))
                if distance in min_dist:
                    counter[i - 1] += 1
                    counter[j] += 1


        # sort counter and keep the particles corresponding to the last third
        sort_count = sorted(range(len(counter)), key=lambda k: counter[k])
        sort_count = sort_count[round(2 * len(sort_count) / 3):]
        wanted_array=[]
        for i in sort_count:
            wanted_array=self.particlecloud[i]

        # find the mean position
        x_values = y_values = z_values = 0
        for p in wanted_array.poses:
            x_values += p.position.x     # means -->  x_values = x_values + p.position.x
            y_values += p.position.y
            z_values += p.position.z


        meanX = x_values / len(self.wanted_array.poses)
        meanY = y_values / len(self.wanted_array.poses)
        meanZ = z_values / len(self.wanted_array.poses)
        est_pose.position = [meanX, meanY, meanZ]

        # find the mean orientation
        x_values = y_values = z_values = w_values = 0;
        for p in wanted_array.poses:
            x_values += p.orientation.x
            y_values += p.orientation.y
            z_values += p.orientation.z
        z_values = z_values + p.orientation.w
        meanX = x_values / len(self.wanted_array.poses)
        meanY = y_values / len(self.wanted_array.poses)
        meanZ = z_values / len(self.wanted_array.poses)
        meanW = w_values / len(self.wanted_array.poses)
        est_pose.orientation = [meanX, meanY, meanZ, meanW]

        return est_pose
