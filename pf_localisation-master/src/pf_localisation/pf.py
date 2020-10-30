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
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
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
	#remove the outliers, keep the densest particles
	distances = set();
	for i = 1:len(self.particlecloud.poses):
	    p1 = self.particlecloud.poses[i];
  	    for j = i+1:len(self.particlecloud.poses):
		p2 = self.particlecloud.poses[j];
		distances.add(abs(p1.position-p2.position));

	#sort the distances and keep the first third of them	
	min_dist = sorted(distances)[:math.ceil(len(distances)/3)-1];
	#calculate each particle's number of appearances in the min_dist
	counter = numpy.zeros(len(self.particlecloud));	
        for i = 1:len(self.particlecloud.poses):
	    p1 = self.particlecloud.poses[i];
  	    for j = i+1:len(self.particlecloud.poses):
		p2 = self.particlecloud.poses[j];
		if abs(p1.position-p2.position) in min_dist:
		   counter[i-1] += 1;
		   counter[j-1] += 1;

	
	#sort counter and keep the particles corresponding to the last third
	sort_count = sorted(range(len(counter)), key=lambda k: counter[k]);
	sort_count = sort_count[math.ceil(2*len(sort_count)/3)-1:];
	wanted_set = set();
	for i in sort_count:
	    wanted_set.add(self.particlecloud[i]);
		

	#find the mean position	
	x_values = y_values = z_values = 0;
	for p in wanted_set.poses:
	    x_values = x_values + p.position.x;
            y_values = y_values + p.position.y;
            z_values = z_values + p.position.z;
	meanX = x_values / len(self.wanted_set.poses);
	meanY = y_values / len(self.wanted_set.poses);
	meanZ = z_values / len(self.wanted_set.poses);
	est_pose.position = [meanX, meanY, meanZ];


	#find the mean orientation	
	x_values = y_values = z_values = w_values = 0;
	for p in wanted_set.poses:
	    x_values = x_values + p.orientation.x;
            y_values = y_values + p.orientation.y;
            z_values = z_values + p.orientation.z;
	    z_values = z_values + p.orientation.w;
	meanX = x_values / len(self.wanted_set.poses);
	meanY = y_values / len(self.wanted_set.poses);
	meanZ = z_values / len(self.wanted_set.poses);
	meanW = w_values / len(self.wanted_set.poses);
	est_pose.orientation = [meanX, meanY, meanZ, meanW];

        return est_pose
