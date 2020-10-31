from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np

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
        Liviu to do

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
        # ----- Initialize the particle cloud as an empty array
        self.particlecloud = []

        """Create the noise to multiply by the random Gaussian number that will 
        get added to each of the Poses, that are set to a random position
        and orientation around the initial pose"""
        sensorSigma=3 #variance
        sensorMu=0 #mean
        noise=sensorSigma * np.random.randn(1,2) + sensorMu

        """Create a range for the ammount of random Gaussian values to generate
        having the number of predicted readings be 10% of this value
        and then generate those random Gaussian values"""

        randomGauss = 10*NUMBER_PREDICTED_READINGS

        for i in randomGauss:
            gaussianRandomNum=gauss(0,1)   
        
        # ----- randomize yaw(heading)
        x=random(0,180)
        randomYaw=(MATH.PI/x)

        iterator = 0
        
        """Set the particles to a random position and orientation around the initial pose
        """
        particleNumber = 10**2 # 10**3 # 10**4 # 10**5 experiment with different ammounts of particles

        while iterator < particleNumber:
            particle = [[initialpose.position.x + (gaussianRandomNum * noise), 
                        initialpose.position.y + (gaussianRandomNum * noise), 
                        initialpose.position.z + (gaussianRandomNum * noise)],
                        rotateQuaternion(initialpose.quaternion, randomYaw)]
            self.particlecloud.add(particle)
            iterator += 1

        return particlecloud



    def update_particle_cloud(self, scan):
        """
        Jamie and Traci to do

        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        """
        Initialise arrays for the new particle cloud,
        particle weights and cummulative weights
        """
        newParticleCloud = []
        particleWeights = []
        cummulativeWeights = []

        """
        Build the particle weights array using the get weight
        function provided
        """
        for p in self.particlecloud:
            particleWeights.Add(get_weight(self, scan, p))

        """
        From the weights calculated create the cummulative
        weights array needed for the resampling algorithm.
        If weights for 3 particles were [0.3, 0.5, 0.2] then
        cummulative weights will be [0.3, 0.8, 1.0]
        """
        cummulative = 0

        for w in particleWeights:
            cummulative += particleWeights[counter]
            cummulativeWeights.Add(cummulative)


        """
        For the resampling process go through each of the cummulative
        weights. With each weight add number of particles based on the
        sampling size (defined as 20 in this case) and the accumulative
        weight of the particle.

        In example used above it would be 0.3 > 0 add one particle,
        then check again as while loop 0.3 > 0.05(1/20) and then add
        another particle. This is repeated until while loop criteria
        is no longer hit and then it goes onto the next accumulative particle
        """
        particleCounter = 0
        cummulativeMValue = 0

        for cm in cummulativeWeights:
            while cm > cummulativeMvalue:
                newParticleCloud.Add(self.particlecloud[particleCounter])
                cummulativeMValue += 1/self.NUMBER_PREDICTED_READINGS
            particleCounter += 1

        """
        Overwrite the old particle cloud with the new resampled one
        """
        self.particlecloud = newParticleCloud

        pass

    def estimate_pose(self):
        """
        Marcia and Margarita to do

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
        pass
