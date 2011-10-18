
###########################################################################
#  graspplanning.py - Graspplanning script
#
#  Created: Thu Oct 13 12:50:34 2011
#  Copyright  2011  Bahram Maleki-Fard, AllemaniACs RoboCup Team
#
############################################################################

#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.

import time
import openravepy
from openravepy import *
from numpy import *

## Class to plan a grasp for a given robot and target.
#
# This class loads a pregenerated grasping database and can use
# those grasps to find a valid grasp for the given target, and
# calculate a collision-free path for the arm to move to a grasping
# position.
class GraspPlanner:

    ## Constructor.
    #
    # @param robot the robot to be used for planning.
    # @param target the target KinBody.
    def __init__(self,robot,target):
        ## environment to be used
        self.envreal = robot.GetEnv()

	## robot to be used
	self.robot = robot

        ## target to be used
	self.target = target

        with self.envreal:
            gmodel = databases.grasping.GraspingModel(robot=self.robot,target=self.target)
	    print 'check if model can be loaded'
            if not gmodel.load():
                print 'need to autogenerate model'
                gmodel.autogenerate()
	    print gmodel;

        ## grasping model for given robot and target
	self.gmodel = gmodel

    ## Wait for robot to complete action.
    # @param robot The robot to be checked.
    # @return void
    def waitrobot(self,robot=None):
        if robot is None:
            robot = self.robot
        while not robot.GetController().IsDone():
            time.sleep(0.01)

    ## Grasps an object.
    # This version returns the first valid grasp found. Should be tweaked in later
    # versions, as the first valid grasp might be at the bottom of the target
    # instead of the middle, which would be preferred.
    # @return graspindex if successfull, -1 if failed to find valid grasp
    def graspObject(self):
        env = self.envreal
        robot = self.robot
	gmodel = self.gmodel
	dests = None

        with env:
	    ## taskmanipulation problem/module
            self.taskmanip = interfaces.TaskManipulation(self.robot,graspername=gmodel.grasper.plannername)

        approachoffset = 0.0
	istartgrasp = 0
	target = gmodel.target

	while istartgrasp < len(gmodel.grasps):
            goals,graspindex,searchtime,trajdata = self.taskmanip.GraspPlanning(graspindices=gmodel.graspindices,grasps=gmodel.grasps[istartgrasp:],
                                                                                target=target,approachoffset=approachoffset,destposes=dests,
                                                                                seedgrasps = 3,seeddests=8,seedik=1,maxiter=1000,
                                                                                randomgrasps=True,randomdests=True,outputtraj=True,execute=False)
            istartgrasp = graspindex+1
	    ## stored trajectory for planned path
	    self.trajdata = trajdata
            Tglobalgrasp = gmodel.getGlobalGraspTransform(gmodel.grasps[graspindex],collisionfree=True)

            print 'grasp %d initial planning time: %f'%(graspindex,searchtime)
	    print 'goals:'
	    print goals
	    print 'trajdata'
	    print trajdata
            self.waitrobot(robot)

            with env:
                robot.ReleaseAllGrabbed()

            return graspindex # return successful grasp index

        # exhausted all grasps
        return -1

## Run graspplanning.
# @param envId unique id of the OpenRAVE Environment
# @param robotName unique name of the OpenRAVE Robot
# @param targetName unique name of the target (an OpenRAVE KinBody)
# @return planned grasping trajectory as a string
def runGrasp(envId, robotName, targetName):
    env = RaveGetEnvironment(envId)
    robot = env.GetRobot(robotName)
    target = env.GetKinBody(targetName)

    self = GraspPlanner(robot, target)
    try:
        print 'grasping object %s'%self.target.GetName()
        with self.envreal:
            self.robot.ReleaseAllGrabbed()
        success = self.graspObject()
        print 'success: ',success
	return self.trajdata
    except planning_error, e:
        print 'failed to grasp object %s'%self.target.GetName()
        print e