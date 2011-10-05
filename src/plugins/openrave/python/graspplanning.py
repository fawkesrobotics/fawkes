import time
import openravepy
from openravepy import *
from numpy import *

class MyGraspPlanning:
    def __init__(self,robot,target):
        self.envreal = robot.GetEnv()
	self.robot = robot
	self.target = target

        with self.envreal:
            gmodel = databases.grasping.GraspingModel(robot=self.robot,target=self.target)
	    print 'check if model can be loaded'
            if not gmodel.load():
                print 'need to autogenerate model'
                gmodel.autogenerate()
	    print gmodel;

	self.gmodel = gmodel

    def waitrobot(self,robot=None):
        """busy wait for robot completion"""
        if robot is None:
            robot = self.robot
        while not robot.GetController().IsDone():
            time.sleep(0.01)

    def graspObject(self,**kwargs):
        """grasps an object"""
        env = self.envreal
        robot = self.robot
	gmodel = self.gmodel
	dests = None

        with env:
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

def myGrasp(envId, robotName, targetName):
    env = RaveGetEnvironment(envId)
    robot = env.GetRobot(robotName)
    target = env.GetKinBody(targetName)

    self = MyGraspPlanning(robot, target)
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