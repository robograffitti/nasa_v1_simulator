#! /usr/bin/env python

import roslib; roslib.load_manifest('V1')
import rospy
import actionlib
import math
import random

from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from copy import copy, deepcopy

TORAD = math.pi/180.0
TODEG = 1.0/TORAD

class TrajectoryGenerator :

    def __init__(self, N, wp, group, jointNames, robotName):


        self.group = group
        self.currentData = None
        self.desiredData = None
        self.deadlineData = None

        self.currentState = JointState()
        self.currentState.position = [0]*N
        self.currentState.velocity = [0]*N
        self.currentState.effort = [0]*N
        self.numJoints = N
        self.waypoints = wp
        self.robotName = robotName

        self.jointNames = jointNames
        self.actionGoal = FollowJointTrajectoryGoal()

        print "TrajectoryGenerator::init(" + self.group + ") -- creating new TrajectoryGenerator"

        rospy.Subscriber(str(self.robotName + "/joint_states"), JointState, self.jointStateCallback)

        self.trajClient = actionlib.SimpleActionClient(str(self.robotName + '/' + self.group + '_controller/follow_joint_trajectory'), FollowJointTrajectoryAction)

        print "TrajectoryGenerator::init(" + self.group + ") -- connecting to action server"
        self.trajClient.wait_for_server()
        print "TrajectoryGenerator::init(" + self.group + ") -- connected"



    def getNumJoints(self) :
        return self.numJoints

    def jointStateCallback(self, data):
        self.currentState = data

    def computeTrajectory(self, desiredData, deadline):

        jointTraj = JointTrajectory()
        currentState = copy(self.currentState)
        desiredState = copy(desiredData)

        # create simple lists of both current and desired positions, based on provided desired names
        rospy.loginfo("TrajectoryGenerator::computeTrajectory() -- finding necessary joints")
        desiredPositions = []
        currentPositions = []
        for desIndex in range(len(desiredState.name)) :
            for curIndex in range(len(currentState.name)) :
                if ( desiredState.name[desIndex] == currentState.name[curIndex] ) :
                    desiredPositions.append(desiredState.position[desIndex])
                    currentPositions.append(currentState.position[curIndex])

        rospy.loginfo("TrajectoryGenerator::computeTrajectory() -- creating trajectory")
        jointTraj.joint_names = desiredState.name
        jointTraj.points = list()

        for j in range(self.waypoints) :
            trajPoint = JointTrajectoryPoint()

            t = (deadline / self.waypoints) * (j + 1)
            trajPoint.time_from_start = rospy.Duration(t)

            trajPoint.positions = list()
            for i in range(len(desiredPositions)) :
                trajPoint.positions.append( self.minJerk(currentPositions[i], desiredPositions[i], deadline, t) )

            jointTraj.points.append(trajPoint)

        rospy.loginfo("TrajectoryGenerator::moveToGoal() -- using tolerances")

        return jointTraj


    def minJerk(self, start, end, duration, t):
        tOverD = float(t) / float(duration)
        return start + (end - start)*( 10*math.pow(tOverD,3) - 15*math.pow(tOverD,4) + 6*math.pow(tOverD,5) )

    def moveToGoal(self, jointGoal, deadline, useTolerances) :

        self.actionGoal.trajectory = self.computeTrajectory(jointGoal, deadline)

        if useTolerances :
            rospy.loginfo("TrajectoryGenerator::moveToGoal() -- using tolerances")
            self.actionGoal.path_tolerance = []
            self.actionGoal.goal_tolerance = []

            for i in range(self.numJoints):
                tol = JointTolerance()
                tol.name = self.jointNames[i]
                tol.position = 0.2
                tol.velocity = 1
                tol.acceleration = 10
                self.actionGoal.path_tolerance.append(tol)
                self.actionGoal.goal_tolerance.append(tol)

        else :
            rospy.loginfo("TrajectoryGenerator::moveToGoal() -- not using tolerances")

        self.actionGoal.goal_time_tolerance = rospy.Duration(10.0)

        # send goal and monitor response
        self.trajClient.send_goal(self.actionGoal)

        rospy.loginfo("TrajectoryGenerator::moveToGoal() -- returned state: %s", str(self.trajClient.get_state()))
        rospy.loginfo("TrajectoryGenerator::moveToGoal() -- returned result: %s", str(self.trajClient.get_result()))

        return

    def formatJointStateMsg(self, j, offset) :

        if not (len(j) == self.numJoints) :
            rospy.logerr("TrajectoryGenerator::formatJointStateMsg() -- incorrectly sized joint message")
            return None

        js = JointState()
        js.header.seq = 0
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = ""
        js.name = []
        js.position = []

        for i in range(self.numJoints):
            js.name.append(self.jointNames[i])
            js.position.append(j[i])

        return js


if __name__ == '__main__':
    rospy.init_node('ready_pose')
    try:

        leftHandJointNames = ['LeftThumbMPYaw', 'LeftThumbMPExtensor', 'LeftThumbPIPExtensor', 'LeftThumbDIPExtensor', \
                                'LeftPrimaryMPYaw', 'LeftPrimaryMPExtensor', 'LeftPrimaryPIPExtensor', 'LeftPrimaryDIPExtensor', \
                                'LeftMiddleMPExtensor', 'LeftMiddlePIPExtensor', 'LeftMiddleDIPExtensor', \
                                'LeftLittleMPYaw', 'LeftLittleMPExtensor', 'LeftLittlePIPExtensor', 'LeftLittleDIPExtensor']
        rightHandJointNames =['RightThumbMPYaw', 'RightThumbMPExtensor', 'RightThumbPIPExtensor', 'RightThumbDIPExtensor', \
                                'RightPrimaryMPYaw', 'RightPrimaryMPExtensor', 'RightPrimaryPIPExtensor', 'RightPrimaryDIPExtensor', \
                                'RightMiddleMPExtensor', 'RightMiddlePIPExtensor', 'RightMiddleDIPExtensor', \
                                'RightLittleMPYaw', 'RightLittleMPExtensor', 'RightLittlePIPExtensor', 'RightLittleDIPExtensor']
        rightArmJointNames = ['RightShoulderExtensor','RightShoulderAdductor','RightShoulderSupinator','RightElbowExtensor', \
                                'RightForearmSupinator','RightWristExtensor','RightWrist']
        leftArmJointNames = ['LeftShoulderExtensor','LeftShoulderAdductor','LeftShoulderSupinator','LeftElbowExtensor', \
                                'LeftForearmSupinator','LeftWristExtensor','LeftWrist']
        rightLegJointNames = ['RightHipRotator','RightHipAdductor','RightHipExtensor','RightKneeExtensor','RightAnkleExtensor','RightAnkle']
        leftLegJointNames = ['LeftHipRotator','LeftHipAdductor','LeftHipExtensor','LeftKneeExtensor','LeftAnkleExtensor','LeftAnkle']
        neckJointNames = ['LowerNeckExtensor', 'NeckRotator', 'UpperNeckExtensor']
        waistJointNames = ['WaistRotator','WaistExtensor','WaistLateralExtensor']

        v1TrajectoryGeneratorLeftArm = TrajectoryGenerator(7, 1, "left_arm", leftArmJointNames, "v1")
        v1TrajectoryGeneratorRightArm = TrajectoryGenerator(7, 1, "right_arm", rightArmJointNames, "v1")
        v1TrajectoryGeneratorLeftLeg = TrajectoryGenerator(6, 1, "left_leg", leftLegJointNames, "v1")
        v1TrajectoryGeneratorRightLeg = TrajectoryGenerator(6, 1, "right_leg", rightLegJointNames, "v1")
        v1TrajectoryGeneratorNeck = TrajectoryGenerator(3, 1, "neck", neckJointNames, "v1")
        v1TrajectoryGeneratorWaist = TrajectoryGenerator(3, 1, "waist", waistJointNames, "v1")
        # v1TrajectoryGeneratorLeftHand = TrajectoryGenerator(15, 1, "left_hand", "v1")
        # v1TrajectoryGeneratorRightHand = TrajectoryGenerator(15, 1, "right_hand", "v1")

        # rospy.sleep(2)

        lhrp = [0]*15
        rhrp = [0]*15

        larp = [-0.3, 0.2, 0.0, -1.4, 0.0, 0.0, 0.0]
        rarp = [-0.3, 0.2, 0.0, -1.4, 0.0, 0.0, 0.0]
        llrp = [0.0, -0.1, 0.0, -0.7, -0.3, 0.1]
        rlrp = [0.0, -0.1, 0.0, -0.7, -0.3, 0.1]
        nrp = [0.0, 0.0, 0.05]
        wrp = [0.0, -0.2, 0.0]

        print "ready_pose() -- moving to ready pose"

        jointGoalLeftArm = v1TrajectoryGeneratorLeftArm.formatJointStateMsg(larp, 0)
        jointGoalRightArm = v1TrajectoryGeneratorRightArm.formatJointStateMsg(rarp, 0)
        jointGoalLeftLeg = v1TrajectoryGeneratorLeftLeg.formatJointStateMsg(llrp, 0)
        jointGoalRightLeg = v1TrajectoryGeneratorRightLeg.formatJointStateMsg(rlrp, 0)
        jointGoalNeck = v1TrajectoryGeneratorNeck.formatJointStateMsg(nrp, 0)
        jointGoalWaist = v1TrajectoryGeneratorWaist.formatJointStateMsg(wrp, 0)
        # jointGoalLeftHand = v1TrajectoryGeneratorLeftHand.formatJointStateMsg(lhrp, 0)
        # jointGoalRightHand = v1TrajectoryGeneratorRightHand.formatJointStateMsg(rhrp, 0)

        for i in range(1) :
            v1TrajectoryGeneratorLeftArm.moveToGoal(jointGoalLeftArm, 0.5, False)
            v1TrajectoryGeneratorRightArm.moveToGoal(jointGoalRightArm, 0.5, False)
            v1TrajectoryGeneratorLeftLeg.moveToGoal(jointGoalLeftLeg, 0.5, False)
            v1TrajectoryGeneratorRightLeg.moveToGoal(jointGoalRightLeg, 0.5, False)
            v1TrajectoryGeneratorNeck.moveToGoal(jointGoalNeck, 0.5, False)
            v1TrajectoryGeneratorWaist.moveToGoal(jointGoalWaist, 0.5, False)
            # v1TrajectoryGeneratorLeftHand.moveToGoal(jointGoalLeftHand, 0.5, False)
            # v1TrajectoryGeneratorRightHand.moveToGoal(jointGoalRightHand, 0.5, False)
            rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass




