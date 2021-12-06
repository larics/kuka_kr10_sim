#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_msgs.msg import Header

class PublishPoint():
    def __init__(self):
        self.trajPub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

        rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)

        self._jointState = JointState()

    def jointStateCallback(self, msg):

        self._jointState = msg

    def publish_msg(self, poses2go):
        # Set the message to publish as command.
        traj_vector = JointTrajectory()
        # Current ROS time stamp
        h = Header()
        h.stamp = rospy.Time.now()
        traj_vector.header = h
        traj_vector.joint_names.append('joint_a1');
        traj_vector.joint_names.append('joint_a2');
        traj_vector.joint_names.append('joint_a3');
        traj_vector.joint_names.append('joint_a4');
        traj_vector.joint_names.append('joint_a5');
        traj_vector.joint_names.append('joint_a6');


        iter = 0.0
        for column in poses2go:
            point = JointTrajectoryPoint()
            iter +=1
            for q in column:
                point.positions.append(q)
                point.time_from_start.nsecs = 0
                point.time_from_start.secs = int(iter*1)
            traj_vector.points.append(point)
    
 

        print (traj_vector)
        print ('All systems go!')
        self.trajPub.publish(traj_vector)   

    def run(self):

        while not rospy.is_shutdown():

            self.publish_msg([[0.0, -1.5707, 1.5707, 0.0, 1.5707, 0.0, 0.0]])
            print ("Running!")
            rospy.sleep(3)


if __name__ == '__main__':

    rospy.init_node('RobotTest')
    node = PublishPoint()
    node.run()
