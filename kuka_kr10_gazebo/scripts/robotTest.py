#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_msgs.msg import Header

from scipy.spatial.transform import Rotation as R
import numpy as np

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


        iter = 1.0
        row_no, col_no = poses2go.shape
        for i in range(0, row_no):
            point = JointTrajectoryPoint()
            iter +=1
            #for j in range(0, col_no):
            point.positions = poses2go[i, 0:col_no-1].tolist()[0]
            point.time_from_start.nsecs = 0
            point.time_from_start.secs = int(iter*1) ## Vrijeme za dolazak u zadanu tocku

            traj_vector.points.append(point)
    
 

        print (traj_vector)
        #print ('All systems go!')
        self.trajPub.publish(traj_vector)   

    def get_dk(self, joint_state):
        # Funkcija implementira direktnu kinematiku robota KUKA KR10
        # ULAZI Funkcije: Pozicija zglobova robota joint_state za koju 
        #                 je potrebno odrediti direktnu kinematiku, 
        #                 izrazeno kao numpy vektor 6x1.
        # IZLAZI Funkcije: Poza flandze robota izrazena kao numpy vektor 6x1.

        pass

    def get_ik(self, goal_pose, temp_joint_state):
        # Funkcija implementira analiticku inverznu kinematiku robota KUKA KR10
        # ULAZI Funkcije:
        #   - goal_pose: Kartezijska poza flandze robota izrazena kao numpy vektor 6x1
        #   - temp_joint_state: Trenutna pozicija zglobova robota kao numpy vektor 6x1
        # IZLAZI Funkcije: Najblize rjesenje inverzne trenutnoj pozi, 
        #                   izrazeno kao numpy vektor 6x1.

        pass

    def taylor_path(self, w_1, w_2, q_0, tol=0.01):
        # Funkcija implementira Taylorov postupak
        # ULAZI Funkcije:
        #   - w_1: Kartezijska poza pocetne tocke, izrazena kao numpy vektor 6x1
        #   - w_2: Kartezijska poza krajnje tocke, izrazena kao numpy vektor 6x1
        #   - q_0: Pocetna poza zglobova robota, izrazena kao numpy vektor Nx1 
        #   - tol: Zadana tolerancija, izrazena kao Float64
        # IZLAZI Funkcije: Tocke putanje, izrazene kao numpy matrica, 
        #                  gdje je svaki novi red nova tocka.

        pass

    def tmatrix2pose(self, T):
        # Funkcija pretvara zadanu matricu transformacije u Pose
        # ULAZI Funkcije:
        #   - R: rotacijska matrica, u obliku numpy matrix 3x3
        # IZLAZI Funkcije: Tocke putanje, izrazene kao Pose.

        r = R.from_matrix([[T[0, 0], T[0, 1], T[0, 2]],
                           [T[1, 0], T[1, 1], T[1, 2]],
                           [T[2, 0], T[2, 1], T[2, 2]]])
        poseMsg = Pose()

        poseMsg.position.x = T[0, 3]
        poseMsg.position.y = T[1, 3]
        poseMsg.position.z = T[2, 3]
        poseMsg.orientation.x = r.as_quat()[0]
        poseMsg.orientation.y = r.as_quat()[1]
        poseMsg.orientation.z = r.as_quat()[2]
        poseMsg.orientation.w = r.as_quat()[3]

        return poseMsg

    def pose2tmatrix(self, robot_pose):
        # Funkcija pretvara zadanu pozu robota kao Pose u matricu transformacije
        # ULAZI Funkcije:
        #   - robot_pose: poza robota izrazena kao Pose
        # IZLAZI Funkcije: Poza robota izrazena kao transformacijska matrica.
        
        t_matrix = np.eye(4)
        # positions
        t_matrix[0, 3] = robot_pose.position.x
        t_matrix[1, 3] = robot_pose.position.y
        t_matrix[2, 3] = robot_pose.position.z
        # orientation
        r = R.from_quat([robot_pose.orientation.x, 
                         robot_pose.orientation.y, 
                         robot_pose.orientation.z, 
                         robot_pose.orientation.w])
        for i in range(0, 3):
            for j in range(0, 3):
                t_matrix[i, j] = r.as_matrix()[i, j]

        #print (t_matrix)
        return t_matrix

    def run(self):
        rospy.sleep(1.0)

        print ("Pomakni robota u tocku.")
        ### Primjer slanja robota u tocke zadane
        self.publish_msg(np.matrix([[0.0, -1.5707, 1.5707, 0.0, 1.5707, 0.0, 0.0]]))
        rospy.sleep(5.0)

        ### Point-To-Point gibanje
        print ("Point-to-point ")
        q_ptp = np.matrix([ [ 0.3120,    0.5526,   -1.7611,   -0.3322,    1.2267,    0.1159],
                            [-0.3120,    0.5526,   -1.7611,    0.3322,    1.2267,   -0.1159],
                            [ 0.0000,    0.0268,   -1.4615,   -0.0000,    1.4347,    0.0000],
                            [ 0.3120,    0.5526,   -1.7611,   -0.3322,    1.2267,    0.1159]])

        self.publish_msg(q_ptp)
        rospy.sleep(5.0)

        ### Taylor gibanje
        print ("Taylor ")
        q_tay = np.matrix([ [ 0.3120,    0.5526,   -1.7611,   -0.3322,    1.2267,    0.1159],
                            [-0.0000,    0.5682,   -1.8305,    0.0000,    1.2624,   -0.0000],
                            [-0.3120,    0.5526,   -1.7611,    0.3322,    1.2267,   -0.1159],
                            [-0.1599,    0.2932,   -1.6661,    0.1630,    1.3755,   -0.0319],
                            [ 0.0000,    0.0268,   -1.4615,   -0.0000,    1.4347,    0.0000],
                            [ 0.1599,    0.2932,   -1.6661,   -0.1630,    1.3755,    0.0319],
                            [ 0.3120,    0.5526,   -1.7611,   -0.3322,    1.2267,    0.1159]])

        self.publish_msg(q_tay)
        rospy.sleep(6.0)


if __name__ == '__main__':

    rospy.init_node('RobotTest')
    node = PublishPoint()
    node.run()
