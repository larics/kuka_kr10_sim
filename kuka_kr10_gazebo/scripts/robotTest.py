#!/usr/bin/env python
import sys
import rospy
import copy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_msgs.msg import Header

from scipy.spatial.transform import Rotation as R
import numpy as np

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState
from tf import TransformListener

import math

class PublishPoint():
    def __init__(self):
        self.trajPub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

        rospy.Subscriber("/joint_states", JointState, self.jointStateCallback)

        self._jointState = JointState()

        self.tf = TransformListener()

        moveGroupName = 'kr10_plain'

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # robotGroupName is the group from moveit
        group = moveit_commander.MoveGroupCommander(moveGroupName)
        group.allow_replanning(True)
        group.allow_looking(True)


        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()


        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.fileForLogging = None

        self.DH_table = np.matrix([[0.0,                    400.0/1000.,    math.pi/2,  25.0/1000.],
                                   [math.pi/2-math.pi/2*1,  0.0,            0.0,        560.0/1000.],
                                   [0.0+math.pi/2*1,        0.0,            math.pi/2,  35.0/1000.],
                                   [math.pi*1,              515.0/1000.,    math.pi/2,  0.0],
                                   [math.pi*1,              0.0,            math.pi/2,  0.0],
                                   [-math.pi/2*1,           80.0/1000.,     0.0,        0.0]])

        self.DH_joint_correction = [-1.0, -1.0, -1.0, -1.0, -1.0, 1.0]

        self.W = np.matrix([[0.6,  0.0, 0.8, 1, 0, 0],
                            [0.6, -0.3, 0.4, 1, 0, 0],
                            [0.6,  0.0, 0.1, 1, 0, 0],
                            [0.6,  0.3, 0.4, 1, 0, 0]])

        self.Q = np.zeros(np.shape(self.W))

    def jointStateCallback(self, msg):

        self._jointState = msg

    def publish_msg(self, poses2go, delta_t=1):
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


        iter = 0
        row_no, col_no = poses2go.shape
        #print ("row_no, col_no: ", row_no, col_no)
        for i in range(0, row_no):
            point = JointTrajectoryPoint()
            iter +=1
            point.positions = poses2go[i, :].tolist()[0]
            point.time_from_start = rospy.Duration.from_sec(iter*delta_t)  # Vrijeme za dolazak u zadanu tocku

            traj_vector.points.append(point)
    
        self.trajPub.publish(traj_vector)   

    def get_dk(self, joint_state):
        # Funkcija implementira direktnu kinematiku robota KUKA KR10
        # ULAZI Funkcije: Pozicija zglobova robota joint_state za koju 
        #                 je potrebno odrediti direktnu kinematiku, 
        #                 izrazeno kao numpy vektor 6x1.
        # IZLAZI Funkcije: Poza flandze robota izrazena kao numpy vektor 6x1.

        pass
    
    def get_ik (self, target, temp_joint_state):
        # Inverzna kinematika preko MoveiITa.
        # ULAZI Funkcije:
        #   - target: Kartezijska pozicija flandze robota izrazena kao numpy vektor 3x1
        #             Orijentacija fiksna, prema naprijed
        #   - temp_joint_state: Trenutna pozicija zglobova robota kao lista 6x1
        # IZLAZI Funkcije: Rjesenje inverzne kinematike kao lista 6x1

        targetPose = PoseStamped()
        targetPose.header.stamp = rospy.Time.now()
        targetPose.pose.position.x = target[0]
        targetPose.pose.position.y = target[1]
        targetPose.pose.position.z = target[2]
        targetPose.pose.orientation.w = 1 

        robotTemp = RobotState()
        robotTemp.joint_state.name = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        robotTemp.joint_state.position = temp_joint_state

        service_request = PositionIKRequest()
        service_request.group_name = "kr10_plain"
        service_request.ik_link_name = "link_6"
        service_request.pose_stamped = targetPose
        service_request.robot_state = robotTemp
        service_request.timeout.secs = 1

        rospy.wait_for_service('compute_ik')
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        resp = compute_ik(service_request)
    
        return list(resp.solution.joint_state.position)


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
        

    def interpolate_q(self, Q, T_param):
        # Polinomska interpolacija jedinstvenim polinomom u prostoru zglobova.
        # Svaki od 4 reda ulazne matrice Q predstavlja vektor vrijednosti zglobova
        # kroz koji manipulator mora proci. Izlaz funkcije su vrijednosti 
        # otipkanog polinoma frekvencijom 10 Hz.
        # ULAZI Funkcije:
        #   - Q: Točke putanje u prostoru zglobova, izrazena kao numpy matrica Nx6
        #   - T_param: Parametrička vrijemena segmenta
        # IZLAZI Funkcije: Matrica točaka zglobova, otipkanog polinoma 
        #       frekvencijom 1 Hz.

        pass

    def ho_cook(self, Q, v_max_lim, a_max_lim, f_s=250):
        # Funkcija implementira Ho-Cookovu metodu planiranja trajektorije
        # robotske ruke postivajuci zadana ogranicenja brzine (v_max) i akceleracije
        # (a_max). Funkcija kao rezultat vraca otipkanu trajektoriju frekvencijom
        # f_s u prostoru zglobova
        # ULAZI Funkcije:
        #   - Q: Točke putanje u prostoru zglobova, izrazena kao numpy matrica Nx6
        #   - v_max: Ograničenja brzina zglobova, izrazeno kao vektor 6x1
        #   - a_max: Ograničenja akceleracija zglobova, izrazeno kao vektor 6x1
        #   - f_s: Frekvencija otipkavanja trajektorije
        # IZLAZI Funkcije: Otipkane točke trajektorije, izrazene kao numpy matrica, 
        #                  gdje je svaki novi red nova tocka.

        m, n = np.shape(Q)

        # Racunanje parametrickog vremena (5.24)
        T = np.zeros((m, 1))
        for k in range(0, m-1):
            T[k+1] = ...

        # Prva iteracija
        iter_max = 10;
        iter_cnt = 0;
        S = 0
        while (round(S, 2) != 1.00 and iter_cnt < iter_max):
            if (iter_cnt > 0):   
                # Skaliraj parametricka vremena
                T = T*S
            else:
                # U prvoj iteraciji preskoci skaliranje
                S = 1;
            iter_cnt = iter_cnt + 1


            # Izracunaj matrice M, A i Dq (5.50)
            # Popuni matricu M
            M = np.zeros((m-2, m-2))
            M[0, 0] = 3/T[1] + 2/T[2]
            ...

            # Popuni matricu A
            A = np.zeros((n, m-2))
            A[:, 0] = np.sum(((6/T[1]**2)*np.sum([Q[1, :], -1*Q[0, :]], axis=0), (3/T[2]**2)*np.sum([Q[2, :], -1*Q[1, :]], axis=0)), axis=0)
            ...

            # Izracunaj Dq
            Dq = np.matmul(A, np.linalg.pinv(M))

            # Izracunaj matricu B
            B4 = np.zeros((n, 5, 2))
            B3 = np.zeros((n, 4, m-3))

            # Prvi segment (5.35)
            Q1 = np.zeros((n, 4))
            Q1[:, 0] = ...

            T1 = np.zeros((4, 5))
            T1[0, 0] = ...

            B4[:, :, 0] = ...

            # Medjusegmenti (5.23)
            for i in range(1, m-2):
                T_temp = np.zeros((4, 4))
                Q_temp = np.zeros((n, 4))

                Q_temp[:, 0] = ...

                T_temp[0, 0] = ...

                B3[:, :, i-1] = ...

            # Zadnji segment (5.41)
            Tm = np.zeros((4, 5))
            Qm = np.zeros((n, 4))

            Qm[:, 0] = ...

            Tm[0, 0] = ...
            
            B4[:, :, 1] = ...

            # Odredi max. brzine i akceleracije
            V_max = np.zeros((n, m-1))
            for i in range(0, n):
                V_max[i, 0] =       max(abs((np.polyval(np.polyder(B4[i, range(4, -1, -1), 0]),     np.linspace(0, T[1], (T[1]/0.01))) )))
            for k in range(1, m-2):
                for i in range(0, n):
                    V_max[i, k] =   max(abs((np.polyval(np.polyder(B3[i, range(3, -1, -1), k-1]),   np.linspace(0, T[k+1], (T[k+1]/0.01))) )))
            for i in range(0, n):
                V_max[i, m-2] =     max(abs((np.polyval(np.polyder(B4[i, range(4, -1, -1), 1]),     np.linspace(0, T[m-1], (T[m-1]/0.01))) )))

            A_max = np.zeros((n, m-1))
            for i in range(0, n):
                A_max[i, 0] =  ...
            for k in range(1, m-2):
                for i in range(0, n):
                    A_max[i, k] = ...
            for i in range(0, n):
                A_max[i, m-2] =   ...

            v_max = np.zeros((n, 1))
            a_max = np.zeros((n, 1))
            for i in range(0, n):
                v_max[i, 0] = max(V_max[i, :])
                a_max[i, 0] = max(A_max[i, :])

            Sv = 0
            Sa = 0 
            # Odredi faktore normiranja (5.52-55)
            for i in range(0, n):
                if (v_max[i, 0]/v_max_lim[i] > Sv):
                    Sv = v_max[i, 0]/v_max_lim[i]
                if (a_max[i, 0]/a_max_lim[i] > Sa):
                    Sa = (a_max[i, 0]/a_max_lim[i])**0.5
            # Odredi ukupni faktor normiranja
            S = ...

        # Otipkavanje trajektorije
        Ts = 1/f_s 
        # Prvi segment
        for i in range(0, n):
            temp_q = np.polyval(B4[i, range(4, -1, -1), 0], np.linspace(0, T[1], (T[1]/Ts)))
            temp_dq = np.polyval(np.polyder(B4[i, range(4, -1, -1), 0]), np.linspace(0, T[1], (T[1]/Ts)))
            temp_ddq = np.polyval(np.polyder(np.polyder(B4[i, range(4, -1, -1), 0])), np.linspace(0, T[1], (T[1]/Ts)))

            if (i == 0):
                temp_seg_q = np.matrix(temp_q)
                temp_seg_dq = np.matrix(temp_dq)
                temp_seg_ddq = np.matrix(temp_ddq)
            else:
                temp_seg_q = np.hstack((temp_seg_q, np.matrix(temp_q)))
                temp_seg_dq = np.hstack((temp_seg_dq, np.matrix(temp_dq)))
                temp_seg_ddq = np.hstack((temp_seg_ddq, np.matrix(temp_ddq)))
        Q_q = copy.deepcopy(temp_seg_q)
        Q_dq = copy.deepcopy(temp_seg_dq)
        Q_ddq = copy.deepcopy(temp_seg_ddq)
        
        # Medjusegmenti segment
        for k in range(1, m-2):
            for i in range(0, n):
                temp_q = np.polyval(B3[i, range(3, -1, -1), k-1], np.linspace(0, T[k+1], (T[k+1]/Ts)))
                temp_dq = np.polyval(np.polyder(B3[i, range(3, -1, -1), k-1]), np.linspace(0, T[k+1], (T[k+1]/Ts)))
                temp_ddq = np.polyval(np.polyder(np.polyder(B3[i, range(3, -1, -1), k-1])), np.linspace(0, T[k+1], (T[k+1]/Ts)))

                if (i == 0):
                    temp_seg_q = np.matrix(temp_q)
                    temp_seg_dq = np.matrix(temp_dq)
                    temp_seg_ddq = np.matrix(temp_ddq)
                else:
                    temp_seg_q = np.hstack((temp_seg_q, np.matrix(temp_q)))
                    temp_seg_dq = np.hstack((temp_seg_dq, np.matrix(temp_dq)))
                    temp_seg_ddq = np.hstack((temp_seg_ddq, np.matrix(temp_ddq)))
            if (k==1):
                t_q = np.matrix(temp_seg_q)
                t_dq = np.matrix(temp_seg_dq)
                t_ddq = np.matrix(temp_seg_ddq)
            else:
                t_q = np.vstack((t_q, temp_seg_q))
                t_dq = np.vstack((t_dq, temp_seg_dq))
                t_ddq = np.vstack((t_ddq, temp_seg_ddq))
        Q_q = copy.deepcopy(np.vstack((Q_q, t_q)))
        Q_dq = copy.deepcopy(np.vstack((Q_dq, t_dq)))
        Q_ddq = copy.deepcopy(np.vstack((Q_ddq, t_ddq)))
        
        # Zadnji segment
        for i in range(0, n):
            temp_q = np.polyval(B4[i, range(4, -1, -1), 1], np.linspace(0, T[m-1], (T[m-1]/Ts)))
            temp_dq = np.polyval(np.polyder(B4[i, range(4, -1, -1), 1]), np.linspace(0, T[m-1], (T[m-1]/Ts)))
            temp_ddq = np.polyval(np.polyder(np.polyder(B4[i, range(4, -1, -1), 1])), np.linspace(0, T[m-1], (T[m-1]/Ts)))

            if (i == 0):
                temp_seg_q = np.matrix(temp_q)
                temp_seg_dq = np.matrix(temp_dq)
                temp_seg_ddq = np.matrix(temp_ddq)
            else:
                temp_seg_q = np.hstack((temp_seg_q, np.matrix(temp_q)))
                temp_seg_dq = np.hstack((temp_seg_dq, np.matrix(temp_dq)))
                temp_seg_ddq = np.hstack((temp_seg_ddq, np.matrix(temp_ddq)))
        Q_q = np.vstack((Q_q, temp_seg_q))
        Q_dq = np.vstack((Q_dq, temp_seg_dq))
        Q_ddq = np.vstack((Q_ddq, temp_seg_ddq))
        

        return [Q_q, Q_dq, Q_ddq]

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
        # Inverz waypointa
        for i in range(0, np.shape(self.W)[0]):
            self.Q[i] = np.matrix(self.get_ik(self.W[i].tolist()[0], [0, -math.pi/2, math.pi/2, 0, 0, 0]))

        # Point-To-Point
        self.publish_msg(..., 2)
        rospy.sleep(10)

        # Taylor
        #self.publish_msg(taylor_q, 1)
        #rospy.sleep(20.0)

        
        # Interpolacija
        # Odredi parametricka vremena
        
        #...
        #Q_interpolate = self.interpolate_q(self.Q, t_param)

        self.publish_msg(Q_interpolate, 0.1)
        rospy.sleep(15.0)

        # Ho-Cook
        [HC_Q, HC_dQ, HC_ddQ] = self.ho_cook(taylor_q, [...], [...])
        print (HC_Q)
        self.publish_msg(HC_Q, 1.0/250)


if __name__ == '__main__':

    rospy.init_node('RobotTest')
    node = PublishPoint()
    node.run()
