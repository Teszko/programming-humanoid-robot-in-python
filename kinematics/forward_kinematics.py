'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH

from __future__ import print_function

import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
from numpy import sin, cos, dot, arcsin

from keyframes import wipe_forehead

from angle_interpolation import AngleInterpolationAgent



class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       }

        self.kinematik_values = {'HeadYaw': 0,
                                 'HeadPitch': 0,
                                 'LShoulderPitch': 0,
                                 'LShoulderRoll': 0,
                                 'LElbowYaw': 0,
                                 'LElbowRoll': 0,
                                 'LHipYawPitch': 0,
                                 'LHipRoll': 0,
                                 'LHipPitch': 0,
                                 'LKneePitch': 0,
                                 'LAnklePitch': 0,
                                 'LAnkleRoll': 0,
                                 'RHipYawPitch': 0,
                                 'RHipRoll': 0,
                                 'RHipPitch': 0,
                                 'RKneePitch': 0,
                                 'RAnklePitch': 0,
                                 'RAnkleRoll': 0,
                                 'RShoulderPitch': 0,
                                 'RShoulderRoll': 0,
                                 'RElbowYaw': 0,
                                 'RElbowRoll': 0
                                 }

    def print_kinematic_values(self):
        out = "LElbowRoll: " + str(self.kinematik_values['LElbowRoll']) + " "
        out += "RElbowRoll: " + str(self.kinematik_values['RElbowRoll']) + " "
        out += "LAnkleRoll: " + str(self.kinematik_values['LAnkleRoll']) + " "
        out += "RAnkleRoll: " + str(self.kinematik_values['RAnkleRoll']) + " "
        print(out, end=" ")

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    @staticmethod
    def yaw_matrix(joint_angle, x, y, z):
        # Z rotation
        s = sin(joint_angle)
        c = cos(joint_angle)
        return matrix([[c,  -s, 0, 0],
                       [s,  c,  0, 0],
                       [0,  0,  c, 0],
                       [x,  y,  z, 1]])

    @staticmethod
    def pitch_matrix(joint_angle, x, y, z):
        # Y rotation
        s = sin(joint_angle)
        c = cos(joint_angle)
        return matrix([[c,  0, s, 0],
                       [0,  1, 0, 0],
                       [-s, 0, c, 0],
                       [x,  y, z, 1]])

    @staticmethod
    def roll_matrix(joint_angle, x, y, z):
        # x rotation
        s = sin(joint_angle)
        c = cos(joint_angle)
        return matrix([[1,  0, 0,  0],
                       [0,  c, -s, 0],
                       [0,  s, c,  0],
                       [x,  y, z,  1]])

    @staticmethod
    def decompose_matrix(A):
        x=A[3, 0]
        y=A[3, 1]
        z=A[3, 2]
        rx=arcsin(-1 * A[1, 2])
        ry=arcsin(A[0, 2])
        rz=arcsin(A[0, 1])
        return [x, y, z, rx, ry, rz]

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE

        # 'Head': ['HeadYaw', 'HeadPitch']

        if joint_name == 'HeadYaw':
            T = self.yaw_matrix(joint_angle, 0, 0, 459)

        elif joint_name == 'HeadPitch':
            T1 = self.pitch_matrix(joint_angle, 0, 0, 0)
            T2 = self.pitch_matrix(0, 0, 0, 100)  # head hight
            T = dot(T1, T2)

        # 'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],

        elif joint_name == 'LShoulderPitch':
            T = self.pitch_matrix(joint_angle, 0, 98, 435)

        elif joint_name == 'LShoulderRoll':
            T = self.roll_matrix(joint_angle, 0, 0, 0)

        elif joint_name == 'LElbowYaw':
            T = self.yaw_matrix(joint_angle, 105, 15, 0)

        elif joint_name == 'LElbowRoll':
            T1 = self.roll_matrix(joint_angle, 0, 0, 0)
            T2 = self.roll_matrix(0, 113.7, 0, 0)
            T = dot(T1, T2)

        elif joint_name == 'RShoulderPitch':
            T = self.pitch_matrix(joint_angle, 0, -98, 435)

        elif joint_name == 'RShoulderRoll':
            T = self.roll_matrix(joint_angle, 0, 0, 0)

        elif joint_name == 'RElbowYaw':
            T = self.yaw_matrix(joint_angle, 105, -15, 0)

        elif joint_name == 'RElbowRoll':
            T1 = self.roll_matrix(joint_angle, 0, 0, 0)
            T2 = self.roll_matrix(0, 113.7, 0, 0)
            T = dot(T1, T2)

        # 'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
        elif joint_name == 'LHipYawPitch':
            R = self.yaw_matrix(45, 0, 0, 248)
            T1 = self.pitch_matrix(joint_angle, 0, 0, 0)
            T = dot(R, T1)

        elif joint_name == 'LHipRoll':
            T = self.roll_matrix(joint_angle, 0, 50, -50)

        elif joint_name == 'LHipPitch':
            T = self.pitch_matrix(joint_angle, 0, 0, 0)

        elif joint_name == 'LKneePitch':
            T = self.pitch_matrix(joint_angle, 0, 0, -100.0)

        elif joint_name == 'LAnklePitch':
            T = self.pitch_matrix(joint_angle, 0, 0, -102.9)

        elif joint_name == 'LAnkleRoll':
            T1 = self.roll_matrix(joint_angle, 0, 0, 0)
            T2 = self.roll_matrix(0, 0, 0, -45.19)
            T = dot(T1, T2)

        #  'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
        elif joint_name == 'RHipYawPitch':
            R = self.yaw_matrix(-45, 0, 0, 248)
            T1 = self.pitch_matrix(joint_angle, 0, 0, 0)
            T = dot(R, T1)

        elif joint_name == 'RHipRoll':
            T = self.roll_matrix(joint_angle, 0, -50, -50)

        elif joint_name == 'RHipPitch':
            T = self.pitch_matrix(joint_angle, 0, 0, 0)

        elif joint_name == 'RKneePitch':
            T = self.pitch_matrix(joint_angle, 0, 0, -100.0)

        elif joint_name == 'RAnklePitch':
            T = self.pitch_matrix(joint_angle, 0, 0, -102.9)

        elif joint_name == 'RAnkleRoll':
            T1 = self.roll_matrix(joint_angle, 0, 0, 0)
            T2 = self.roll_matrix(0, 0, 0, -45.19)
            T = dot(T1, T2)

        return T

    def transform_matrix(self, T, joint, angle):
        Tl = self.local_trans(joint, angle)
        return dot(T, Tl)

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE

                T = self.transform_matrix(T, joint, angle)

                self.transforms[joint] = T

            last_joint=chain_joints[-1]
            if last_joint == "LElbowRoll":
                print(last_joint+": ("+str(self.decompose_matrix(T))+")", end="; ")
                #print(self.transforms[last_joint])
        print("[********]")

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.keyframes = wipe_forehead(1)
    agent.run()
