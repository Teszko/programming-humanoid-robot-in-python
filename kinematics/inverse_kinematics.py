'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity

'''
theta = random.random(N)
lambda_ = 1
max_step = 0.1
def inverse_kinematics_fork(x_e, y_e, z_e, theta_x, theta_y, theta_z, theta):
    target = matrix([[x_e, y_e, z_e, theta_x, theta_y, theta_z]]).T
    for i in range(10):
        Ts = forward_kinematics(T0, l, theta)
        Te = matrix([from_trans(Ts[-1])]).T
        e = target - Te
        e[e > max_step] = max_step
        e[e < -max_step] = -max_step
        T = matrix([from_trans(i) for i in Ts[1:-1]]).T
        J = Te - T
        dT = Te - T
        J[0, :] = -dT[1, :] # x
        J[1, :] = dT[0, :] # y
        J[-1, :] = 1  # angular
        d_theta = lambda_ * pinv(J) * e
        theta += asarray(d_theta.T)[0]
        if  linalg.norm(d_theta) < 1e-4:
            break
    return theta
    '''

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        joint_angles = []
        # YOUR CODE HERE

        max_step = 0.1
        lambda_ = 1

        chain = self.chains[effector_name]
        last_element_name = chain[-1]

        target = ForwardKinematicsAgent.decompose_matrix(transform)

        Ts = self.transforms[last_element_name]
        Te = ForwardKinematicsAgent.decompose_matrix(Ts)

        e = target - Te
        e[e > max_step] = max_step
        e[e < -max_step] = -max_step
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
