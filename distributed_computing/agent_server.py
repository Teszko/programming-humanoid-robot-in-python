'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

from werkzeug.wrappers import Request, Response
from werkzeug.serving import run_simple
from jsonrpc import JSONRPCResponseManager, dispatcher

import threading
import numpy as np
import pickle
from sklearn import svm, metrics
from os import listdir, path
import json
import time

ROBOT_POSE_DATA_DIR = '../joint_control/robot_pose_data/'

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, simspark_ip='127.0.0.1',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ServerAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)

        self.posture = 'unknown'
        self.posture_classifier = None  # LOAD YOUR CLASSIFIER
        self.classes = listdir(ROBOT_POSE_DATA_DIR)

        def load_pose_data(i):
            '''load pose data from file'''
            data = []
            target = []
            filename = path.join(ROBOT_POSE_DATA_DIR, self.classes[i])
            data = pickle.load(open(filename))
            target = [i] * len(data)
            return data, target

        all_data = []
        all_target = []

        for i in range(0, len(self.classes)):
            data, target = load_pose_data(i)
            all_data = all_data + data
            all_target = all_target + target

        all_data = np.array(all_data)
        all_target = np.array(all_target)
        clf = svm.SVC(gamma=0.001, C=100.)
        clf.fit(all_data, all_target)

        self.posture_classifier = clf.predict

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(ServerAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        pose = [perception.joint['LHipYawPitch'], perception.joint['LHipRoll'], perception.joint['LHipPitch'], perception.joint['LKneePitch'],
                perception.joint['RHipYawPitch'], perception.joint['RHipRoll'], perception.joint['RHipPitch'], perception.joint['RKneePitch'],
                perception.imu[0], perception.imu[1]]
        pose = np.array(pose).reshape((1, -1))
        posture = self.classes[self.posture_classifier(pose)[0]]

        return posture

    def werkzeug_init(self):
        run_simple('localhost', 4000, self.application, threaded=True)

    @Request.application
    def application(self, request):
        dispatcher["echo"] = lambda s: self.echo(s)
        dispatcher["get_angle"] = lambda jn: self.get_angle(jn)
        dispatcher["set_angle"] = lambda jn, a: self.set_angle(jn, a)
        dispatcher["get_posture"] = lambda: self.get_posture()
        dispatcher["execute_keyframes"] = lambda kf: self.execute_keyframes(kf)

        response = JSONRPCResponseManager.handle(request.data, dispatcher)
        return Response(response.json, mimetype='application/json')

    def echo(self, e):
        print(e)
        return e

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        angle = self.perception.joint[joint_name]
        return angle
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle
        return angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.start_keyframe_animation(json.loads(keyframes))

        while self.animation_in_progress:
            time.sleep(0.2)

        return "okay"

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return None

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return None

if __name__ == '__main__':
    agent = ServerAgent()
    print("run_simple()")
    threading.Thread(target=agent.werkzeug_init).start()
    print("agent.run()")
    agent.run()

