'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import sys
import os
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

import weakref
import requests, json
from keyframes import hello
import math

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.url = "http://localhost:4000/jsonrpc"
        self.headers = {'content-type': 'application/json'}

    def echo(self, t):
        payload = {
            "method": "echo",
            "params": [t],
            "jsonrpc": "2.0",
            "id": 0,
        }
        response = requests.post(self.url, data=json.dumps(payload), headers=self.headers).json()
        print(response[u'result'])

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        payload = {
            "method": "get_angle",
            "params": [joint_name],
            "jsonrpc": "2.0",
            "id": 0,
        }
        response = requests.post(self.url, data=json.dumps(payload), headers=self.headers).json()
        return response[u'result']

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        payload = {
            "method": "set_angle",
            "params": [joint_name, angle],
            "jsonrpc": "2.0",
            "id": 0,
        }
        response = requests.post(self.url, data=json.dumps(payload), headers=self.headers).json()
        return response[u'result']

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        payload = {
            "method": "get_posture",
            "params": [],
            "jsonrpc": "2.0",
            "id": 0,
        }
        response = requests.post(self.url, data=json.dumps(payload), headers=self.headers).json()
        return response[u'result']

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        payload = {
            "method": "execute_keyframes",
            "params": [json.dumps(keyframes)],
            "jsonrpc": "2.0",
            "id": 0,
        }
        response = requests.post(self.url, data=json.dumps(payload), headers=self.headers).json()
        return response[u'result']

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
    agent = ClientAgent()
    # TEST CODE HERE
    agent.echo("Hello Server!")
    print "Get angle of LElbowYaw: ", agent.get_angle("LElbowYaw")
    print "Set angle of LElbowYaw to 30 - ", agent.set_angle("LElbowYaw", math.radians(-30))
    print "Posture - ", agent.get_posture()
    print "Keyframes (hello) - ", agent.execute_keyframes(hello())
