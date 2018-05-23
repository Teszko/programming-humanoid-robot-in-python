'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import wipe_forehead
import pickle
from os import listdir, path
import numpy as np
from sklearn import svm, metrics

ROBOT_POSE_DATA_DIR = './robot_pose_data/'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
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
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        pose = [perception.joint['LHipYawPitch'], perception.joint['LHipRoll'], perception.joint['LHipPitch'], perception.joint['LKneePitch'],
                perception.joint['RHipYawPitch'], perception.joint['RHipRoll'], perception.joint['RHipPitch'], perception.joint['RKneePitch'],
                perception.imu[0], perception.imu[1]]
        pose = np.array(pose).reshape((1, -1))
        posture = self.classes[self.posture_classifier(pose)[0]]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
