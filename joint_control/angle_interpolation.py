'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import wipe_forehead

import numpy as np

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
	self.start_time = None
	self.__end_time = 0
	self.animation_in_progress = False


    def __bezier(self, z, p):
        bz = np.multiply(np.power((1 - z), 3), p[0]) + \
             np.multiply(3 * z * np.power((1 - z), 2), p[1]) + \
             np.multiply(3 * (1 - z) * z * z, p[2]) + \
             np.multiply(z * z * z, p[3])
        return bz


    def start_keyframe_animation(self, keyframes):
        self.keyframes = keyframes
        self.__end_time = 0
        self.animation_in_progress = True

        for i in range(0, len(self.keyframes[1])):
            t1 = self.keyframes[1][i][-1]
            if t1 > self.__end_time:
                self.__end_time = t1

        self.start_time = None


    def __bezier_binary_search(self, a, b, t, p):
        val = (a + b) / 2

        bvec = self.__bezier(val, p)

        if abs(bvec[0] - t) < 0.0001:
            return bvec

        else:
            if bvec[0] < t:
                return self.__bezier_binary_search(val, b, t, p)
            else:
                return self.__bezier_binary_search(a, val, t, p)


    def bezier_time_interpolation(self, t, p):
        return self.__bezier_binary_search(0.0, 1.0, t, p)


    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
	if self.start_time is None:
            self.start_time = perception.time
            animation_time = 0
        else:
            animation_time = perception.time - self.start_time

        if animation_time > self.__end_time:
            self.animation_in_progress = False
        else:
            self.animation_in_progress = True
	
        joint_names = keyframes[0]
        times = keyframes[1]
        keys = keyframes[2]
        num_joints = len(joint_names)

        for j in range(0, num_joints):
            joint_name = joint_names[j]
            number_of_keyframes = len(keys[j])  # initial position is considered first keyframe.
            joint_times = times[j]

            left_frame = 0
            right_frame = 0
            for f in range(0, number_of_keyframes):
                if animation_time > joint_times[f]:
                    left_frame = f
                    right_frame = f+1

            if right_frame >= number_of_keyframes:
                right_frame = left_frame

            left_frame_key = keys[j][left_frame]
            right_frame_key = keys[j][right_frame]

            t1 = joint_times[left_frame]
            t2 = joint_times[right_frame]
            if (t2 - t1) > 0:
                time_factor = (animation_time - t1) / (t2 - t1)
            else:
                time_factor = 0

            value = self.bezier_time_interpolation(time_factor, [[0, left_frame_key[0]],
                                                                 [0 + left_frame_key[2][1], left_frame_key[0] + left_frame_key[2][2]],
                                                                 [1 + right_frame_key[1][1], right_frame_key[0] + right_frame_key[1][2]],
								 [1, right_frame_key[0]]])

            target_joints[joint_name] = value[1]


        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = wipe_forehead(1)
    agent.run()
