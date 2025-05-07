import time
import numpy as np
from collections import deque

from teleop.utils import rotvec_to_quat, quat_to_rotvec
from teleop.utils import interpolate_quat, quat_multiply, quat_conjugate


class VRTracker:
    def __init__(
        self,
        rtde,
        gripper,
        rate=0.008,
        smooth_step=0.1,  # Smoothing step size, smaller is smoother
        pos_mapping=(1, 1, 1),  # x, y, z scaling
    ):
        self.rtde = rtde
        self.gripper = gripper
        self.rate = rate
        self.paused = True
        self.gripper_open = True

        self.input_anchor = None  # [x, y, z, *quat]
        self.tool_anchor = None  # [x, y, z, *quat]
        self.target = None # [x, y, z, *quat]
        self.smooth_step = smooth_step

        self.pos_mapping = np.array(pos_mapping)

    def resume(self, input_anchor):
        # Assume input is [x, y, z, *quat]
        self.input_anchor = np.array(input_anchor)
        # Tool pose is [x, y, z, *quat]
        tool_anchor = self.rtde.get_tool_pose()
        self.tool_anchor = np.zeros(7)
        self.tool_anchor[:3] = tool_anchor[:3]
        self.tool_anchor[3:] = rotvec_to_quat(tool_anchor[3:])

        self.target = np.copy(self.tool_anchor)
        self.paused = False

    def pause(self):
        self.paused = True

    def track(self, user_input):
        # Assume input is [x, y, z, *quat]
        if self.paused:
            return
        user_input = np.array(user_input)

        # The relative input value is the difference 
        # between the user input and the anchor
        rel_translation = user_input[:3] - self.input_anchor[:3]
        rel_quat = quat_multiply(
            user_input[3:], quat_conjugate(self.input_anchor[3:])
        )

        # Scale the relative translation
        rel_translation = rel_translation * self.pos_mapping

        # The actual required tool pose is the sum of the anchor and the input
        global_translation = self.tool_anchor[:3] + rel_translation
        global_quat = quat_multiply(rel_quat, self.tool_anchor[3:])

        # Smooth the target by interpolating 
        # between the current target and the new target
        self.target[:3] += (
            self.smooth_step * (global_translation - self.target[:3])
        )
        self.target[3:] = interpolate_quat(
            self.target[3:], global_quat, self.smooth_step
        )

        # Send the smoothed target [x, y, z, *rotvec] to the robot
        target = np.zeros(6)
        target[:3] = self.target[:3]
        target[3:] = quat_to_rotvec(self.target[3:])
        self.rtde.servo_tool(target, time=self.rate)

        return target

    def trigger_gripper(self):
        if self.gripper_open:
            self.gripper.close_gripper()
        else:
            self.gripper.open_gripper()
        self.gripper_open = not self.gripper_open
