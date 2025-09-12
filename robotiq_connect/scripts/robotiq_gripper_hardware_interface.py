import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

from controller_interface import ControllerInterface
from hardware_interface import (
    HardwareInfo,
    ActuatorInterface,
    StateInterface,
    CommandInterface,
    hardware_interface,
)
from hardware_interface.components import (
    Actuator,
    ComponentInterfaceReturnType,
)
import socket
import time


class RobotiqGripperHardware(Actuator):
    def __init__(self, info: HardwareInfo):
        super().__init__(info)
        self.hw_start_sec = 1.0
        self.hw_stop_sec = 1.0
        self.hw_position = 0.0  # current gripper width
        self.hw_command = 0.0   # command from controller
        self.gripper_socket = None

    def configure(self):
        try:
            self.gripper_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.gripper_socket.connect(('192.168.20.35', 30002))  # IP ของ UR3e/Gripper
            print("[Gripper] Socket connected")
            return ComponentInterfaceReturnType.OK
        except Exception as e:
            print(f"[Gripper] Connection failed: {e}")
            return ComponentInterfaceReturnType.ERROR

    def start(self):
        print("[Gripper] Hardware interface started")
        return ComponentInterfaceReturnType.OK

    def stop(self):
        print("[Gripper] Hardware interface stopped")
        return ComponentInterfaceReturnType.OK

    def read(self, time: Time, period: Duration):
        # Simulate reading current state
        self.hw_position = self.hw_command
        return ComponentInterfaceReturnType.OK

    def write(self, time: Time, period: Duration):
        # Clamp command between 0.0 (open) and 0.8 (close)
        width = max(0.0, min(0.8, self.hw_command))
        close_level = int((1.0 - width / 0.8) * 255)

        try:
            cmd = f"""
                    def gripper_close():
                    rq_set_force(50)
                    rq_set_speed(50)
                    rq_move_and_wait({close_level})
                    end
                    gripper_close()
                    """
            self.gripper_socket.send(cmd.encode('utf-8'))
        except Exception as e:
            print(f"[Gripper] Write error: {e}")
        return ComponentInterfaceReturnType.OK

    def export_state_interfaces(self):
        return [StateInterface("gripper_finger_joint", "position", lambda: self.hw_position)]

    def export_command_interfaces(self):
        return [CommandInterface("gripper_finger_joint", "position", lambda: self.hw_command, lambda v: setattr(self, 'hw_command', v))]


def create_actuator(info: HardwareInfo):
    return RobotiqGripperHardware(info)
