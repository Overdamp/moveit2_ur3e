#!/usr/bin/env python3
import os
import socket
import time
import ament_index_python.packages

# Network configuration
HOST = "192.168.20.36"  # UR3e IP in LAN
PORT = 30002  # Socket port for UR-script

# Get the path to the package's share directory
package_share_directory = ament_index_python.packages.get_package_share_directory('robotiq_connect')

# Build paths to UR-script files
script_path1 = os.path.join(package_share_directory, 'scripts', 'gripper_activate.script')
script_path2 = os.path.join(package_share_directory, 'scripts', 'gripper_close.script')
script_path3 = os.path.join(package_share_directory, 'scripts', 'gripper_open.script')

class GripperControl:
    def __init__(self):
        # Initialize socket connection
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((HOST, PORT))
            print("Socket connected for gripper control")
            self._activate_gripper()
        except Exception as e:
            print(f"Failed to connect socket for gripper: {e}")
            raise

    def _activate_gripper(self):
        """Activate gripper using UR-script"""
        try:
            with open(script_path1, "rb") as f:
                script = f.read()
                self.socket.send(script)
            print("Gripper activated")
            time.sleep(1)
        except Exception as e:
            print(f"Gripper activation failed: {e}")

    def close_gripper(self, force=50, speed=50, range=200):
        """Close gripper with specified parameters"""
        try:
            with open(script_path2, "rb") as f:
                script = f.read()
                add = f"rq_set_force({force})\r\nrq_set_speed({speed})\r\nrq_move_and_wait({range})\r\nend\r\n"
                send = script + bytes(add, 'utf-8')
                self.socket.send(send)
            print("Gripper closed")
            time.sleep(0.5)
        except Exception as e:
            print(f"Gripper close failed: {e}")

    def open_gripper(self):
        """Open gripper using UR-script"""
        try:
            with open(script_path3, "rb") as f:
                script = f.read()
                self.socket.send(script)
            print("Gripper opened")
            time.sleep(0.5)
        except Exception as e:
            print(f"Gripper open failed: {e}")

    def __del__(self):
        """Cleanup socket connection on destruction"""
        try:
            self.socket.close()
            print("Socket connection closed")
        except Exception as e:
            print(f"Error closing socket: {e}")

def main():
    try:
        gripper = GripperControl()
        print("Testing gripper activation...")
        gripper._activate_gripper()  # ทดสอบการ activate gripper
        print("Testing gripper open...")
        gripper.open_gripper()  # ทดสอบการเปิด gripper
        time.sleep(2)
        print("Testing gripper close...")
        gripper.close_gripper()  # ทดสอบการปิด gripper
    except Exception as e:
        print(f"Error in main: {e}")

if __name__ == '__main__':
        main()