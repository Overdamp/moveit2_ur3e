#!/usr/bin/env python3
import socket
import time

# ตั้งค่า IP และ Port ของ UR
robot_ip = "192.168.20.35"
robot_port = 30002

# สร้าง TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(2.0)

try:
    sock.connect((robot_ip, robot_port))
    print("✅ Connected to UR robot")

    # เปิด gripper (set digital out 0 to True)
    open_cmd = "set_standard_digital_out(0, True)\n"
    sock.sendall(open_cmd.encode('utf-8'))
    print("🖐️ Gripper opened")
    time.sleep(2)

    # ปิด gripper (set digital out 0 to False)
    close_cmd = "set_standard_digital_out(0, False)\n"
    sock.sendall(close_cmd.encode('utf-8'))
    print("✊ Gripper closed")
    time.sleep(2)

except Exception as e:
    print(f"❌ Error: {e}")

finally:
    sock.close()
    print("❎ Disconnected")

