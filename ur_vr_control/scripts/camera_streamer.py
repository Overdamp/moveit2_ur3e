#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import struct
import time
import threading

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        # ประกาศพารามิเตอร์
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('vr_host', '127.0.0.1')
        self.declare_parameter('vr_port', 5556)
        self.declare_parameter('quality', 50)
        self.declare_parameter('frame_rate', 30.0)  # อัตราเฟรมต่อวินาที
        self.declare_parameter('image_width', 640)  # ความกว้างของภาพ
        self.declare_parameter('image_height', 480)  # ความสูงของภาพ

        # อ่านพารามิเตอร์
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.vr_host = self.get_parameter('vr_host').get_parameter_value().string_value
        self.vr_port = self.get_parameter('vr_port').get_parameter_value().integer_value
        self.quality = self.get_parameter('quality').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value

        # ตั้งค่า QoS สำหรับ subscriber
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # สร้าง subscriber
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos
        )
        self.br = CvBridge()
        self.sock = None
        self.last_frame_time = 0.0
        self.frame_interval = 1.0 / self.frame_rate
        self.lock = threading.Lock()  # สำหรับจัดการ thread-safe
        self.is_running = True

        # เริ่ม thread สำหรับจัดการการเชื่อมต่อ
        self.connection_thread = threading.Thread(target=self.connect_to_vr, daemon=True)
        self.connection_thread.start()

    def connect_to_vr(self):
        """จัดการการเชื่อมต่อกับ Unity server"""
        while self.is_running and rclpy.ok():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)  # ตั้ง timeout การเชื่อมต่อ
                self.sock.connect((self.vr_host, self.vr_port))
                self.get_logger().info(f'Connected to VR server at {self.vr_host}:{self.vr_port}')
                break
            except socket.timeout:
                self.get_logger().error(f'Connection timeout to VR server at {self.vr_host}:{self.vr_port}')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to VR server: {e}')
            finally:
                if self.sock is None or not self.is_socket_connected():
                    time.sleep(1.0)  # รอ 1 วินาทีก่อนลองใหม่

    def image_callback(self, msg):
        """Callback เมื่อได้รับภาพจาก topic"""
        if not self.is_running:
            return

        current_time = time.time()
        if current_time - self.last_frame_time < self.frame_interval:
            return  # ข้ามหากยังไม่ถึงเวลาเฟรมถัดไป

        try:
            with self.lock:
                # แปลง ROS Image message เป็น OpenCV image
                cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                # ปรับขนาดภาพ
                if self.image_width > 0 and self.image_height > 0:
                    cv_image = cv2.resize(cv_image, (self.image_width, self.image_height))

                # แปลงภาพเป็น JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), max(0, min(self.quality, 100))]
                success, buffer = cv2.imencode('.jpg', cv_image, encode_param)
                if not success:
                    self.get_logger().error('Failed to encode image to JPEG')
                    return

                frame_size = len(buffer)
                if frame_size > 1024 * 1024 * 10:  # จำกัดขนาดสูงสุด 10MB
                    self.get_logger().error(f'Frame size too large: {frame_size} bytes')
                    return

                if self.sock is None or not self.is_socket_connected():
                    self.get_logger().warn('No connection to VR server. Attempting to reconnect...')
                    self.connect_to_vr()

                try:
                    # ส่งขนาดของเฟรม (4 bytes)
                    self.sock.sendall(struct.pack("!I", frame_size))
                    # ส่งข้อมูลภาพ
                    self.sock.sendall(buffer)
                    self.get_logger().info(f'Sent frame of size {frame_size} bytes')
                    self.last_frame_time = current_time
                except Exception as e:
                    self.get_logger().error(f'Error sending frame: {e}')
                    self.close_socket()
                    self.connect_to_vr()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def is_socket_connected(self):
        """ตรวจสอบว่า socket ยังเชื่อมต่ออยู่หรือไม่"""
        try:
            if self.sock is None:
                return False
            self.sock.setblocking(0)
            data = self.sock.recv(1, socket.MSG_PEEK)
            self.sock.setblocking(1)
            return True
        except:
            return False

    def close_socket(self):
        """ปิด socket อย่างปลอดภัย"""
        with self.lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None

    def destroy_node(self):
        """ทำความสะอาดเมื่อปิด node"""
        self.is_running = False
        self.close_socket()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()