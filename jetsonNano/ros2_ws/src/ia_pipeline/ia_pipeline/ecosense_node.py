import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import struct
import json
import time

class EcoSenseNode(Node):
    def __init__(self):
        super().__init__('ecosense_detector')

        # --- PARAMETERS ---
        self.declare_parameter('server_ip', '127.0.0.1') 
        self.declare_parameter('server_port', 55001)
        
        self.server_ip = self.get_parameter('server_ip').value
        self.server_port = self.get_parameter('server_port').value

        self.bridge = CvBridge()
        
        # --- TCP CLIENT SETUP ---
        self.sock = None
        self.connect_to_server()

        # --- ROS ---
        self.sub = self.create_subscription(Image, '/usb_cam_right/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/usb_cam_right/image_processed', 10)
        self.get_logger().info(f"Node started. Target: {self.server_ip}:{self.server_port}")

    def connect_to_server(self):
        """Persistent connection to save handshake time"""
        try:
            if self.sock: self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.server_ip, self.server_port))
            self.get_logger().info("Connected to GPU Server.")
        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")
            self.sock = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Infer
        detections = self.infer_remote(cv_image)
        
        # Draw & Publish
        if detections is not None:
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                label = f"{det['class_name']} {det['score']:.2f}"
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.pub.publish(out_msg)

    def infer_remote(self, img):
        if self.sock is None:
            self.connect_to_server()
            if self.sock is None: return []

        try:
            # 1. Encode Image (JPG is faster over network than raw)
            _, img_encoded = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            data = img_encoded.tobytes()

            # 2. Send: Length (4 bytes) + Image Data
            self.sock.sendall(struct.pack('>I', len(data)) + data)

            # 3. Receive: Length (4 bytes)
            raw_len = self.recvall(4)
            if not raw_len: raise ConnectionError("Server closed")
            resp_len = struct.unpack('>I', raw_len)[0]

            # 4. Receive: JSON Data
            resp_data = self.recvall(resp_len)
            response = json.loads(resp_data.decode('utf-8'))
            
            return response.get('detections', [])

        except (BrokenPipeError, ConnectionResetError, ConnectionError) as e:
            self.get_logger().warn(f"Lost connection: {e}. Reconnecting...")
            self.connect_to_server()
            return []
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return []

    def recvall(self, n):
        data = bytearray()
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet: return None
            data.extend(packet)
        return data

def main(args=None):
    rclpy.init(args=args)
    node = EcoSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
