#!/usr/bin/env python
# ros1_bridge_server.py
import rospy
import socket
import threading
from std_msgs.msg import Int32

class ROS1Bridge:
    def __init__(self):
        rospy.init_node('ros1_tcp_bridge')
        
        # 仅保留TCP->ROS1的发布器
        self.ros1_pub = rospy.Publisher('/ros1_game_state', Int32, queue_size=10)
        
        # 移除ROS1订阅器，避免反向通信
        # 之前的错误点：rospy.Subscriber('/ros1_game_state', Int32, self.ros1_callback)
        
        # TCP服务器配置
        self.host = '127.0.0.1'
        self.port = 52000
        self.sock = None
        self.conn = None
        
        # 启动TCP服务器线程
        self.server_thread = threading.Thread(target=self.tcp_server)
        self.server_thread.start()
        
        # 连接状态标志（现仅用于接收状态）
        self.is_connected = False
        
        rospy.on_shutdown(self.shutdown)

    def tcp_server(self):
        """TCP服务器主循环（仅处理TCP->ROS1）"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        rospy.loginfo(f"🚀 ROS1 TCP Server started on {self.host}:{self.port}")
        
        while not rospy.is_shutdown():
            self.conn, addr = self.sock.accept()
            self.is_connected = True
            rospy.loginfo(f"✅ Connected by {addr}")
            
            try:
                while not rospy.is_shutdown():
                    data = self.conn.recv(4)
                    if len(data) == 4:
                        value = int.from_bytes(data, byteorder='big')
                        # 发布到ROS1话题（单向）
                        msg = Int32()
                        msg.data = value
                        self.ros1_pub.publish(msg)
                        rospy.loginfo(f"📥 From TCP: {value}")
            except Exception as e:
                rospy.logerr(f"TCP error: {str(e)}")
            finally:
                self.conn.close()
                self.is_connected = False

    def shutdown(self):
        if self.conn:
            self.conn.close()
        if self.sock:
            self.sock.close()
        rospy.loginfo("🛑 ROS1 bridge shutdown")

if __name__ == '__main__':
    bridge = ROS1Bridge()
    rospy.spin()