import socket
import time
import logging
import rospy
from std_msgs.msg import UInt8

class PosStateTCPClient:
    def __init__(self, host='127.0.0.1', port=52000):
        self.host = host
        self.port = port
        self.sock = None
        self.exit_flag = False
        self.logger = self._setup_logger()
        self.publisher = None  # ROS1 publisher

    def _setup_logger(self):
        """配置日志系统"""
        logger = logging.getLogger('TCP_Client')
        logger.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        logger.addHandler(ch)
        return logger

    def _connect(self):
        """建立TCP连接（带详细错误处理）"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(1)
            return True
        except Exception as e:
            self.logger.error(f"连接失败: {str(e)}")
            return False

    def _reconnect_loop(self):
        """带退出检查的自动重连循环"""
        while not self.exit_flag and not rospy.is_shutdown():
            if self._connect():
                self.logger.info(f"成功连接到 {self.host}:{self.port}")
                return True
            self.logger.info("等待重连...")
            for _ in range(5):  # 将5秒拆分为5次1秒等待
                time.sleep(1)
                if self.exit_flag or rospy.is_shutdown():
                    return False
        return False

    def _receive_loop(self):
        """带退出检查的数据接收循环"""
        try:
            while not self.exit_flag and not rospy.is_shutdown():
                try:
                    data = self.sock.recv(1024)
                    if not data:
                        self.logger.warning("连接被服务器关闭")
                        break
                    value = int.from_bytes(data, byteorder='big')
                    self.logger.info(f"收到数据: {value}")
                    if self.publisher:
                        self.publisher.publish(value)
                except socket.timeout:
                    continue
                except Exception as e:
                    self.logger.error(f"接收错误: {str(e)}")
                    break
        finally:
            if self.sock:
                self.sock.close()

    def run(self):
        """主运行逻辑"""
        rospy.init_node('tcp_client_node', anonymous=True)
        self.publisher = rospy.Publisher('/ros2_game_state', UInt8, queue_size=10)
        
        try:
            while not self.exit_flag and not rospy.is_shutdown():
                if self._reconnect_loop():
                    self._receive_loop()
        finally:
            self.shutdown()

    def shutdown(self):
        """安全关闭"""
        self.exit_flag = True
        if self.sock:
            self.sock.close()
        self.logger.info("客户端已安全关闭")

if __name__ == '__main__':
    client = PosStateTCPClient()
    try:
        client.run()
    except KeyboardInterrupt:
        client.shutdown()