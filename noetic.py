from sentry_serial.msg import navigation
import socket
import rospy
from std_msgs.msg import String

# 全局变量
yaw = 0.0
pitch = 0.0
x = 0.0
y = 0.0
z = 0.0
pub = None
client_socket = None

def tcp_server():
    global pub, client_socket
    host = '127.0.0.1'
    port = 51015  # 固定端口

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server_socket.bind((host, port))
    except OSError as e:
        rospy.logerr(f"Port {port} is occupied: {e}")
        raise
    server_socket.listen(5)
    rospy.loginfo(f"TCP Server started on {host}:{port}")

    rospy.init_node('noetic_tcp_server', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    navigation_sub = rospy.Subscriber('navigation', navigation, navigation_callback)

    while not rospy.is_shutdown():
        client_socket, addr = server_socket.accept()
        rospy.loginfo(f"Connected by {addr}")
        try:
            rate = rospy.Rate(50)
            while not rospy.is_shutdown():
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            client_socket.close()
            client_socket = None

def navigation_callback(data):
    global yaw, pitch, x, y, z, client_socket
    # 假设yaw/pitch/x/y/z是std_msgs/Float64类型
    yaw = data.yaw.data    # 关键修改：提取实际数值
    pitch = data.pitch.data
    x = data.x.data
    y = data.y.data
    z = data.z.data

    # 修改后的消息格式（去掉data层级）
    message = f"yaw: {yaw}, pitch: {pitch}, x: {x}, y: {y}, z: {z}, time: {rospy.get_time()}"
    rospy.loginfo(message)
    pub.publish(message)

    if client_socket:
        try:
            client_socket.send(message.encode())
        except socket.error as e:
            rospy.logerr(f"Send error: {e}")
            client_socket.close()
            client_socket = None

if __name__ == '__main__':
    try:
        tcp_server()
    except rospy.ROSInterruptException:
        pass