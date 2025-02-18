from sentry_serial.msg import navigation
import socket
import rospy
from std_msgs.msg import String

# 定义全局变量
yaw = 0.0
x = 0.0
y = 0.0
z = 0.0
pub = None
client_socket = None

def tcp_server():
    global pub, client_socket
    host = '127.0.0.1'
    port = 51010
    max_port = 65535
    server_socket = None

    # 自动寻找可用端口
    while port <= max_port:
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((host, port))
            break
        except OSError as e:
            if e.errno == 98:
                print(f"Port {port} is occupied, trying {port + 1}")
                port += 1
                if server_socket:
                    server_socket.close()
            else:
                raise
    else:
        raise OSError(f"No available ports between {port} and {max_port}")

    print(f"Successfully bound to port {port}")
    server_socket.listen(5)

    rospy.init_node('noetic_tcp_server', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(50)

    navigation_sub = rospy.Subscriber('navigation', navigation, navigation_callback)

    while not rospy.is_shutdown():
        client_socket, addr = server_socket.accept()
        print(f"Connection from: {addr}")
        
        try:
            while True:
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            if client_socket:
                client_socket.close()
                client_socket = None

def navigation_callback(data):
    global yaw, x, y, z, pub, client_socket
    yaw = data.yaw.data
    x = data.x.data
    y = data.y.data
    z = data.z.data

    message = f"yaw: {yaw}, x: {x}, y: {y}, z: {z}, time: {rospy.get_time()}"
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
    except OSError as e:
        print(f"Critical error: {e}")