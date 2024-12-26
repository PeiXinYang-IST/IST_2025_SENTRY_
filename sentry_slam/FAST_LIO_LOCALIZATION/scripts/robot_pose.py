#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

def odom_callback(msg):
    # 这里我们从 Odometry 消息中提取位置和朝向信息
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    
    # 创建 tf 变换消息
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"  # 父坐标系是 "odom"
    t.child_frame_id = "robot"  # 子坐标系是 "robot"
    
    # 变换的位置信息
    t.transform.translation.x = position.x
    t.transform.translation.y = position.y
    t.transform.translation.z = position.z

    # 变换的旋转信息（四元数）
    t.transform.rotation.w = 1.0
    
    # 发布 tf 变换
    tf_broadcaster.sendTransform(t)

def main():
    # 初始化 ROS 节点
    rospy.init_node('odom_to_tf_broadcaster')
    
    # 创建 TF 广播器
    global tf_broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    # 订阅 odom 话题
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # 持续运行 ROS 节点
    rospy.spin()

if __name__ == '__main__':
    main()
