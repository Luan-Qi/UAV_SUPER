#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import tf

class TfToOdomConverter(object):
    def __init__(self):

        # 想要转换的TF (根据你cartographer的配置修改)
        self.source_frame = "map"
        self.child_frame = "base_link"

        # 发布 odom
        self.odom_pub = rospy.Publisher("/global_Odometry", Odometry, queue_size=10)

        rospy.Subscriber("/tf", TFMessage, self.tf_callback)

        # rospy.loginfo("TF → Odometry 转换节点已启动")

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == self.source_frame and \
               transform.child_frame_id == self.child_frame:

                odom_msg = Odometry()
                odom_msg.header.stamp = transform.header.stamp
                odom_msg.header.frame_id = self.source_frame
                odom_msg.child_frame_id = self.child_frame

                # 位置
                odom_msg.pose.pose.position = transform.transform.translation

                # 姿态
                odom_msg.pose.pose.orientation = transform.transform.rotation

                # 速度未知，填 0
                odom_msg.twist.twist.linear.x = 0.0
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = 0.0

                self.odom_pub.publish(odom_msg)

if __name__ == "__main__":
    rospy.init_node("tf_to_odom")
    TfToOdomConverter()
    rospy.spin()
