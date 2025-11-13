#!/usr/bin/env python3
import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def callback(msg):
    points = []
    for p in msg.points:
        points.append([p.x, p.y, p.z, float(p.reflectivity)])

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]

    cloud_msg = pc2.create_cloud(msg.header, fields, points)
    cloud_msg.header.frame_id = "lidar_link"
    pub.publish(cloud_msg)

if __name__ == "__main__":
    rospy.init_node("livox_to_pointcloud2")
    pub = rospy.Publisher("/livox/lidar_cloud", PointCloud2, queue_size=10)
    rospy.Subscriber("/livox/lidar", CustomMsg, callback, queue_size=10)
    rospy.spin()
