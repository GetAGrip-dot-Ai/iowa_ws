#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
import tf
import math


def aprilarg_reader():
	point_pub = rospy.Publisher('/scrapper/patch_cmd', PointStamped, queue_size=1)
	rospy.init_node('debug_april_tag', anonymous=True)

	listener = tf.TransformListener()
	static_tf = np.array([[0, 0.5, math.sqrt(3)/2, 0.135],[-1, 0, 0, 0.035],[0, -math.sqrt(3)/2, 0.5, 0.1],[0, 0, 0,1 ]])

	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/oak_left_camera_optical_frame', '/tag_1', rospy.Time(0))
			trans.append(1)
			new_points = 1000.0*np.matmul(static_tf, np.array(trans).T)
			# print(new_points)
			point_msg = PointStamped()
			point_msg.point.x = new_points[0]
			point_msg.point.y = new_points[1]
			point_msg.point.z = new_points[2]
			point_msg.header.stamp = rospy.Time.now()
			point_msg.header.frame_id = 'base_link'
			point_pub.publish(point_msg)


		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		rate.sleep()



if __name__ == '__main__':
	try:
		aprilarg_reader()
	except rospy.ROSInterruptException:
		pass