#!/usr/bin/env python

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf 

def check_motion(path, pose):
	# return True
	if len(path.poses) == 0:
		return True
	last_pose = path.poses[-1]
	dx = pose.pose.position.x - last_pose.pose.position.x
	dy = pose.pose.position.y - last_pose.pose.position.y
	dz = pose.pose.position.z - last_pose.pose.position.z

	dist = np.sqrt(dx*dx + dy*dy + dz*dz)

	if dist > 0.0001:
		return True
	else:
		return False

def main():
	rospy.init_node('tooltip_pose_publisher', anonymous=True)
	rate = rospy.Rate(30.0)
	path_pub = rospy.Publisher('/end_effector_path', Path, queue_size=1)
	goal_path_pub = rospy.Publisher('/goal_path', Path, queue_size=1)
	time_now = rospy.Time.now()
	goal_path = Path()
	goal_path.header.frame_id = 'base_link'
	goal_points = [[0.6,0.,0.8], [0.6,-0.3,0.4], [0.6,0.,0.], [0.6,0.3,0.4], [0.6,0.,0.8]]
	goal_path.header.stamp = time_now
	for point in goal_points:
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'base_link'
		goal_pose.header.stamp = time_now
		goal_pose.pose.position.x = point[0]
		goal_pose.pose.position.y = point[1]
		goal_pose.pose.position.z = point[2]
		goal_pose.pose.orientation.x = 0
		goal_pose.pose.orientation.y = 0
		goal_pose.pose.orientation.z = 0
		goal_pose.pose.orientation.w = 1
		goal_path.poses.append(goal_pose)
	
	path = Path()
	path.header.frame_id = 'base_link'
	listener = tf.TransformListener()

	while not rospy.is_shutdown():
		try:
			position, orientation = listener.lookupTransform('base_link', 'link_6', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		time_now = rospy.Time.now()
		new_pose = PoseStamped()

		new_pose.header.frame_id = 'base_link'
		new_pose.header.stamp = time_now
		new_pose.pose.position.x = position[0]
		new_pose.pose.position.y = position[1]
		new_pose.pose.position.z = position[2]
		new_pose.pose.orientation.x = orientation[0]
		new_pose.pose.orientation.y = orientation[1]
		new_pose.pose.orientation.z = orientation[2]
		new_pose.pose.orientation.w = orientation[3]
		if check_motion(path, new_pose):
			path.poses.append(new_pose)
		path.header.stamp = time_now
		# print(len(path.poses))

		path_pub.publish(path)
		goal_path_pub.publish(goal_path)
		rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass


