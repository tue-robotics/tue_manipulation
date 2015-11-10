#!/usr/bin/python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

rospy.init_node('visualize_joint_trajectory_action_goal')

def callback(goal):	
	t = [p.time_from_start.to_sec() for p in goal.goal.trajectory.points]
	joints = {name : {"positions" : [], "velocities" : [], "accelerations" : []} for name in goal.goal.trajectory.joint_names}
	for point in goal.goal.trajectory.points:
		for index, joint_point in enumerate(point.positions):
			joints[goal.goal.trajectory.joint_names[index]]["positions"].append(joint_point)
		for index, joint_point in enumerate(point.velocities):
			joints[goal.goal.trajectory.joint_names[index]]["velocities"].append(joint_point)
		for index, joint_point in enumerate(point.accelerations):
			joints[goal.goal.trajectory.joint_names[index]]["accelerations"].append(joint_point)

	i = 1
	pp = PdfPages('/tmp/plots.pdf')
	for name, points in joints.iteritems():

		plt.figure(i)

		plt.subplot("311")
		plt.plot(t, points["positions"])
		plt.ylabel("positions")

		plt.title(name)

		plt.subplot("312")
		plt.plot(t, points["velocities"])
		plt.ylabel("velocities")

		plt.subplot("313")
		plt.plot(t, points["accelerations"])
		plt.ylabel("accelerations")

		i+=1

		plt.savefig(pp, format='pdf')

	plt.draw()
	pp.close()

plt.ion()

rospy.Subscriber("goal", FollowJointTrajectoryActionGoal, callback, queue_size=10)
rospy.spin()