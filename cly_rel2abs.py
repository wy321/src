import numpy as np

def cly_rel2abs(robot_abs,cly_dis):
	theta = robot_abs[2]
	z = cly_dis[2]
	x = cly_dis[1]
	dx = z*np.sin(theta) + x*np.cos(theta)
	dy = z*np.cos(theta) - x*np.sin(theta)
	cly_abs = np.array([cly_dis[0], robot_abs[0] + dx, robot_abs[1] + dy])
	return cly_abs
