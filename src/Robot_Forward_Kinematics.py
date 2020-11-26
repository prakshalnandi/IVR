import numpy as np

def CalculateFK(angles):
	theta1 = angles[0]
	theta2 = angles[1]
	theta3 = angles[2]
	theta4 = angles[3]

	c1 = np.cos(theta1)
	c2 = np.cos(theta2)
	c3 = np.cos(theta3)
	c4 = np.cos(theta4)
	s1 = np.sin(theta1)
	s2 = np.sin(theta2)
	s3 = np.sin(theta3)
	s4 = np.sin(theta4)

	X = 3.5 * s1 * s2 * c3 + 3 * c2 * s1 * s4 + 3 * c4 * (s1 * s2 * c3 + c1 * s3) + 3.5 * c1 * s3
	Y = 3.5 * s1 * s3 - 3.5 * s2 * c3 * c1 + 3 * c4 * (s1 * s3 - s2 * c3 * c1) - 3 * c1 * c2 * s4
	Z = -3 * s2 * s4 + 3 * c3 * c4 * c2 + 3.5 * c3 * c2 + 2.5

	return (X,Y,Z)

angles = [0, 0, np.pi/2, 0]
print(CalculateFK(angles))

# def fk3(angles):
# 	theta1 = angles[0] - np.pi/2
# 	theta2 = angles[1] - np.pi/2
# 	theta3 = angles[2]
# 	theta4 = angles[3]
#
# 	c1 = np.cos(theta1)
# 	c2 = np.cos(theta2)
# 	c3 = np.cos(theta3)
# 	c4 = np.cos(theta4)
# 	s1 = np.sin(theta1)
# 	s2 = np.sin(theta2)
# 	s3 = np.sin(theta3)
# 	s4 = np.sin(theta4)
# 	k = np.array([
# 		[-c1 * s2 * s4 + c4 * (c1 * c2 * c3 - s1 * s3), -c1 * c4 * s2 - s4 * (c1 * c2 * c3 - s1 * s3), -c1 * c2 * s3 - c3 * s1, 3.5 * c1 * c2 * c3 - 3 * c1 * s2 * s4 + 3 * c4 * (c1 * c2 * c3 - s1 * s3) - 3.5 * s1 * s3],
# 		[c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4, -c4 * s1 * s2 - s4 * (c1 * s3 + c2 * c3 * s1), c1 * c3 - c2 * s1 * s3, 3.5 * c1 * s3 + 3.5 * c2 * c3 * s1 + 3 * c4 * (c1 * s3 + c2 * c3 * s1) - 3 * s1 * s2 * s4],
# 		[-c2 * s4 - c3 * c4 * s2, -c2 * c4 + c3 * s2 * s4, s2 * s3, -3 * c2 * s4 - 3 * c3 * c4 * s2 - 3.5 * c3 * s2 + 2.5],
# 		[0, 0, 0, 1]
# 	])
# 	return k[:, -1][:-1]
#
#
# #angles = [np.pi/2, 0, np.pi/2, 0]
# angles = [0.5, 1.0, -0.4, 0.6]
# angles = [0, 0, np.pi/2, 0]
# print(fk3(angles))
# ar = fk3(angles)
# print(round(ar[0]),round(ar[1]),round(ar[2]))