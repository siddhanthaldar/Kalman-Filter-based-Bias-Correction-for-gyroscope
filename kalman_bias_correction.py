import rospy
from sensor_msgs.msg import Imu
from math import *
import numpy as np 
import matplotlib.pyplot as plt


def kalman(data):
	global b
	global P
	global R
	global sigma
	global I
	global count
	global count_min
	global count_max
	global size
	global g
	global g_correct

	if count < count_min:
		return
	
	if count > count_max :	
		g_correct = g - np.transpose(b)
		plt.subplot(121)
		plt.hist(g, bins='auto')
		plt.title('Original data')
		plt.subplot(122)
		plt.hist(g_correct, bins='auto')
		plt.title('Corrected data')
		plt.suptitle('Kalman Correction')
		plot.show()
		
	gx = data.angular_velocity.x
	gy = data.angular_velocity.y
	gz = data.angular_velocity.z

	# rospy.loginfo("I heard %s", gx)

	G = np.array([[gx], [gy], [gz]], np.float64)
	G_meas = np.subtract(G, b) # G = G_true + epsilon, where epsilon is the gyroscope noise
	H = np.multiply(2.0,np.add(np.transpose(G_meas), np.multiply(-1.0, np.transpose(b))))
	R = np.multiply(4.0,np.matmul(np.transpose((G_meas - b)),np.matmul(sigma,(G_meas - b)))) + np.multiply(2,np.trace((sigma**2)))
	Z = np.power((np.sum(G_meas)),2.0)  #np.linalg.det
	
	Y = Z - np.matmul(H,b)
	S = np.matmul(H,np.matmul(P,np.transpose(H))) + R
	K = np.matmul(P,np.matmul(np.transpose(H),np.linalg.inv(S)))

	b = b + np.matmul(K,Y)
	P = np.matmul((I - np.matmul(K,H)),P)

	g[count - count_min] = G 
	count+=1

	print("bias : [ ", b[0][0]," , ",b[1][0]," , ",b[2][0]," ]")
	return



def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/imu0", Imu, kalman)

	rospy.spin()

if __name__ == '__main__':
	global b
	global P
	global R
	global sigma
	global I
	global count
	global count_min
	global count_max
	global size
	global g
	global g_correct

	hz = 2
	time_start = 25
	time_end = 45 #seconds
	count_min = hz * time_start
	count_max = hz * time_end
	size = count_max - count_min	
	g = np.empty([size,3], dtype = np.float64)
	g_correct = np.empty([size,3], dtype = np.float64)

	b = np.array([[0],[0],[0]], np.float64) #matrix([[0.],[0,],[0.]])
	P = R = sigma = np.array([[0.0282,0,0],[0,0.0247,0],[0,0,0.0328]], np.float64) #matrix([[0.0282,0.,0.],[0.,0.0247,0.],[0.,0.,0.0328]])
	I = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], np.float64) #matrix([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

	count = 0

	listener()	
