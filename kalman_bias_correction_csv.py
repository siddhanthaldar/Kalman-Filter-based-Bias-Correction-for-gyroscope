import rospy
from sensor_msgs.msg import Imu
from math import *
import numpy as np 
import matplotlib.pyplot as plt
import csv

def kalman():
	global b
	global P
	global R
	global sigma
	global I
	global g_correct

	gyro_data = np.genfromtxt("angular_velocity.csv", delimiter=',')[5000:7000]

	for i in range(2000):
		gx = gyro_data[i][0]
		gy = gyro_data[i][1]
		gz = gyro_data[i][2]


		G = np.array([[gx], [gy], [gz]], np.float64)
		G_meas = G ## G = G_true + epsilon + b, where epsilon is the gyroscope noise
		H = np.multiply(2.0,np.add(np.transpose(G_meas), np.multiply(-1.0, np.transpose(b))))
		R = np.multiply(4.0,np.matmul(np.transpose((G_meas - b)),np.matmul(sigma,(G_meas - b)))) + np.multiply(2,np.trace((sigma**2)))
		Z = np.linalg.norm(G_meas)**2 

		Y = Z - np.matmul(H,b)
		S = np.matmul(H,np.matmul(P,np.transpose(H))) + R
		K = np.matmul(P,np.matmul(np.transpose(H),np.linalg.inv(S)))

		b = b + np.matmul(K,Y)
		P = np.matmul((I - np.matmul(K,H)),P)

	g_correct = np.zeros([2000,3], dtype = np.float64)
	g_correct = gyro_data - np.transpose(b)

	plt.subplot(121)
	plt.hist(gyro_data, bins='auto')
	plt.title('Original data')
	plt.subplot(122)
	plt.hist(g_correct, bins='auto')
	plt.title('Corrected data')
	plt.suptitle('Kalman Correction')
	plt.show()



if __name__ == '__main__':
	global b
	global P
	global R
	global sigma
	global I
	global g_correct

	std_dev = 1.6968e-04
	b = np.array([[0],[0],[0]], np.float64) 
	P = R = sigma = np.array([[std_dev,0,0],[0,std_dev,0],[0,0,std_dev]], np.float64) 
	I = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], np.float64)

	kalman()