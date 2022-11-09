#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This scripts will generates random points along an random axis which is started from the lidar center.

import rospy
import time
import numpy as np
import math as m
import random
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from cyber_security.msg import CyberLidarFakePoints
from std_msgs.msg import Header
import struct
import sys


# Eulers Rotation matrix
def Rx(theta):
  return np.array([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.array([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.array([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

begin_time = 0

def reset_parameters():
    global is_attack, start_dist, end_dist, radius, number_of_points, attack_duration, publish_freq

    is_attack = False
    start_dist = 5
    end_dist = 60
    radius = 1
    number_of_points = 3
    attack_duration = 1
    publish_freq = 5


def recieve_cyber_param(data):
    global is_attack, start_dist, end_dist, radius, number_of_points, attack_duration, begin_time, phi_base, theta_base, publish_freq

    if data.is_attack:
        is_attack = True
        start_dist = data.start_distance
        end_dist = data.end_distance
        number_of_points = data.number_of_points
        attack_duration = data.radius
        publish_freq = data.frequency
        begin_time = time.time()
        phi_base = m.pi/6 + m.pi/18 * random.random()
        theta_base = m.radians(80) + m.radians(random.random() * 20)
        print(is_attack)
    else:
        reset_parameters()




def point_generator():
    global is_attack, number_of_points

    # P1, P2 are the start and end points on the laser line that randoms are generated
    # start_dist = 5 # meters the distance from the sensor for the noise begining
    # end_dist = 50 # meters  the distance from the sensor for the noise finishing
    p1 = np.array([0, 0, start_dist])
    p2 = np.array([0, 0, end_dist])

    # radius = 1 # Radius of the false points
    # number_of_points = 3 #random.randint(100, 300, 3) #100 # of points to generate

    

    # Rotation to random direction
    if is_attack: # starting of the attack
        elapsed_time = time.time() - begin_time # check the elapsed time
        theta =  theta_base

        if elapsed_time < attack_duration:
            phi = (elapsed_time/attack_duration) * m.pi/6  + phi_base  # attack omega(0-30°) + phi_base
            
        else:
            is_attack = False
            number_of_points = 3
            phi = phi_base + m.pi/6
            
     
    else:
        phi = 2 * m.pi * random.random() #  0 < phi < 360°| Horizontal Field of View: 360°
        theta = m.radians(75) + m.radians(random.random() * 40) #  75° < theta < 125° | Front Top Lidar vertical Field of View: 40° (-25° to +15°) 


    final_axial_FP = [[0,0,0]] * number_of_points # Initialization of the final fake points in the target direction
    Rot_mat = np.dot(Rz(phi),Ry(theta)) # Random Rotation matrix
    # print(phi)

    axis = np.subtract(p2,p1) # Target direction for Fake Points
    # axis_unit = axis / m.sqrt(np.dot(axis, axis)) # [i=0,j=0,k=1] 

    for i in range(number_of_points):
        axial_points = np.add(p1, np.multiply(axis,random.random()))
        circle_r = random.random() * radius
        circle_theta = random.random() * 2 * m.pi
        circle_points = np.array([m.cos(circle_theta) * circle_r, m.sin(circle_theta) * circle_r, 0])
        # print(Rot_mat)
        # print(np.add(axial_points,circle_points))

        # final_axial_FP[i] = Rot_mat * np.add(axial_points,circle_points).reshape(3,1) # the final fake points in the target direction
        final_axial_FP[i] = np.matmul(Rot_mat,np.add(axial_points,circle_points))
        # print(final_axial_FP)

    return final_axial_FP



# FPs = point_generator()
# print(FPs)
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(list(i[0] for i in FPs),list(i[1] for i in FPs),list(i[2] for i in FPs))
# plt.show()


def fake_point_publisher():

    rospy.init_node('fake_point_pub', anonymous=True)
    pub = rospy.Publisher('/points_fake', PointCloud2, queue_size=10)
    rospy.Subscriber("/cyber_attack_param",CyberLidarFakePoints, recieve_cyber_param)
    reset_parameters()
    
    header = Header()
    header.frame_id = "lidar_front_top"
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1),
              ]

    while not rospy.is_shutdown():
        FPs = point_generator() # generate fake points along an arbitrary direction

        for j in range(len(FPs)): # Adding rgb data to the list of point
            rgb_int = np.random.randint(0, 255, 3) 
            rgb = struct.unpack('I', struct.pack('BBBB',rgb_int[2], rgb_int[1], rgb_int[0] , 255))[0]
            FPs[j] = np.insert(FPs[j],3,rgb)
        header.stamp = rospy.Time.now()
        fake_pcl = point_cloud2.create_cloud(header, fields, FPs)
        pub.publish(fake_pcl)
        time.sleep(1/publish_freq)

if __name__ == '__main__':
    try:
        fake_point_publisher()
    except rospy.ROSInterruptException:
        pass
