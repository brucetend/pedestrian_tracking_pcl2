#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 17:38:11 2019

@author: oh372
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from rospy_message_converter import message_converter

def callback(data):
    #make a list of all points of incoming PointCloud2.msg but only relevant fields x,y,cluster,label
    cloud_points = list(point_cloud2.read_points(cloud=data, field_names = ('x','y','cluster','label')))
    
    
    dictionary={}
    for element in range(len(cloud_points)):
        data_point = list(cloud_points[element])
        
        # only proceed with points that have the label "pedestrian", 
        if data_point[3] == 3:
            key = data_point[2]

            #check if pedestrian already exists in dictionary by its cluster
            if key in dictionary:
                # append array of the key with new data points if key already exists in dictionary
                dictionary[key] = np.vstack([dictionary[key], [data_point[0], data_point[1]]])
                
            else:
                #make new entry in dictionary for new key
                dictionary.update({key: np.array([data_point[0], data_point[1]])})


    centroid_dictionary={}
    centroids_array = []

    for cluster in dictionary:

        if dictionary[cluster].size > 2:
            #calculate centroid for KF
            centroid = np.mean(dictionary[cluster], axis=0)
            # calculate Bounding Box parameters center and size of bounding box for vision_msgs/BoundingBox3D Message, Z-COORDINATE IS MISSING!
            # maximum_coord = array([x(max), y(max)])
            maximum_coord = np.amax(dictionary[cluster], axis=0)
            minimum_coord = np.amin(dictionary[cluster], axis=0)

            pose_center = np.array([float(minimum_coord[0]+(maximum_coord[0]-minimum_coord[0])/2), float(minimum_coord[1]+(maximum_coord[1]-minimum_coord[1])/2)])
            size = np.array([float(maximum_coord[0]-minimum_coord[0]), float(maximum_coord[1]-minimum_coord[1])])
            
        else:
            centroid = dictionary[cluster]
            pose_center = centroid

        centroid_dictionary.update({cluster: centroid})
        centroids_array.append(centroid[0])
        centroids_array.append(centroid[1])
    
    rospy.loginfo(centroid_dictionary)
    rospy.loginfo(centroids_array)

    #Publish dictionary as String    
    centroid_message = str(centroid_dictionary)
    pub = rospy.Publisher('centroid', String, queue_size=10)
    pub.publish(centroid_message)

    #Publish centroids as Float64MultiArray
    pub = rospy.Publisher('centroid2', Float64MultiArray, queue_size=10)
    data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
    data_to_send.data = centroids_array # assign the 1D-array (=list) with the values you want to send
    
    centroid_array_columns = 2  # Anzahl der Koordinaten (x,y -> 2)
    centroid_array_rows = len(centroids_array)/centroid_array_columns  # Anzahl Centroids in frame

    # assign the shape of the matrix
    '''data_to_send.layout.dim[0].size = centroid_array_rows
    data_to_send.layout.dim[1].size = centroid_array_columns
    data_to_send.layout.dim[2].size = 1'''
    pub.publish(data_to_send)
    




def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/cocar/ibeo_lux_201/cloud', PointCloud2, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()