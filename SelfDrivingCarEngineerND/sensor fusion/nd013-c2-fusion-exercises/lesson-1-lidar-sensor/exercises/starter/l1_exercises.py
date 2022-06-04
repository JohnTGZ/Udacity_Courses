# ---------------------------------------------------------------------
# Exercises from lesson 1 (lidar)
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Starter Code
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

from PIL import Image
import io
import sys
import os
import cv2
import numpy as np
import zlib

## Add current working directory to path
sys.path.append(os.getcwd())

## Waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2

def load_range_image(frame, lidar_name):
    lidar = [obj for obj in frame.lasers if obj.name == lidar_name][0] 
    ri = []
    if len(lidar.ri_return1.range_image_compressed) > 0:
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
    return ri

# Exercise C1-5-5 : Visualize intensity channel
def vis_intensity_channel(frame, lidar_name):

    print("Exercise C1-5-5")
    # extract range image from frame
    ri = load_range_image(frame, lidar_name)
    ri[ri<0]=0.0

    # map value range to 8bit
    #From the documentation within dataset.proto (lines 174-178) we know 
    # that the inner dimensions of a range image cell are 
    # [range, intensity, elongation, is_in_no_label_zone]
    ri_intensity = ri[:,:,1]
    max_intensity = np.amax(ri_intensity)
    min_intensity = np.amin(ri_intensity)
    ri_intensity = (max_intensity/2) * ri_intensity * (255/(max_intensity - min_intensity))
    img_intensity = ri_intensity.astype(np.uint8)

    # focus on +/- 45° around the image center
    deg45 = int(img_intensity.shape[1] / 8)
    ri_center = int(img_intensity.shape[1]/2)
    img_intensity = img_intensity[:,ri_center-deg45:ri_center+deg45]

    print('max. val = ' + str(round(np.amax(img_intensity[:,:]),2)))
    print('min. val = ' + str(round(np.amin(img_intensity[:,:]),2)))

    cv2.imshow('intensity_image', img_intensity)
    cv2.waitKey(0)


# Exercise C1-5-2 : Compute pitch angle resolution
def print_pitch_resolution(frame, lidar_name):

    print("Exercise C1-5-2")
    # load range image
    ri = load_range_image(frame, lidar_name)
        
    # compute vertical field-of-view from lidar calibration 
    num_rows = ri.shape[0]

    # compute pitch resolution and convert it to angular minutes
    calib_lidar = [obj for obj in frame.context.laser_calibrations if obj.name == lidar_name][0]

    # compute vertical field of view (vfov) in rad
    vfov_rad = calib_lidar.beam_inclination_max - calib_lidar.beam_inclination_min

    print (((vfov_rad*180/np.pi) / num_rows)/(1/60.0))


# Exercise C1-3-1 : print no. of vehicles
def print_no_of_vehicles(frame):

    # find out the number of labeled vehicles in the given frame
    # Hint: inspect the data structure frame.laser_labels
    num_vehicles = 0
    #Iterate through the laser labels
    for label in frame.laser_labels:
        if label.type == label.TYPE_VEHICLE:
            num_vehicles += 1
            
    print("number of labeled vehicles in current frame = " + str(num_vehicles))


def print_lidar_calib(frame):    
    lidar_name = dataset_pb2.LaserName.TOP
    calib_lidar = [obj for obj in frame.context.laser_calibrations if obj.name == lidar_name][0]
    print(calib_lidar)