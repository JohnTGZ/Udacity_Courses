# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Evaluate performance of object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import numpy as np
import matplotlib
matplotlib.use('wxagg') # change backend so that figure maximizing works on Mac as well     
import matplotlib.pyplot as plt

import torch
from shapely.geometry import Polygon
from operator import itemgetter

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# object detection tools and helper functions
import misc.objdet_tools as tools


# compute various performance measures to assess object detection
def measure_detection_performance(detections, labels, labels_valid, min_iou=0.5):
    
     # find best detection for each valid label 
    true_positives = 0 # no. of correctly detected objects
    center_devs = []
    ious = []
    for label, valid in zip(labels, labels_valid):
        matches_lab_det = []
        if valid: # exclude all labels from statistics which are not considered valid
            
            # compute intersection over union (iou) and distance between centers

            ####### ID_S4_EX1 START #######     
            #######
            print("student task ID_S4_EX1 ")

            ## step 1 : extract the four corners of the current label bounding-box

            # compute location of each corner of a box and returns [front_left, rear_left, rear_right, front_right]
            label_corner = tools.compute_box_corners(   label.box.center_x, label.box.center_y, 
                                                        label.box.width, label.box.length, 
                                                        label.box.heading)
            # box {
                # center_x: 29.173896674168645
                # center_y: 0.7435155805960676
                # center_z: 0.8929607095304846
                # width: 2.0354414125040745
                # length: 4.307956063242486
                # height: 1.9099999999999966
                # heading: -0.010818934012628567
            # }
                # metadata {
                # speed_x: 16.235600779837682
                # speed_y: 0.20547721495915994
                # accel_x: -0.41109690882592526
                # accel_y: -0.12968907061861043
            # }

            ## step 2 : loop over all detected objects
            for obj in detections:
                ## step 3 : extract the four corners of the current detection
                # print(obj)
                # obj: [1, x, y, z, h, w, l, yaw]
                detection_corner = tools.compute_box_corners(   obj[1], obj[2], 
                                                                obj[5], obj[6], 
                                                                obj[7])
                ## step 4 : computer the center distance between label and detection bounding-box in x, y, and z
                dist_x = obj[1] - label.box.center_x
                dist_y = obj[2] - label.box.center_y
                dist_z = obj[3] - label.box.center_z
                # center_devs.append([dist_x , 
                #                     dist_y , 
                #                     dist_z])
                
                ## step 5 : compute the intersection over union (IOU) between label and detection bounding-box

                
                try:
                    poly_1 = Polygon(detection_corner)
                    poly_2 = Polygon(label_corner)
                    intersection = poly_1.intersection(poly_2).area
                    union = poly_1.union(poly_2).area
                    iou = intersection / union
                except Exception as err:
                    print("Error in computating IOU:", err)

                ## step 6 : if IOU exceeds min_iou threshold, store [iou,dist_x, dist_y, dist_z] in matches_lab_det and increase the TP count
                if (iou > min_iou):
                    matches_lab_det.append([iou, dist_x, dist_y, dist_z])
                    # true_positives += 1

            #######
            ####### ID_S4_EX1 END #######     
            
        # find best match and compute metrics
        if matches_lab_det:
            best_match = max(matches_lab_det,key=itemgetter(1)) # retrieve entry with max iou in case of multiple candidates   
            ious.append(best_match[0])
            center_devs.append(best_match[1:])

    # print(f"ious: {ious}")
    # print(f"center_devs: {center_devs}")

    ####### ID_S4_EX2 START #######     
    #######
    print("student task ID_S4_EX2")
    
    # compute positives and negatives for precision/recall

    ## step 1 : compute the total number of positives present in the scene
    all_positives = labels_valid.sum()

    true_positives = len(ious)

    ## step 2 : compute the number of false negatives
    #What is supposed to be positive but ended up negative
    # All TRUE positives - true positives from model
    false_negatives = all_positives - true_positives

    ## step 3 : compute the number of false positives
    #What is supposed to be negative but ended up positive
    # All positives from model - true positives from model
    false_positives = len(detections) - true_positives
    
    # print(f"false_negatives: \
    #         {false_negatives}")

    # print(f"false_positives: \
    #         {false_positives}")


    #######
    ####### ID_S4_EX2 END #######     
    
    pos_negs = [all_positives, true_positives, false_negatives, false_positives]
    det_performance = [ious, center_devs, pos_negs]
    
    return det_performance


# evaluate object detection performance based on all frames
def compute_performance_stats(det_performance_all):

    # extract elements
    ious = []
    center_devs = []
    pos_negs = []
    for item in det_performance_all:
        ious.append(item[0])
        center_devs.append(item[1])
        pos_negs.append(item[2])
    pos_negs_arr = np.asarray(pos_negs)
    
    ####### ID_S4_EX3 START #######     
    #######    
    print('student task ID_S4_EX3')

    print(f"pos_negs_arr: {pos_negs_arr}")
    ## step 1 : extract the total number of positives, true positives, false negatives and false positives
    total_positives = sum(pos_negs_arr[:,0])
    true_positives = sum(pos_negs_arr[:,1])
    false_negatives = sum(pos_negs_arr[:,2])
    false_positives = sum(pos_negs_arr[:,3])

    ## step 2 : compute precision
    #   Measures model's accuracy in classifying a sample as positive
    precision = (true_positives)/ (float(true_positives) + false_positives)

    ## step 3 : compute recall 
    #   Measures model's ability to detect positive samples
    recall = (true_positives)/ (float(true_positives) + false_negatives)

    #######    
    ####### ID_S4_EX3 END #######     
    print('precision = ' + str(precision) + ", recall = " + str(recall))   

    # serialize intersection-over-union and deviations in x,y,z
    ious_all = [element for tupl in ious for element in tupl]
    devs_x_all = []
    devs_y_all = []
    devs_z_all = []
    for tuple in center_devs:
        for elem in tuple:
            dev_x, dev_y, dev_z = elem
            devs_x_all.append(dev_x)
            devs_y_all.append(dev_y)
            devs_z_all.append(dev_z)
    

    # compute statistics
    stdev__ious = np.std(ious_all)
    mean__ious = np.mean(ious_all)

    stdev__devx = np.std(devs_x_all)
    mean__devx = np.mean(devs_x_all)

    stdev__devy = np.std(devs_y_all)
    mean__devy = np.mean(devs_y_all)

    stdev__devz = np.std(devs_z_all)
    mean__devz = np.mean(devs_z_all)
    #std_dev_x = np.std(devs_x)

    # plot results
    data = [precision, recall, ious_all, devs_x_all, devs_y_all, devs_z_all]
    titles = ['detection precision', 'detection recall', 'intersection over union', 'position errors in X', 'position errors in Y', 'position error in Z']
    textboxes = ['', '', '',
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (np.mean(devs_x_all), ), r'$\mathrm{sigma}=%.4f$' % (np.std(devs_x_all), ), r'$\mathrm{n}=%.0f$' % (len(devs_x_all), ))),
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (np.mean(devs_y_all), ), r'$\mathrm{sigma}=%.4f$' % (np.std(devs_y_all), ), r'$\mathrm{n}=%.0f$' % (len(devs_x_all), ))),
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (np.mean(devs_z_all), ), r'$\mathrm{sigma}=%.4f$' % (np.std(devs_z_all), ), r'$\mathrm{n}=%.0f$' % (len(devs_x_all), )))]

    f, a = plt.subplots(2, 3)
    a = a.ravel()
    num_bins = 20
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    for idx, ax in enumerate(a):
        ax.hist(data[idx], num_bins)
        ax.set_title(titles[idx])
        if textboxes[idx]:
            ax.text(0.05, 0.95, textboxes[idx], transform=ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)
    plt.tight_layout()
    plt.show()

