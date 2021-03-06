#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
       'sticky_notes',
       'book',
       'snacks',
       'biscuits',
       'eraser',
       'soap2',
       'soap',
       'glue']

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_pcls = []

    for model_name in models:
        print('Loading %s' %model_name)
        spawn_model(model_name)
        print('Loaded %s' %model_name)

        for i in range(15):
        
            print('Pose %s on current model' %i)
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()
                
                print('Try #%s' %(try_count+1))

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True
                    
                

            # Extract histogram features
            labeled_pcls.append([sample_cloud, model_name])

        print('Deleting current model')
        delete_model()


    pickle.dump(labeled_pcls, open('object_pcls.sav', 'wb'))

