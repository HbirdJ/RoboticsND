#!/usr/bin/env python

print "Importing modules..."

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

print "Finished importing modules."

world_num = input("Enter the current world number: ")

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Helper filter functions
def passthrough(pcl, axis, ax_min, ax_max):
    pfilter = pcl.make_passthrough_filter()
    pfilter.set_filter_field_name(axis)
    pfilter.set_filter_limits(ax_min, ax_max)
    
    return pfilter.filter()
    
def ransac(cloud, max_dist):
    seg_filter = cloud.make_segmenter()
    seg_filter.set_model_type(pcl.SACMODEL_PLANE)
    seg_filter.set_method_type(pcl.SAC_RANSAC)
    seg_filter.set_distance_threshold(max_dist)
    
    inlay, coeff = seg_filter.segment()
    inliers = cloud.extract(inlay, negative=False)
    outliers = cloud.extract(inlay, negative=True)
    
    return inliers, outliers

def outlier(cloud, neighbors = 50, threshold = 1.0):
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(neighbors)
    outlier_filter.set_std_dev_mul_thresh(threshold)
    cloud_filtered = outlier_filter.filter()
    
    return outlier_filter.filter()

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    print "Point Cloud Recieved. Analyzing..."

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    pcloud_xyzrgb = ros_to_pcl(pcl_msg)
    
    # Statistical Outlier Filtering
    neighbors = 5 
    threshold = .1
    outliers = outlier(pcloud_xyzrgb, neighbors, threshold)

    # Voxel Grid Downsampling
    vox = outliers.make_voxel_grid_filter()
    vox_size = .01
    vox.set_leaf_size(vox_size, vox_size, vox_size)
    
    pcloud_downsamp = vox.filter()

    # PassThrough Filter
    z_axis = 'z'
    z_min = .5
    z_max = 1
    cloud_pass1 = passthrough(pcloud_downsamp, z_axis, z_min, z_max)
    
    y_axis = 'x'
    y_min = .35
    y_max = 1.
    cloud_pass2 = passthrough(cloud_pass1, y_axis, y_min, y_max)

    # RANSAC Plane Segmentation
    cloud_table, cloud_objects = ransac(cloud_pass2, .01)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(.05)
    ec.set_MinClusterSize(30)
    ec.set_MaxClusterSize(2000)
    
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    print "%s Objects observed" %(len(cluster_indices))

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cloud_clusters = pcl.PointCloud_PointXYZRGB()
    cloud_clusters.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    pcl_filtered = pcl_to_ros(cloud_pass2)
    pcl_objects = pcl_to_ros(cloud_objects)
    pcl_table = pcl_to_ros(cloud_table)
    pcl_clusters = pcl_to_ros(cloud_clusters)

    # Publish ROS messages
    pcl_filtered_pub.publish(pcl_filtered)
    pcl_objects_pub.publish(pcl_objects)
    pcl_table_pub.publish(pcl_table)
    pcl_clusters_pub.publish(pcl_clusters)

    # Exercise-3:
    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        
        # Compute the associated feature vector
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
    
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Check for duplicated object detections:
    if len(detected_objects_labels) != len(set(detected_objects_labels)):
        print "Model detecting duplicate items. Waiting for next point cloud..."
        return
    
    try:
        print("Finished pcl analysis, passing objects to mover.")
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass
    
    

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    #Initialize variables
    objects = []
    groups = []
    
    object_name = String()
    arm_name = String()

    test_scene_num = Int32()
    test_scene_num.data = world_num
    
    pick_pose = Pose()
    place_pose = Pose()

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    bins = rospy.get_param('/dropbox')

    # Parse parameters into individual variables
    for i in xrange(len(object_list_param)):
        print("parsing object pick list #%s" %(i+1))
        object_label = object_list_param[i]['name']
        objects.append(object_label)
        object_group = object_list_param[i]['group']
        groups.append(object_group)
    
    pick_list = dict(zip(objects, groups))
    
    bin_dicts = []
        
    for i in xrange(len(bins)):
        bin_group = bins[i]['group']        
        bin_name = bins[i]['name']
        bin_location = bins[i]['position']
        bin_info = {"group": bin_group, "name": bin_name, "location": bin_location}
        bin_dicts.append(bin_info)
        
    print(bin_dicts)
    
    # TODO: Rotate PR2 in place to capture side tables for the collision map
    print("Rotation code = TODO")
    
    dict_list = []
    
    # Loop through the pick list
    for obj in objects:
        for i in object_list:
            if i.label == obj:
                print("Found pick list item %s in scene" %obj)
                
                place_bin = pick_list[obj]
                print("Pick list requests %s bin" %place_bin)
                
                bin_info = filter(lambda binned: binned['group'] == place_bin, bin_dicts)

                #Get the PointCloud for a given object and obtain it's   centroid
                points_arr = ros_to_pcl(i.cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]
                centroid_scalar = (np.asscalar(centroid[0]), np.asscalar(centroid[1]), np.asscalar(centroid[2]))
                
                object_name.data = obj
                pick_pose.position.x = centroid_scalar[0]
                pick_pose.position.y = centroid_scalar[1]
                pick_pose.position.z = centroid_scalar[2]
                
                place_xyz = bin_info[0]['location']
                place_pose.position.x = place_xyz[0]
                place_pose.position.y = place_xyz[1]
                place_pose.position.z = place_xyz[2]
                
                # Assign the arm to be used for pick_place
                arm_name.data = bin_info[0]['name']
                
                # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
                print("Created yaml_dict: %s" %yaml_dict)
                dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            #resp = pick_place_routine(test_scene_num, object.label, WHICH_ARM, centroid_scalar, PLACE_POSE)

            #print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    yaml_filename = "./pr2_robot/config/output_%s.yaml" %world_num
    send_to_yaml(yaml_filename, dict_list)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('analysis', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("/clusters", PointCloud2, queue_size=1)
    pcl_filtered_pub = rospy.Publisher("/filtered", PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher("/labeled", DetectedObjectsArray, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)

    # Load Model From disk
    print "Loading classifier model..."
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    print "Classifier model loaded."

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        print "Node running"
        rospy.spin()
