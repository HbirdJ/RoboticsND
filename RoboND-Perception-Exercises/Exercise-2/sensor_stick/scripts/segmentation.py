#!/usr/bin/env python

# Import modules
from pcl_helper import *
import pcl

# TODO: Define functions as required
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

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    pcloud_xyzrgb = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    vox = pcloud_xyzrgb.make_voxel_grid_filter()
    vox_size = .02
    vox.set_leaf_size(vox_size, vox_size, vox_size)
    
    pcloud_downsamp = vox.filter()

    # PassThrough Filter
    z_axis = 'z'
    z_min = .7
    z_max = 1.3
    cloud_pass1 = passthrough(pcloud_downsamp, z_axis, z_min, z_max)
    
    y_axis = 'y'
    y_min = -20.
    y_max = -1.35
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

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
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
    pcl_objects = pcl_to_ros(cloud_objects)
    pcl_table = pcl_to_ros(cloud_table)
    pcl_clusters = pcl_to_ros(cloud_clusters)

    # Publish ROS messages
    pcl_objects_pub.publish(pcl_objects)
    pcl_table_pub.publish(pcl_table)
    pcl_clusters_pub.publish(pcl_clusters)


if __name__ == '__main__':

    #ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    #Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    #Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("/clusters", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    #Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
