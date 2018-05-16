#!/usr/bin/env python

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
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

TestSceneNum = 3
OutputFile = 'output{}.yaml'.format(str(TestSceneNum))


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


def statistical_outlier_filter(point_cloud, k=50, scale_factor=0.3):
    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = point_cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(10)

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(scale_factor)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()

    return cloud_filtered

def voxel_grid_downsampling(point_cloud, leaf_size=0.005):
    # Create a VoxelGrid filter object for our input point cloud

    vox = point_cloud.make_voxel_grid_filter()
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    return cloud_filtered

def pass_through_filter(point_cloud, axis='z', axis_min=0.0, axis_max=1.0):
    # Create a PassThrough filter object.
    passthrough = point_cloud.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    passthrough.set_filter_field_name(axis)
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()
    return cloud_filtered

def ransac_filtering(point_cloud, max_distance=0.01):
    # Create the segmentation object
    seg = point_cloud.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_table = point_cloud.extract(inliers, negative=False)
    cloud_objects = point_cloud.extract(inliers, negative=True)
    return cloud_table, cloud_objects

def euclidean_clustering(cloud_objects, white_cloud, cluster_tolerance=0.02, min_cluster_size=100, max_cluster_size=10000):
    
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(cluster_tolerance)
    ec.set_MinClusterSize(min_cluster_size)
    ec.set_MaxClusterSize(max_cluster_size)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

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
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list) 
    return cluster_cloud, cluster_indices

def classify_clusters(cluster_indices, cloud_objects, white_cloud):
    detected_objects_labels = []
    detected_objects = []
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
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
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

    return detected_objects, detected_objects_labels
    
    

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud_filtered = ros_to_pcl(pcl_msg)

    # Statistical Outlier Filtering
    cloud_filtered = statistical_outlier_filter(cloud_filtered)

    # Voxel Grid Downsampling
    cloud_filtered = voxel_grid_downsampling(cloud_filtered)


    # PassThrough Filter
    cloud_filtered = pass_through_filter(cloud_filtered, axis='z', axis_min=0.6, axis_max=0.8)
    cloud_filtered = pass_through_filter(cloud_filtered, axis='y', axis_min=-0.5, axis_max=0.5)


    # RANSAC Plane Segmentation
    cloud_table, cloud_objects = ransac_filtering(cloud_filtered)


    white_cloud = XYZRGB_to_XYZ(cloud_objects)

    # Euclidean Clustering
    cluster_cloud, cluster_indices = euclidean_clustering(cloud_objects, white_cloud)


    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters
    detected_objects, detected_objects_labels = classify_clusters(cluster_indices, cloud_objects, white_cloud)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)


    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    test_scene_num = Int32()
    test_scene_num.data = TestSceneNum
    dict_list = []
    

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')

    # Loop through the pick list
    for obj in object_list_param:
        
	object_name = String()
        arm_name = String()
        pick_pose = Pose()
        place_pose = Pose()
 	
        place_pose.position.x = np.float(0.0)
        place_pose.position.z = np.float(0.605)

	object_name.data = obj['name']
	
	
	#  Get the PointCloud for a given object and obtain it's centroid
	
	target_obj = None

	for det_obj in object_list:
	    if det_obj.label == object_name.data:
                target_obj = det_obj
		break
		
	if target_obj is not None:
	    points_arr = ros_to_pcl(det_obj.cloud).to_array()
    	    centroid = np.mean(points_arr, axis=0)[:3]
	    pick_pose.position.x = np.asscalar(centroid[0])
	    pick_pose.position.y = np.asscalar(centroid[1])
	    pick_pose.position.z = np.asscalar(centroid[2])
	

        #  Assign the arm to be used for pick_pose
	object_group = obj['group']
	if object_group == 'green':
	    arm_name.data =  'right'
	    place_pose.position.y = np.float(-0.71)
	else:
	    arm_name.data = 'left'
	    place_pose.position.y = np.float(0.71)

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
	yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        #rospy.wait_for_service('pick_place_routine')

        #try:
        #    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
        #    resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

        #    print ("Response: ",resp.success)

        #except rospy.ServiceException, e:
        #    print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    send_to_yaml(OutputFile, dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model_final.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']


    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()



