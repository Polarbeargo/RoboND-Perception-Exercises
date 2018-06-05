#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    pcl_msg = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    outlier_filter = outlier_filter(pcl_msg, 5, 0.05)

    # TODO: Voxel Grid Downsampling
    outlier_filter = voxel_grid(cloud_filtered, 0.005)

    # TODO: PassThrough Filter
    # Filter the point cloud along the x axis
    outlier_filter = pass_through_filter(
        outlier_filter, 'x', axis_min=0.1, axis_max=0.9)

    # Filter the point cloud along the y axis
    outlier_filter = pass_through_filter(
        outlier_filter, 'y', axis_min=-0.47, axis_max=0.47)

    # Filter the point cloud along the z axis
    outlier_filter = pass_through_filter(
        outlier_filter, 'z', axis_min=0.6, axis_max=0.9)

    # TODO: RANSAC Plane Segmentation
    inliers, coefficients = ransac_filter(outlier_filter, max_distance=0.01)

    # TODO: Extract inliers and outliers
    outlier_table = cloud_filtered.extract(inliers, negative=False)
    outlier_objects = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(outlier_objects)
    cluster_indices = euclidean_cluster(white_cloud)

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(outlier_objects)
    ros_cloud_table = pcl_to_ros(outlier_table)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber(
        "/pr2/world/points", PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher(
        "/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher(
        "/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher(
        "/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher(
        "/detected_objects", DetectedObjectsArray, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()