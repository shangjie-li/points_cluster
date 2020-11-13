#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <sensor_msgs/PointCloud2.h>

#define PI 3.1415926

class EuClusterCore
{

private:
    std::string sub_topic_;
    std::string pub_topic_;
    
    double min_cluster_size_;
    double max_cluster_size_;
    
    double oriented_rectangle_fitting_distance_;
    double fitting_accuracy_;
    
    int seg_num_;
    std::vector<double> seg_distance_;
    std::vector<double> cluster_distance_;

    ros::Subscriber sub_pointcloud;
    ros::Publisher pub_boundingboxes;

    std_msgs::Header pointcloud_header;
    
    double point_project(pcl::PointXYZ &p1, pcl::PointXYZ &p2, pcl::PointXYZ &pp, pcl::PointXYZ &pp_);
    
    void find_rect(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, pcl::PointXYZ &p_p1_best, pcl::PointXYZ &p_p2_best, pcl::PointXYZ &p_p3_best, pcl::PointXYZ &p_p4_best,
               double &theta_best, double theta_min, double theta_max, double theta_interval);

    void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, double in_max_cluster_distance, std::vector<jsk_recognition_msgs::BoundingBox> &obj_list);

    void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<jsk_recognition_msgs::BoundingBox> &obj_list);

    void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

public:
    EuClusterCore(ros::NodeHandle &nh);
    ~EuClusterCore();
};
