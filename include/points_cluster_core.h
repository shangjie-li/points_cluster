#pragma once
#include "omp.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

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

#define PI 3.1415926

class EuCluster
{
private:
    std::string sub_topic_;
    std::string pub_topic_;

    bool show_objects_num_;
    bool show_time_;
    
    double min_cluster_points_num_;
    double max_cluster_points_num_;
    
    double min_cluster_size_;
    double max_cluster_size_;
    
    int seg_num_;
    std::vector<double> seg_distance_;
    std::vector<double> cluster_distance_;

    bool road_info_;
    std::vector<float> road_edge_left_;
    std::vector<float> road_edge_right_;

    ros::Subscriber sub_;
    ros::Publisher pub_;

    void segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                        double cluster_distance,
                        std::vector<visualization_msgs::Marker> &objs);

    void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                        std::vector<visualization_msgs::Marker> &objs);
    
    void crop(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
              const pcl::PointCloud<pcl::PointXYZ>::Ptr out_pc_ptr,
              std::vector<float> edge_left,
              std::vector<float> edge_right);

    void callback(const sensor_msgs::PointCloud2ConstPtr &in);

public:
    EuCluster(ros::NodeHandle &nh);
    ~EuCluster();
};
