#include "points_cluster_core.h"

EuClusterCore::EuClusterCore(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/rslidar_points_no_ground");
    nh.param<std::string>("pub_topic", pub_topic_, "/clustered_bounding_boxes");
    
    nh.param<double>("min_cluster_points_num", min_cluster_points_num_, 5);
    nh.param<double>("max_cluster_points_num", max_cluster_points_num_, 4000);
    
    nh.param<double>("min_cluster_size", min_cluster_size_, 0.1);
    nh.param<double>("max_cluster_size", max_cluster_size_, 10);
    
    nh.param<double>("oriented_rectangle_fitting_distance", oriented_rectangle_fitting_distance_, 10);
    nh.param<double>("fitting_accuracy", fitting_accuracy_, 2);
    
    nh.param<int>("seg_num_", seg_num_, 5);
    ros::param::get("~seg_distance",seg_distance_);
	ros::param::get("~cluster_distance",cluster_distance_);
    
    sub_pointcloud = nh.subscribe(sub_topic_, 1, &EuClusterCore::point_cb, this);
    pub_boundingboxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(pub_topic_, 1);

    ros::spin();
}

EuClusterCore::~EuClusterCore(){}

double EuClusterCore::point_project(pcl::PointXYZ &p1, pcl::PointXYZ &p2, pcl::PointXYZ &pp, pcl::PointXYZ &pp_)
{
    pp_.x = p1.x + p2.x - pp.x;
    pp_.y = p1.y + p2.y - pp.y;
    
    if (p1.x == p2.x)
    {
        pp_.x = p1.x;
        pp_.y = pp.y;
    }
    else
    {
        double dd = (p2.y - p1.y) * (p2.y - p1.y) + (p2.x - p1.x) * (p2.x - p1.x);
        double tmp_x = (p2.x - p1.x) * (p2.y - p1.y) * (pp.y - p1.y) + pp.x * (p2.x - p1.x) * (p2.x - p1.x) + p1.x * (p2.y - p1.y) * (p2.y - p1.y);
        double tmp_y = (p2.x - p1.x) * (p2.y - p1.y) * (pp.x - p1.x) + pp.y * (p2.y - p1.y) * (p2.y - p1.y) + p1.y * (p2.x - p1.x) * (p2.x - p1.x); 
        pp_.x = tmp_x / dd;
        pp_.y = tmp_y / dd;
    }
    
    return sqrt(pow(pp.x - pp_.x, 2) + pow(pp.y - pp_.y, 2));
}

void EuClusterCore::find_rect(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, pcl::PointXYZ &p_p1_best, pcl::PointXYZ &p_p2_best, pcl::PointXYZ &p_p3_best, pcl::PointXYZ &p_p4_best,
               double &theta_best, double theta_min, double theta_max, double theta_interval)
{
    double reference_dis_sum_min = std::numeric_limits<float>::max();
    int ii_max = (theta_max - theta_min) / theta_interval;

#pragma omp parallel for
    //以不同方位角拟合矩形
    for (int ii = 0; ii < ii_max; ii++)
    {
        double theta_i = ii * theta_interval;
        pcl::PointXYZ p_original;
        p_original.x = 0;
        p_original.y = 0;
            
        pcl::PointXYZ p_l1;
        p_l1.x = cos(theta_i);
        p_l1.y = sin(theta_i);
        pcl::PointXYZ p_l2;
        p_l2.x = cos(theta_i + PI / 2);
        p_l2.y = sin(theta_i + PI / 2);
            
        pcl::PointXYZ p_l1_min;
        p_l1_min.x = std::numeric_limits<float>::max();
        pcl::PointXYZ p_l1_max;
        p_l1_max.x = - std::numeric_limits<float>::max();
        pcl::PointXYZ p_l2_min;
        p_l2_min.y = std::numeric_limits<float>::max();
        pcl::PointXYZ p_l2_max;
        p_l2_max.y = - std::numeric_limits<float>::max();
        
        //遍历每个点，求包络矩形
        for (size_t pii = 0; pii < in_pc->points.size(); pii++)
        {
            pcl::PointXYZ p;
            p.x = in_pc->points[pii].x;
            p.y = in_pc->points[pii].y;
            pcl::PointXYZ p_;
            double dis_pro_temp_;
                
            dis_pro_temp_ = point_project(p_original, p_l1, p, p_);
            if (p_.x < p_l1_min.x)
                p_l1_min = p_;
            if (p_.x > p_l1_max.x)
                p_l1_max = p_;
                
            dis_pro_temp_ = point_project(p_original, p_l2, p, p_);
            if (p_.y < p_l2_min.y)
                p_l2_min = p_;
            if (p_.y > p_l2_max.y)
                p_l2_max = p_;
        }
        
        pcl::PointXYZ p_p1;
        pcl::PointXYZ p_p2;
        pcl::PointXYZ p_p3;
        pcl::PointXYZ p_p4;
            
        p_p1.x = p_l1_min.x + p_l2_min.x;
        p_p1.y = p_l1_min.y + p_l2_min.y;
        p_p2.x = p_l1_max.x + p_l2_min.x;
        p_p2.y = p_l1_max.y + p_l2_min.y;
        p_p3.x = p_l1_max.x + p_l2_max.x;
        p_p3.y = p_l1_max.y + p_l2_max.y;
        p_p4.x = p_l1_min.x + p_l2_max.x;
        p_p4.y = p_l1_min.y + p_l2_max.y;
            
        double reference_dis_sum = 0;
            
        //遍历每个点，求点云到矩形边界的参考距离和
        for (size_t pii = 0; pii < in_pc->points.size(); pii++)
        {
            pcl::PointXYZ p;
            p.x = in_pc->points[pii].x;
            p.y = in_pc->points[pii].y;
            pcl::PointXYZ p_;
                
            double dis_pro_temp_ = std::numeric_limits<float>::max();
                
            double dis_pro_temp_1_2_ = point_project(p_p1, p_p2, p, p_);
            dis_pro_temp_ = (dis_pro_temp_1_2_ < dis_pro_temp_) ? dis_pro_temp_1_2_ : dis_pro_temp_;
            double dis_pro_temp_2_3_ = point_project(p_p2, p_p3, p, p_);
            dis_pro_temp_ = (dis_pro_temp_2_3_ < dis_pro_temp_) ? dis_pro_temp_2_3_ : dis_pro_temp_;
            double dis_pro_temp_3_4_ = point_project(p_p3, p_p4, p, p_);
            dis_pro_temp_ = (dis_pro_temp_3_4_ < dis_pro_temp_) ? dis_pro_temp_3_4_ : dis_pro_temp_;
            double dis_pro_temp_4_1_ = point_project(p_p4, p_p1, p, p_);
            dis_pro_temp_ = (dis_pro_temp_4_1_ < dis_pro_temp_) ? dis_pro_temp_4_1_ : dis_pro_temp_;
                 
            reference_dis_sum += dis_pro_temp_;
        }
            
        if (reference_dis_sum < reference_dis_sum_min)
        {
            reference_dis_sum_min = reference_dis_sum;
            theta_best = theta_i;
            p_p1_best = p_p1;
            p_p2_best = p_p2;
            p_p3_best = p_p3;
            p_p4_best = p_p4;
        }
    }
}

void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                    double in_max_cluster_distance, std::vector<jsk_recognition_msgs::BoundingBox> &obj_list)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //对点云降维处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);

    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(in_max_cluster_distance);
    euclid.setMinClusterSize(min_cluster_points_num_);
    euclid.setMaxClusterSize(max_cluster_points_num_);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    //遍历每个聚类点簇
    for (size_t i = 0; i < local_indices.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_pc(new pcl::PointCloud<pcl::PointXYZ>);
        double local_pc_dis = std::numeric_limits<float>::max();
        //遍历每个点，建立点簇云
        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            pcl::PointXYZ p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;
            local_pc->points.push_back(p);
            
            double dis = sqrt(pow(p.x, 2) + pow(p.y, 2));
            if (dis < local_pc_dis)
                local_pc_dis = dis;
        }
        
        double theta_best = 0;
        
        pcl::PointXYZ p_p1_best;
        pcl::PointXYZ p_p2_best;
        pcl::PointXYZ p_p3_best;
        pcl::PointXYZ p_p4_best;
        
        //拟合带方位角的矩形
        if (local_pc_dis < oriented_rectangle_fitting_distance_)
        {
            double theta_min = 0;
            double theta_max = 90;
            
            find_rect(local_pc, p_p1_best, p_p2_best, p_p3_best, p_p4_best, theta_best, theta_min, theta_max, fitting_accuracy_);
        }
        
        //拟合不带方位角的矩形
        else
        {
            float min_x = std::numeric_limits<float>::max();
            float max_x = -std::numeric_limits<float>::max();
            float min_y = std::numeric_limits<float>::max();
            float max_y = -std::numeric_limits<float>::max();
            
            for(size_t pii = 0; pii < local_pc->points.size(); pii++)
            {
                if (local_pc->points[pii].x < min_x)
                    min_x = local_pc->points[pii].x;
                if (local_pc->points[pii].x > max_x)
                    max_x = local_pc->points[pii].x;
                if (local_pc->points[pii].y < min_y)
                    min_y = local_pc->points[pii].y;
                if (local_pc->points[pii].y > max_y)
                    max_y = local_pc->points[pii].y;
            }
            
            p_p1_best.x = min_x;
            p_p1_best.y = min_y;
            p_p2_best.x = max_x;
            p_p2_best.y = min_y;
            p_p3_best.x = max_x;
            p_p3_best.y = max_y;
            p_p4_best.x = min_x;
            p_p4_best.y = max_y;
        }
        
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        //遍历每个点，寻找点簇的高度极限
        for (size_t pii = 0; pii < local_pc->points.size(); pii++)
        {
            if (local_pc->points[pii].z < min_z)
                min_z = local_pc->points[pii].z;
            if (local_pc->points[pii].z > max_z)
                max_z = local_pc->points[pii].z;
        }
        
        //将聚类结果转换为BoundingBox消息类型
        jsk_recognition_msgs::BoundingBox bounding_box;

        bounding_box.header = pointcloud_header;

        bounding_box.pose.position.x = (p_p1_best.x + p_p3_best.x) / 2;
        bounding_box.pose.position.y = (p_p1_best.y + p_p3_best.y) / 2;
        bounding_box.pose.position.z = (min_z + max_z) / 2;
        
        bounding_box.pose.orientation.x = 0;
        bounding_box.pose.orientation.y = 0;
        bounding_box.pose.orientation.z = sin(0.5 * theta_best);
        bounding_box.pose.orientation.w = cos(0.5 * theta_best);

        bounding_box.dimensions.x = sqrt(pow(p_p1_best.x - p_p2_best.x, 2) + pow(p_p1_best.y - p_p2_best.y, 2));
        bounding_box.dimensions.y = sqrt(pow(p_p1_best.x - p_p4_best.x, 2) + pow(p_p1_best.y - p_p4_best.y, 2));
        bounding_box.dimensions.z = max_z - min_z;
        
        //根据边界框尺寸筛选聚类点簇
        double box_size = 0;
        box_size = (bounding_box.dimensions.x > box_size) ? bounding_box.dimensions.x : box_size;
        box_size = (bounding_box.dimensions.y > box_size) ? bounding_box.dimensions.y : box_size;
        box_size = (bounding_box.dimensions.z > box_size) ? bounding_box.dimensions.z : box_size;
        
        if (box_size < min_cluster_size_ || box_size > max_cluster_size_)
        {
            continue;
        }

        obj_list.push_back(bounding_box);
    }
}

void EuClusterCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<jsk_recognition_msgs::BoundingBox> &obj_list)
{
    //根据分区，定义子云
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(seg_num_);
    for (size_t j = 0; j < segment_pc_array.size(); j++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[j] = tmp;
    }

    //遍历输入点云中的点，根据距离将不同的点划分到不同的子云中
    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        for (size_t j = 0; j < segment_pc_array.size(); j++)
        {
            if (origin_distance < seg_distance_[j])
            {
                segment_pc_array[j]->points.push_back(current_point);
                break;
            }
        }
    }

    //对各子云进行聚类
    for (size_t j = 0; j < segment_pc_array.size(); j++)
    {
        cluster_segment(segment_pc_array[j], cluster_distance_[j], obj_list);
    }
}

void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud_header = in_cloud_ptr->header;

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    
    std::vector<jsk_recognition_msgs::BoundingBox> global_obj_list;
    cluster_by_distance(current_pc_ptr, global_obj_list);
    
    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header = pointcloud_header;
    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i]);
    }

    pub_boundingboxes.publish(bbox_array);
}


