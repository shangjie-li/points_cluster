#include "points_cluster_core.h"

EuCluster::EuCluster(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/pandar_points_no_ground");
    nh.param<std::string>("pub_topic", pub_topic_, "/objects");

    nh.param<bool>("show_objects_num", show_objects_num_, false);
    nh.param<bool>("show_time", show_time_, false);
    
    nh.param<double>("min_cluster_points_num", min_cluster_points_num_, 5);
    nh.param<double>("max_cluster_points_num", max_cluster_points_num_, 4000);
    
    nh.param<double>("min_cluster_size", min_cluster_size_, 0.1);
    nh.param<double>("max_cluster_size", max_cluster_size_, 8);
    
    nh.param<int>("seg_num", seg_num_, 5);
    ros::param::get("~seg_distance", seg_distance_);
    ros::param::get("~cluster_distance", cluster_distance_);

    nh.param<bool>("road_info", road_info_, false);
    
    sub_ = nh.subscribe(sub_topic_, 1, &EuCluster::callback, this);
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_, 1);
    ros::spin();
}

EuCluster::~EuCluster(){}

void EuCluster::segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                        double cluster_distance,
                        std::vector<visualization_msgs::Marker> &objs)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);

    //对点云降维处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc_ptr, *pc_ptr_2d);
    
    #pragma omp for
    for (size_t i = 0; i < pc_ptr_2d->points.size(); i++)
    {
        pc_ptr_2d->points[i].z = 0;
    }

    if (pc_ptr_2d->points.size() > 0)
        tree_ptr->setInputCloud(pc_ptr_2d);

    std::vector<pcl::PointIndices> local_indices;

    //对点云聚类，提取聚类点簇的索引
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> eucluster;
    eucluster.setInputCloud(pc_ptr_2d);
    eucluster.setClusterTolerance(cluster_distance);
    eucluster.setMinClusterSize(min_cluster_points_num_);
    eucluster.setMaxClusterSize(max_cluster_points_num_);
    eucluster.setSearchMethod(tree_ptr);
    eucluster.extract(local_indices);

    //遍历每个聚类点簇
    //local_indices.size()代表聚类点簇的数量
    #pragma omp for
    for (size_t i = 0; i < local_indices.size(); i++)
    {
        //建立点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr sub_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            pcl::PointXYZ new_point;
            new_point.x = in_pc_ptr->points[*pit].x;
            new_point.y = in_pc_ptr->points[*pit].y;
            new_point.z = in_pc_ptr->points[*pit].z;
            sub_pc_ptr->points.push_back(new_point);
        }

        //计算尺寸
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();

        for (size_t p = 0; p < sub_pc_ptr->points.size(); p++)
        {
            if (sub_pc_ptr->points[p].x < min_x)
                min_x = sub_pc_ptr->points[p].x;
            if (sub_pc_ptr->points[p].x > max_x)
                max_x = sub_pc_ptr->points[p].x;
            if (sub_pc_ptr->points[p].y < min_y)
                min_y = sub_pc_ptr->points[p].y;
            if (sub_pc_ptr->points[p].y > max_y)
                max_y = sub_pc_ptr->points[p].y;
        }

        //floor(x)返回小于或等于x的最大整数
        //ceil(x)返回大于x的最小整数

        int num_x = ceil((max_x - min_x) / max_cluster_size_);
        int num_y = ceil((max_y - min_y) / max_cluster_size_);

        //在sub_pc_ptr中重新分配num_x*num_y个目标点云，使得每个目标点云的尺寸均满足要求
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> ptrs;
        ptrs.resize(num_x, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>(num_y));
        for (size_t nx = 0; nx < num_x; nx++)
        {
            for (size_t ny = 0; ny < num_y; ny++)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                ptrs[nx][ny] = temp_pc_ptr;
            }
        }

        for (size_t p = 0; p < sub_pc_ptr->points.size(); p++)
        {
            auto x_idx = (size_t)floor((sub_pc_ptr->points[p].x - min_x) / max_cluster_size_);
            auto y_idx = (size_t)floor((sub_pc_ptr->points[p].y - min_y) / max_cluster_size_);

            pcl::PointXYZ new_point;
            new_point.x = sub_pc_ptr->points[p].x;
            new_point.y = sub_pc_ptr->points[p].y;
            new_point.z = sub_pc_ptr->points[p].z;
            ptrs[x_idx][y_idx]->points.push_back(new_point);
        }

        //对num_x*num_y个目标拟合3D包围盒
        for (size_t nx = 0; nx < num_x; nx++)
        {
            for (size_t ny = 0; ny < num_y; ny++)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr obj_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                obj_pc_ptr = ptrs[nx][ny];

                //拟合3D包围盒
                float obj_min_x = std::numeric_limits<float>::max();
                float obj_max_x = -std::numeric_limits<float>::max();
                float obj_min_y = std::numeric_limits<float>::max();
                float obj_max_y = -std::numeric_limits<float>::max();
                float obj_min_z = std::numeric_limits<float>::max();
                float obj_max_z = -std::numeric_limits<float>::max();

                for (size_t p = 0; p < obj_pc_ptr->points.size(); p++)
                {
                    if (obj_pc_ptr->points[p].x < obj_min_x)
                        obj_min_x = obj_pc_ptr->points[p].x;
                    if (obj_pc_ptr->points[p].x > obj_max_x)
                        obj_max_x = obj_pc_ptr->points[p].x;
                    if (obj_pc_ptr->points[p].y < obj_min_y)
                        obj_min_y = obj_pc_ptr->points[p].y;
                    if (obj_pc_ptr->points[p].y > obj_max_y)
                        obj_max_y = obj_pc_ptr->points[p].y;
                    if (obj_pc_ptr->points[p].z < obj_min_z)
                        obj_min_z = obj_pc_ptr->points[p].z;
                    if (obj_pc_ptr->points[p].z > obj_max_z)
                        obj_max_z = obj_pc_ptr->points[p].z;
                }

                float phi = 0;

                float dimension_x = obj_max_x - obj_min_x;
                float dimension_y = obj_max_y - obj_min_y;
                float dimension_z = obj_max_z - obj_min_z;

                if (dimension_x < min_cluster_size_ && dimension_y < min_cluster_size_ && dimension_z < min_cluster_size_) {continue;}

                //将聚类结果转换为Marker消息类型
                visualization_msgs::Marker marker;

                //设置标记位姿
                marker.pose.position.x = (obj_min_x + obj_max_x) / 2;
                marker.pose.position.y = (obj_min_y + obj_max_y) / 2;
                marker.pose.position.z = (obj_min_z + obj_max_z) / 2;
                marker.pose.orientation.x = 0;
                marker.pose.orientation.y = 0;
                marker.pose.orientation.z = sin(0.5 * phi);
                marker.pose.orientation.w = cos(0.5 * phi);

                //设置标记尺寸
                marker.scale.x = dimension_x;
                marker.scale.y = dimension_y;
                marker.scale.z = dimension_z;

                objs.push_back(marker);
            }
        }

    }
}

void EuCluster::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                        std::vector<visualization_msgs::Marker> &objs)
{
    //根据分区，定义子云
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_ptr_array(seg_num_);
    for (size_t j = 0; j < pc_ptr_array.size(); j++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        pc_ptr_array[j] = tmp;
    }

    //遍历输入点云中的点，根据距离将不同的点划分到不同的子云中
    #pragma omp for
    for (size_t i = 0; i < in_pc_ptr->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc_ptr->points[i].x;
        current_point.y = in_pc_ptr->points[i].y;
        current_point.z = in_pc_ptr->points[i].z;

        float distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        for (size_t j = 0; j < pc_ptr_array.size(); j++)
        {
            if (distance < seg_distance_[j])
            {
                pc_ptr_array[j]->points.push_back(current_point);
                break;
            }
        }
    }

    //对各子云进行聚类
    for (size_t j = 0; j < pc_ptr_array.size(); j++) {segment(pc_ptr_array[j], cluster_distance_[j], objs);}
}

void EuCluster::crop(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr out_pc_ptr,
                     std::vector<float> edge_left,
                     std::vector<float> edge_right)
{
    pcl::ExtractIndices<pcl::PointXYZ> clipper;

    clipper.setInputCloud(in_pc_ptr);
    pcl::PointIndices indices;
    
    #pragma omp for
    for (size_t i = 0; i < in_pc_ptr->points.size(); i++)
    {
        float x = in_pc_ptr->points[i].x;
        float y = in_pc_ptr->points[i].y;
        if (y > x * x * x * edge_right[0] + x * x * edge_right[1] + x * edge_right[2] + edge_right[3] &&
            y < x * x * x * edge_left[0] + x * x * edge_left[1] + x * edge_left[2] + edge_left[3])
        {
            continue;
        }
        indices.indices.push_back(i);
    }

    clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    clipper.setNegative(true);
    clipper.filter(*out_pc_ptr);
}

void EuCluster::callback(const sensor_msgs::PointCloud2ConstPtr &in)
{
    ros::Time time_start = ros::Time::now();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*in, *current_pc_ptr);

    if (road_info_)
    {
        ros::param::get("~road_edge_left", road_edge_left_);
        ros::param::get("~road_edge_right", road_edge_right_);
        crop(current_pc_ptr, cropped_pc_ptr, road_edge_left_, road_edge_right_);
    }
    else
    {
        cropped_pc_ptr = current_pc_ptr;
    }
    
    std::vector<visualization_msgs::Marker> objs;
    cluster(cropped_pc_ptr, objs);
    
    visualization_msgs::MarkerArray msg;
    for (size_t i = 0; i < objs.size(); i++)
    {
        objs[i].header = in->header;

        //设置该标记的命名空间和ID，ID应该是独一无二的
        //具有相同命名空间和ID的标记将会覆盖前一个
        objs[i].ns = "obstacle";
        objs[i].id = i;
        
        //设置标记类型
        objs[i].type = visualization_msgs::Marker::CUBE;
        
        //设置标记行为：ADD为添加，DELETE为删除
        objs[i].action = visualization_msgs::Marker::ADD;

        //设置标记颜色，确保不透明度alpha不为0
        objs[i].color.r = 0.8f;
        objs[i].color.g = 0.0f;
        objs[i].color.b = 0.0f;
        objs[i].color.a = 0.85;

        objs[i].lifetime = ros::Duration(0.1);
        objs[i].text = ' ';

        msg.markers.push_back(objs[i]);
    }

    ros::Time time_end = ros::Time::now();

    if (show_objects_num_ || show_time_)
    {
        std::cout<<""<<std::endl;
    }

    if (show_objects_num_)
    {
        std::cout<<"size of objects:"<<objs.size()<<std::endl;
    }

    if (show_time_)
    {
        std::cout<<"cost time:"<<time_end - time_start<<"s"<<std::endl;
    }

    pub_.publish(msg);
}


