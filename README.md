# points_cluster

ROS package for clustering points

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/points_cluster.git
   cd ..
   catkin_make
   ```
   
## 参数配置
 - 修改`points_cluster/launch/points_cluster.launch`
   ```Shell
   <param name="sub_topic" value="/rslidar_points_no_ground" />
   <param name="pub_topic" value="/clustered_bounding_boxs" />
        
   <param name="min_cluster_size" value="20" />
   <param name="max_cluster_size" value="500" />
        
   <param name="seg_num" value="5" />
   <rosparam param="seg_distance" > [15, 30, 45, 60, 120] </rosparam>
   <rosparam param="cluster_distance" > [0.5, 1.0, 1.5, 2.0, 2.5] </rosparam>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_ground_topic`指明发布的聚类结果话题。
    - `seg_distance`为不同的聚类距离。
    - `cluster_distance`为不同的聚类阈值。

## 运行
 - 启动`points_cluster`
   ```Shell
   roslauch points_cluster points_cluster.launch
   ```

