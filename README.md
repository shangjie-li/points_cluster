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
   
   <param name="min_cluster_points_num" value="5" />
   <param name="max_cluster_points_num" value="4000" />
   
   <param name="min_cluster_size" value="0.1" />
   <param name="max_cluster_size" value="10" />
   
   <param name="oriented_rectangle_fitting_distance" value="10" />
   <param name="fitting_accuracy" value="2" />
        
   <param name="seg_num" value="5" />
   <rosparam param="seg_distance" > [15, 30, 45, 60, 120] </rosparam>
   <rosparam param="cluster_distance" > [0.5, 1.0, 1.5, 2.0, 2.5] </rosparam>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_ground_topic`指明发布的聚类结果话题。
    - `oriented_rectangle_fitting_distance`为拟合带方向包络框的限制距离，单位为米。
    - `fitting_accuracy`为拟合的方位角精度，单位为度。
    - `seg_distance`为不同的聚类距离，单位为米。
    - `cluster_distance`为不同的聚类阈值。

## 运行
 - 启动`points_cluster`
   ```Shell
   roslauch points_cluster points_cluster.launch
   ```

