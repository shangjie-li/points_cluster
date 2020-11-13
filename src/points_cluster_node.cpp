#include "points_cluster_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_cluster");

    ros::NodeHandle nh("~");
    
    omp_set_num_threads(4);

    EuClusterCore core(nh);
    
    return 0;
    
}
