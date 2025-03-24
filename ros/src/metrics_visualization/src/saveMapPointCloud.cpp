#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

void mapSaverCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    std::cout << "[Map Saver]: Latest Map Message Obtained." << std::endl;
    
    pcl::PCLPointCloud2* pclCloud2 = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    pcl_conversions::toPCL(*cloud_msg, *pclCloud2);
    pcl::fromPCLPointCloud2(*pclCloud2, pclCloud);

    pcl::io::savePLYFileASCII("./static_map.ply", pclCloud);
    std::cerr << "[Map Saver]:" << " Saved " << pclCloud.size() << " data points to static_map.ply in the current directory." << std::endl;

    delete pclCloud2;  // 释放动态分配的内存
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "save_map_node");
	ros::NodeHandle nh;
    ros::Subscriber mapSub;

	// subscribe the map
    mapSub = nh.subscribe("/occupancy_map/voxel_map", 10, mapSaverCB);

	ros::spin();
	return 0;
}