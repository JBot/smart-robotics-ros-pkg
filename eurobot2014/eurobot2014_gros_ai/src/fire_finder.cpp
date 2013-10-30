#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>
#include <vector>


class FireFinder {
    public:
        FireFinder();

        ros::Subscriber laser_opponent_sub_;
        ros::Subscriber laser_fire_sub_;
        ros::Publisher debug_pub;
        ros::Publisher fire_pub;

    private:
        void laserOppCallback(const sensor_msgs::LaserScan::ConstPtr & laser);
        void laserFireCallback(const sensor_msgs::LaserScan::ConstPtr & laser);
        ros::NodeHandle nh;

	laser_geometry::LaserProjection projector;
	tf::TransformListener tflistener;

	pcl::PassThrough<pcl::PointXYZ> passX;
	pcl::PassThrough<pcl::PointXYZ> passY;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ> opp_cloud;
	pcl::PointCloud<pcl::PointXYZ> cloud_opp_filtered;

        uint8_t state;
};

FireFinder::FireFinder()
{
    laser_opponent_sub_ = nh.subscribe < sensor_msgs::LaserScan > ("/ROBOT/laser_opponent", 5, &FireFinder::laserOppCallback, this);
    laser_fire_sub_ = nh.subscribe < sensor_msgs::LaserScan > ("/ROBOT/laser_fire", 5, &FireFinder::laserFireCallback, this);
    debug_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/DEBUG/Fire_PointCloud", 5);
    //fire_pub = nh.advertise < pcl::PointCloud<pcl::PointXYZ> > ("/ROBOT/Fire_PointCloud", 5);

    tflistener.setExtrapolationLimit(ros::Duration(0.1));

    passX.setFilterFieldName("x");
    passX.setFilterLimits(0.05, 1.95);

    passY.setFilterFieldName("y");
    passY.setFilterLimits(-1.45, 1.45);

    //tree = ((new pcl::search::KdTree<pcl::PointXYZ>).makeShared());
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(100);


    state = 0;

}

void FireFinder::laserOppCallback(const sensor_msgs::LaserScan::ConstPtr & laser)
{
    //ROS_ERROR("!");
    sensor_msgs::PointCloud2 tmp_cloud;
    projector.transformLaserScanToPointCloud("map", *laser, tmp_cloud, tflistener);

    pcl::fromROSMsg(tmp_cloud, opp_cloud);

    passX.setInputCloud(opp_cloud.makeShared());
    passX.filter(opp_cloud);

    passY.setInputCloud(opp_cloud.makeShared());
    passY.filter(opp_cloud);


}

void FireFinder::laserFireCallback(const sensor_msgs::LaserScan::ConstPtr & laser)
{
    //ROS_ERROR("!");
    sensor_msgs::PointCloud2 tmp_cloud;
    projector.transformLaserScanToPointCloud("map", *laser, tmp_cloud, tflistener);

    debug_pub.publish(tmp_cloud);
    // Filter
    pcl::fromROSMsg(tmp_cloud, cloud);

    passX.setInputCloud(cloud.makeShared());
    passX.filter(cloud);

    passY.setInputCloud(cloud.makeShared());
    passY.filter(cloud);

    // Segmentation
    std::vector<pcl::PointIndices> cluster_indices;
    tree->setInputCloud(cloud.makeShared());

    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud.makeShared());
    ec.extract(cluster_indices);

    // Take the mean value of each cluster
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_cluster;

    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{

	double x = 0.0;
	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
		x += cloud.points[*pit].x;
		
		}

	}
    cloud_cluster.width = cloud_cluster.points.size();
    cloud_cluster.height = 1;
    cloud_cluster.is_dense = true;


}



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
     * directly, but for most command-line programs, passing argc and argv is the easiest
     * way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "Fire_Finder");
    FireFinder firefinder;

    // Refresh rate
    ros::Rate loop_rate(30);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}
