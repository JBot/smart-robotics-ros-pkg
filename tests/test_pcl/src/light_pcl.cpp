#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/mls.h>
#include <pcl_ros/transforms.h>


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


class LightPCL {
	public:
		LightPCL(tf::TransformListener& tf);

		ros::Subscriber pc_sub_;

		ros::Publisher simplifiedpcl_pub;

	private:
		void pcCallback(const sensor_msgs::PointCloud2::ConstPtr & pc);
		ros::NodeHandle nh;

		tf::TransformListener tflistener;
		tf::TransformListener& tf_;

};

LightPCL::LightPCL(tf::TransformListener& tf):
	tf_(tf)
{
	pc_sub_ = nh.subscribe < sensor_msgs::PointCloud2 > ("/camera/depth_registered/points", 1, &LightPCL::pcCallback, this);

	simplifiedpcl_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/kinect/simplified", 5);
}

void LightPCL::pcCallback(const sensor_msgs::PointCloud2::ConstPtr & pc)
{
	pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PCLPointCloud2::Ptr cloud_out (new pcl::PCLPointCloud2 ());

	sensor_msgs::PointCloud2 pc2;


	// Transformation into PCL type PointCloud2
	pcl_conversions::toPCL(*(pc), *(pcl_pc));

	//////////////////
	// Voxel filter //
	//////////////////
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (pcl_pc);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	// Transformation into ROS type
	//pcl::toPCLPointCloud2(*(cloud_filtered2), *(cloud_out));
	pcl_conversions::moveFromPCL(*(cloud_filtered), pc2);

	simplifiedpcl_pub.publish(pc2);
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
	ros::init(argc, argv, "lightPCL");

	tf::TransformListener listener(ros::Duration(10));
	LightPCL lightPCL(listener);

	// Refresh rate
	ros::Rate loop_rate(30);
	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}
