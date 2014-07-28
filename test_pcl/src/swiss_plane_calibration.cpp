#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
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


class KinectPlaneCalibration {
	public:
		KinectPlaneCalibration(tf::TransformListener& tf);

		ros::Subscriber pc_sub_;
		ros::Subscriber start_calib_sub_;

		ros::Publisher calib_pub;

	private:
		void pcCallback(const sensor_msgs::PointCloud2::ConstPtr & pc);
		void startCallback(const std_msgs::Empty::ConstPtr & empty);
		ros::NodeHandle nh;

		tf::TransformListener tflistener;
		tf::TransformListener& tf_;

		char go_calib;

};

KinectPlaneCalibration::KinectPlaneCalibration(tf::TransformListener& tf):
	tf_(tf)
{
	pc_sub_ = nh.subscribe < sensor_msgs::PointCloud2 > ("/swissranger/pointcloud2_raw", 1, &KinectPlaneCalibration::pcCallback, this);
	start_calib_sub_ = nh.subscribe < std_msgs::Empty > ("/CALIB/start_calib_kinect", 1, &KinectPlaneCalibration::startCallback, this);

	calib_pub = nh.advertise < geometry_msgs::Quaternion > ("/CALIB/PlanModel", 5);

	go_calib = 0;
}

void KinectPlaneCalibration::startCallback(const std_msgs::Empty::ConstPtr & empty)
{
	go_calib = 1;
}

void KinectPlaneCalibration::pcCallback(const sensor_msgs::PointCloud2::ConstPtr & pc)
{

	if( go_calib == 1 ) {
		go_calib = 0;

		pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2);
		pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PCLPointCloud2::Ptr cloud_out (new pcl::PCLPointCloud2 ());

		sensor_msgs::PointCloud2 pc2;

		geometry_msgs::Quaternion quat;

		// Transformation into PCL type PointCloud2
		pcl_conversions::toPCL(*(pc), *(pcl_pc));

		//////////////////
		// Voxel filter //
		//////////////////
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (pcl_pc);
		sor.setLeafSize (0.03f, 0.03f, 0.03f);
		sor.filter (*cloud_filtered);


        pcl_conversions::moveFromPCL(*(cloud_filtered), pc2);

        //debug2_pub.publish(pc2);

        pcl_conversions::toPCL((pc2), *(pcl_pc));


        // Transformation into PCL type PointCloud<pcl::PointXYZRGB>
        pcl::fromPCLPointCloud2(*(pcl_pc), *(cloud_filtered1));


		if(pcl_ros::transformPointCloud("map", *(cloud_filtered1), *(cloud_filtered2), tf_)) 
		{

			////////////////////////
			// PassThrough filter //
			////////////////////////
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud (cloud_filtered2);
			pass.setFilterFieldName ("x");
			pass.setFilterLimits (-0.003, 3.0);
			//pass.setFilterLimitsNegative (true);
			pass.filter (*cloud_filtered2);
			pass.setInputCloud (cloud_filtered2);
			pass.setFilterFieldName ("y");
			pass.setFilterLimits (-1.0, 1.0);
			//pass.setFilterLimitsNegative (true);
			pass.filter (*cloud_filtered2);

			/////////////////////////
			// Planar segmentation //
			/////////////////////////
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			//seg.setMaxIterations (1000);
			seg.setDistanceThreshold (0.02);

			// Create the filtering object
			pcl::ExtractIndices<pcl::PointXYZ> extract;

			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud_filtered2);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			}

			std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
				<< coefficients->values[1] << " "
				<< coefficients->values[2] << " " 
				<< coefficients->values[3] << std::endl;


			quat.x = coefficients->values[0];
			quat.y = coefficients->values[1];
			quat.z = coefficients->values[2];
			quat.w = coefficients->values[3];

			calib_pub.publish(quat);

		}
	}

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
	ros::init(argc, argv, "Kinect_Plane_Calibration");

	tf::TransformListener listener(ros::Duration(10));
	KinectPlaneCalibration planefinder(listener);

	// Refresh rate
	ros::Rate loop_rate(50);
	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}
