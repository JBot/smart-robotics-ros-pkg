// ROS includes
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <laser_geometry/laser_geometry.h>

// PCL includes
#include <pcl/point_types.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/surface/mls.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>

#include <visualization_msgs/MarkerArray.h>

// Standard includes
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat

#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>
#include <vector>


class PlaneFinder {
	public:
		PlaneFinder(tf::TransformListener& tf);

		ros::Subscriber pc_sub_;
		ros::Subscriber voxel_sub_;

		ros::Publisher debug_pub;
		ros::Publisher debug2_pub;
		ros::Publisher debug3_pub;

		ros::Publisher normals_marker_array_publisher_;

	private:
		// Fonctions de callback
		void pcCallback(const sensor_msgs::PointCloud2::ConstPtr & pc);
		void voxelCallback(const std_msgs::Float32::ConstPtr & size);
		ros::NodeHandle nh;

		tf::TransformListener tflistener;
		tf::TransformListener& tf_;

		// Paramètre des filtres
		double voxel_size_x;
		double voxel_size_y;
		double voxel_size_z;
		int max_iterations;
		double distance_tresh;
		int sor_filter;
		int sor_filter2;
		double normal_radius;
		double normal_z_max;
		double pt_x;
		double pt_y;
		int min_neigh;
		double radius_ror;
		double curv_tresh;
		double z_min;

		// Activation des filtres
		bool enable_paththrough;
		bool enable_sor1;
		bool enable_planar_seg;
		bool enable_sor2;
		bool enable_normal;
		bool enable_planar_proj;
		bool enable_ror;

		visualization_msgs::MarkerArray normals_marker_array_msg_;
		unsigned int lastsize;

		double height;
};

PlaneFinder::PlaneFinder(tf::TransformListener& tf):
	tf_(tf)
{
	ros::NodeHandle nhp("~");

	// Paramètres des filtres
	nh.param<double>("voxel_size_x", voxel_size_x, 0.05);
	nh.param<double>("voxel_size_y", voxel_size_y, 0.05);
	nh.param<double>("voxel_size_z", voxel_size_z, 0.05);
	nhp.param<double>("distance_tresh", distance_tresh, 0.05);
	nhp.param<int>("max_iterations", max_iterations, 800);
	nhp.param<int>("sor_filter", sor_filter, 15);
	nhp.param<int>("sor_filter2", sor_filter2, 10);
	nhp.param<double>("normal_radius", normal_radius, 0.05);
	nhp.param<double>("normal_z_max", normal_z_max, 0.2);
	nhp.param<double>("pt_x", pt_x, 3.5);
	nhp.param<double>("pt_y", pt_y, 1.0);
	nhp.param<int>("min_neigh", min_neigh, 2);
	nhp.param<double>("radius_ror", radius_ror, 0.05);
	nhp.param<double>("z_min", z_min, 0.1);

	// Activation des filtres
	nhp.param<bool>("enable_paththrough", enable_paththrough, true);
	nhp.param<bool>("enable_sor1", enable_sor1, true);
	nhp.param<bool>("enable_planar_seg", enable_planar_seg, true);
	nhp.param<bool>("enable_sor2", enable_sor2, true);
	nhp.param<bool>("enable_normal", enable_normal, false);
	nhp.param<bool>("enable_planar_proj", enable_planar_proj, true);
	nhp.param<bool>("enable_ror", enable_ror, false);

	pc_sub_ = nh.subscribe < sensor_msgs::PointCloud2 > ("/swissranger/pointcloud2_raw", 1, &PlaneFinder::pcCallback, this);
	voxel_sub_ = nh.subscribe < std_msgs::Float32 > ("/swissranger/voxel_size", 1, &PlaneFinder::voxelCallback, this);

	debug_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/DEBUG/PointCloud", 1);
	debug2_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/DEBUG/PointCloud2", 1);
	debug3_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/DEBUG/PointCloud3", 1);

	normals_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("normals_marker_array", 100);

	lastsize = 0;

	height = 0.0;
	curv_tresh = 0.25;
}

void PlaneFinder::voxelCallback(const std_msgs::Float32::ConstPtr & size)
{
	curv_tresh = size->data;
}

void PlaneFinder::pcCallback(const sensor_msgs::PointCloud2::ConstPtr & pc)
{


	pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered4 (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered5 (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCLPointCloud2::Ptr cloud_out (new pcl::PCLPointCloud2 ());

	sensor_msgs::PointCloud2 pc2;


	// Transformation into PCL type PointCloud2
	pcl_conversions::toPCL(*(pc), *(pcl_pc));



	//////////////////
	// Voxel filter //
	//////////////////
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (pcl_pc);
	sor.setLeafSize (voxel_size_x, voxel_size_y, voxel_size_z);
	sor.filter (*cloud_filtered);

	// Transformation into ROS type
	//pcl::toPCLPointCloud2(*(cloud_filtered2), *(cloud_out));
	pcl_conversions::moveFromPCL(*(cloud_filtered), pc2);

	//debug2_pub.publish(pc2);


	pcl_conversions::toPCL((pc2), *(pcl_pc));



	// Transformation into PCL type PointCloud<pcl::PointXYZRGB>
	pcl::fromPCLPointCloud2(*(pcl_pc), *(cloud_filtered1));
	//pcl::fromPCLPointCloud2(*(pcl_pc), *(cloud_filtered1));


	////////////////////////////////////////////////////////
	// Conversion de repères (de camera vers map globale) //
	////////////////////////////////////////////////////////
	if(pcl_ros::transformPointCloud("map", *(cloud_filtered1), *(cloud_filtered2), tf_))
	{

		////////////////////////
		// PassThrough filter //
		////////////////////////
		pcl::PassThrough<pcl::PointXYZ> pass;
		if(enable_paththrough) {
			pass.setInputCloud (cloud_filtered2);
			pass.setFilterFieldName ("x");
			pass.setFilterLimits (1.2, pt_x);
			//pass.setFilterLimitsNegative (true);
			pass.filter (*cloud_filtered2);
			pass.setInputCloud (cloud_filtered2);
			pass.setFilterFieldName ("y");
			pass.setFilterLimits (-pt_y, pt_y);
			//pass.setFilterLimitsNegative (true);
			pass.filter (*cloud_filtered2);
		}


		/////////////////////////////////
		// Statistical Outlier Removal //
		/////////////////////////////////
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorr;
		if(enable_sor1) {
			sorr.setInputCloud (cloud_filtered2);
			sorr.setMeanK (sor_filter);
			sorr.setStddevMulThresh (1.0);
			sorr.filter (*cloud_filtered2);
		}

		pass.setInputCloud (cloud_filtered2);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (z_min, 10.0);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_filtered5);

		if(cloud_filtered5->points.size() > 0) {
			// Obstacle found
			cloud_filtered2 = cloud_filtered5;

		}
		else {

			// Transformation into ROS type
			//pcl::toPCLPointCloud2(*(cloud_filtered2), *(cloud_out));
			//pcl_conversions::moveFromPCL(*(cloud_out), pc2);

			//debug_pub.publish(pc2);


			///////////////////////
			// Normal Estimation //
			///////////////////////
			if(enable_normal) {
				/*
				   pcl::search::Search<pcl::PointXYZ>::Ptr treetree;
				   if (cloud_filtered2->isOrganized ())
				   {
				   treetree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
				   }
				   else
				   {
				   treetree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
				   }

				// Set the input pointcloud for the search tree
				treetree->setInputCloud (cloud_filtered2);

				// Create the normal estimation class, and pass the input dataset to it
				pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> nene;
				nene.setInputCloud (cloud_filtered2);

				// Create an empty kdtree representation, and pass it to the normal estimation object.
				// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
				nene.setSearchMethod (treetree);

				nene.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

				// Output datasets
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);


				pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

				nene.setRadiusSearch (0.03);
				nene.compute (*normals_small_scale);

				// calculate normals with the large scale
				//cout << "Calculating normals for scale..." << scale2 << endl;
				pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

				nene.setRadiusSearch (0.1);
				nene.compute (*normals_large_scale);

				// Create output cloud for DoN results
				pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
				pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud_filtered2, *doncloud);

				//cout << "Calculating DoN... " << endl;
				// Create DoN operator
				pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
				don.setInputCloud (cloud_filtered2);
				don.setNormalScaleLarge (normals_large_scale);
				don.setNormalScaleSmall (normals_small_scale);

				if (!don.initCompute ())
				{
				std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
				exit (EXIT_FAILURE);
				}

				// Compute DoN
				don.computeFeature (*doncloud);

				// Filter by magnitude
				//cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

				// Build the condition for filtering
				pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
				new pcl::ConditionOr<pcl::PointNormal> ()
				);
				range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
				new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::LT, curv_tresh))
				);
				// Build the filter
				pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
				condrem.setInputCloud (doncloud);

				pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

				// Apply filter
				condrem.filter (*doncloud_filtered);

				*/			//doncloud = doncloud_filtered;
				/*			cloud_filtered4 = doncloud_filtered;
				 */
				/*
				   unsigned int size = doncloud_filtered->height * doncloud_filtered->width;
				   cloud_filtered4->header.frame_id = "map";
				   cloud_filtered4->header.stamp = cloud_filtered2->header.stamp;

				   cloud_filtered4->points.resize(size);

				   for (unsigned int i = 0; i < size; ++i)
				   {
				//ROS_INFO(": %f %f %f", doncloud_filtered->points[i].x, doncloud_filtered->points[i].y, doncloud_filtered->points[i].z);

				cloud_filtered4->points[i].x = doncloud_filtered->points[i].x;
				cloud_filtered4->points[i].y = doncloud_filtered->points[i].y;
				cloud_filtered4->points[i].z = doncloud_filtered->points[i].z;

				}

				 */


				pcl::search::Search<pcl::PointXYZ>::Ptr treetree;
				if (cloud_filtered2->isOrganized ())
				{
					treetree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
					//std::cout << "organized" << std::endl;
				}
				else
				{
					treetree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
					//std::cout << "not organized" << std::endl;
				}

				// Set the input pointcloud for the search tree
				treetree->setInputCloud (cloud_filtered2);
				/*
				// Create the normal estimation class, and pass the input dataset to it
				pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> nene;
				nene.setInputCloud (cloud_filtered2);

				// Create an empty kdtree representation, and pass it to the normal estimation object.
				// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
				nene.setSearchMethod (treetree);

				nene.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());


				// Use all neighbors in a sphere of radius 3cm
				nene.setRadiusSearch (normal_radius);

				// Output datasets
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);

				// Compute the features
				nene.compute (*cloud_normals);
				 */


				// Init object (second point type is for the normals, even if unused)
				pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

				mls.setComputeNormals (true);

				// Set parameters
				mls.setInputCloud (cloud_filtered2);
				mls.setPolynomialFit (true);
				mls.setSearchMethod (treetree);
				mls.setSearchRadius (normal_radius);

				// Output datasets
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);

				// Reconstruct
				mls.process (*cloud_normals);


				/*
				   unsigned int size = cloud_normals->height * cloud_normals->width;
				   cloud_filtered4->header.frame_id = "map";
				   cloud_filtered4->header.stamp = cloud_filtered2->header.stamp;

				   cloud_filtered4->points.resize(size);

				   for (unsigned int i = 0; i < size; ++i)
				   {
				//ROS_INFO(": %f %f %f", doncloud_filtered->points[i].x, doncloud_filtered->points[i].y, doncloud_filtered->points[i].z);

				cloud_filtered4->points[i].x = cloud_normals->points[i].x;
				cloud_filtered4->points[i].y = cloud_normals->points[i].y;
				cloud_filtered4->points[i].z = cloud_normals->points[i].z;

				}
				 */




				unsigned int size = cloud_normals->height * cloud_normals->width;

				normals_marker_array_msg_.markers.resize(size);
				unsigned int j = 0;
				for (unsigned int i = 0; i < size; ++i)
				{
					geometry_msgs::Point pos;
					//ROS_INFO(": %f %f %f", cloud_normals->points[i].x, cloud_normals->points[i].y, cloud_normals->points[i].z);
					//ROS_INFO(": %f %f %f", cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z);
					//ROS_INFO(": %f ", cloud_normals->points[i].curvature);
					//axis-angle rotation
					if( isnan(cloud_normals->points[i].normal[0]) ) {
					}
					else {
						if( (fabs(cloud_normals->points[i].normal_z) < normal_z_max) && 
								( (fabs(cloud_normals->points[i].normal_x) > 0.5) || 
								  ( (fabs(cloud_normals->points[i].normal_y) > 0.5) ) ) ) {
							//if( (fabs(cloud_normals->points[i].curvature) > normal_z_max) ) { 

							//ROS_INFO(": %f %f %f %f", cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z, cloud_normals->points[i].curvature);
							//ROS_INFO(": %f %f %f", cloud_normals->points[i].x, cloud_normals->points[i].y, cloud_normals->points[i].z);

							cloud_filtered4->header.frame_id = "map";
							cloud_filtered4->header.stamp = cloud_filtered2->header.stamp;

							cloud_filtered4->points.resize(size);

							cloud_filtered4->points[j].x = cloud_normals->points[i].x;
							cloud_filtered4->points[j].y = cloud_normals->points[i].y;
							cloud_filtered4->points[j].z = cloud_normals->points[i].z;


							j++;

							/*
							//ROS_INFO(": %f %f %f", cloud_normals->points[i-1].x, cloud_normals->points[i-1].y, cloud_normals->points[i-1].z);
							pos.x = cloud_filtered2->points[i].x;
							pos.y = cloud_filtered2->points[i].y;
							pos.z = cloud_filtered2->points[i].z;
							normals_marker_array_msg_.markers[j].pose.position = pos;
							//ROS_INFO(": %f %f %f", cloud_normals->points[i].normal[0], cloud_normals->points[i].normal[1], cloud_normals->points[i].normal[2]);
							tf::Vector3 axis(cloud_normals->points[i].normal_x,cloud_normals->points[i].normal_y,cloud_normals->points[i].normal_z);
							tf::Vector3 marker_axis(1, 0, 0);
							tf::Quaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
							geometry_msgs::Quaternion quat_msg;
							tf::quaternionTFToMsg(qt, quat_msg);
							normals_marker_array_msg_.markers[j].pose.orientation = quat_msg;

							normals_marker_array_msg_.markers[j].header.frame_id = "map";//cloud_normals->header.frame_id;
							normals_marker_array_msg_.markers[j].header.stamp = ros::Time::now();//pcl_cloud.header.stamp;
							normals_marker_array_msg_.markers[j].id = j;
							normals_marker_array_msg_.markers[j].ns = "Normals";
							normals_marker_array_msg_.markers[j].color.r = 1.0f;
							normals_marker_array_msg_.markers[j].color.g = 0.0f;
							normals_marker_array_msg_.markers[j].color.b = 0.0f;
							normals_marker_array_msg_.markers[j].color.a = 0.5f;
							normals_marker_array_msg_.markers[j].lifetime = ros::Duration(1);
							normals_marker_array_msg_.markers[j].type = visualization_msgs::Marker::ARROW;
							normals_marker_array_msg_.markers[j].scale.x = 0.01;
							normals_marker_array_msg_.markers[j].scale.y = 0.01;
							normals_marker_array_msg_.markers[j].scale.z = 0.01;

							normals_marker_array_msg_.markers[j].action = visualization_msgs::Marker::ADD;

							j++;
							 */
						}
						}
					}

					cloud_filtered4->points.resize(j);

					/*
					   normals_marker_array_msg_.markers.resize(j);
					   size = j;

					   if (lastsize > size) {
					   for (unsigned int i = size; i < lastsize; ++i) {
					   normals_marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
					   }
					   }
					   lastsize = size;


					   normals_marker_array_publisher_.publish(normals_marker_array_msg_);
					 */

					/////////////////////////////////
					// Statistical Outlier Removal //
					/////////////////////////////////
					if(enable_ror) {
						sorr.setInputCloud (cloud_filtered4);
						sorr.setMeanK (sor_filter2);
						sorr.setStddevMulThresh (1.0);
						sorr.filter (*cloud_filtered4);
					}

					////////////////////////////
					// Radius Outlier Removal //
					////////////////////////////
					if(enable_ror) {
						pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
						// build the filter
						outrem.setInputCloud(cloud_filtered4);
						outrem.setRadiusSearch(radius_ror);
						outrem.setMinNeighborsInRadius (min_neigh);
						// apply filter
						outrem.filter (*cloud_filtered4);
					}




				}


				// Transformation into ROS type
				pcl::toPCLPointCloud2(*(cloud_filtered4), *(cloud_out));
				//pcl::toPCLPointCloud2(*(cloud_normals), *(cloud_out));
				pcl_conversions::moveFromPCL(*(cloud_out), pc2);

				debug_pub.publish(pc2);



				/////////////////////////
				// Planar segmentation //
				/////////////////////////
				if(enable_planar_seg) {
					pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
					pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
					// Create the segmentation object
					pcl::SACSegmentation<pcl::PointXYZ> seg;
					// Optional
					seg.setOptimizeCoefficients (true);
					// Mandatory
					seg.setModelType (pcl::SACMODEL_PLANE);
					seg.setMethodType (pcl::SAC_RANSAC);
					seg.setMaxIterations (max_iterations);
					seg.setDistanceThreshold (distance_tresh);

					// Create the filtering object
					pcl::ExtractIndices<pcl::PointXYZ> extract;

					int i = 0, nr_points = (int) cloud_filtered2->points.size ();

					// Segment the largest planar component from the remaining cloud
					seg.setInputCloud (cloud_filtered2);
					seg.segment (*inliers, *coefficients);
					if (inliers->indices.size () == 0)
					{
						std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
						//break;
					}
					/*
					   std::cerr << "Model coefficients: " << coefficients->values[0] << " "
					   << coefficients->values[1] << " "
					   << coefficients->values[2] << " "
					   << coefficients->values[3] << std::endl;
					 */
					height = coefficients->values[3];

					// Extract the inliers
					extract.setInputCloud (cloud_filtered2);
					extract.setIndices (inliers);
					extract.setNegative (false);
					extract.filter (*cloud_filtered3);

					if( i == 0 )
					{
						// Transformation into ROS type
						pcl::toPCLPointCloud2(*(cloud_filtered3), *(cloud_out));
						pcl_conversions::moveFromPCL(*(cloud_out), pc2);

						debug2_pub.publish(pc2);
					}
					// Create the filtering object
					extract.setNegative (true);
					extract.filter (*cloud_f);
					cloud_filtered2.swap (cloud_f);
					i++;
				}


				pass.setInputCloud (cloud_filtered2);
				pass.setFilterFieldName ("z");
				pass.setFilterLimits (-height, 3.0);
				//pass.setFilterLimitsNegative (true);
				pass.filter (*cloud_filtered2);



				/////////////////////////////////
				// Statistical Outlier Removal //
				/////////////////////////////////
				if(enable_sor2) {
					sorr.setInputCloud (cloud_filtered2);
					sorr.setMeanK (sor_filter2);
					sorr.setStddevMulThresh (1.0);
					sorr.filter (*cloud_filtered2);
				}

				////////////////////////////
				// Radius Outlier Removal //
				////////////////////////////
				if(enable_ror) {
					pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
					// build the filter
					outrem.setInputCloud(cloud_filtered2);
					outrem.setRadiusSearch(radius_ror);
					outrem.setMinNeighborsInRadius (min_neigh);
					// apply filter
					outrem.filter (*cloud_filtered2);
				}


				/////////////////////////
				// Planar projection   //
				/////////////////////////
				if(enable_planar_proj) {
					// Create a set of planar coefficients with X=Y=0,Z=1
					pcl::ModelCoefficients::Ptr plcoefficients (new pcl::ModelCoefficients ());
					plcoefficients->values.resize (4);
					plcoefficients->values[0] = plcoefficients->values[1] = 0;
					plcoefficients->values[2] = 1.0;
					plcoefficients->values[3] = -0.5;

					// Create the filtering object
					pcl::ProjectInliers<pcl::PointXYZ> proj;
					proj.setModelType (pcl::SACMODEL_PLANE);
					proj.setInputCloud (cloud_filtered2);
					proj.setModelCoefficients (plcoefficients);
					proj.filter (*cloud_filtered2);
				}

			}

			// Transformation into ROS type
			pcl::toPCLPointCloud2(*(cloud_filtered2), *(cloud_out));
			pcl_conversions::moveFromPCL(*(cloud_out), pc2);

			debug3_pub.publish(pc2);

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
		ros::init(argc, argv, "Plane_Finder");

		tf::TransformListener listener(ros::Duration(10));
		PlaneFinder planefinder(listener);

		// Refresh rate
		ros::Rate loop_rate(50);
		while (ros::ok()) {

			ros::spinOnce();
			loop_rate.sleep();
		}

		ros::Duration(2.0).sleep();

		ros::shutdown();
	}
