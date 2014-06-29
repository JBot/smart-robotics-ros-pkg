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


class PlaneFinder {
    public:
        PlaneFinder(tf::TransformListener& tf);

        ros::Subscriber pc_sub_;

        ros::Publisher debug_pub;
        ros::Publisher debug2_pub;

    private:
        void pcCallback(const sensor_msgs::PointCloud2::ConstPtr & pc);
        ros::NodeHandle nh;

	tf::TransformListener tflistener;
	tf::TransformListener& tf_;

};

PlaneFinder::PlaneFinder(tf::TransformListener& tf):
        tf_(tf)
{
    pc_sub_ = nh.subscribe < sensor_msgs::PointCloud2 > ("/camera/depth_registered/points", 1, &PlaneFinder::pcCallback, this);

    debug_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/DEBUG/PointCloud", 5);
    debug2_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/DEBUG/PointCloud2", 5);


    //passY.setFilterLimitsNegative (true);
    //passX.setFilterLimitsNegative (true);


}

void PlaneFinder::pcCallback(const sensor_msgs::PointCloud2::ConstPtr & pc)
{


  pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PCLPointCloud2::Ptr cloud_out (new pcl::PCLPointCloud2 ());
  
  sensor_msgs::PointCloud2 pc2;

  double height = -0.5;


  // Transformation into PCL type PointCloud2
  pcl_conversions::toPCL(*(pc), *(pcl_pc));

  //////////////////
  // Voxel filter //
  //////////////////
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (pcl_pc);
  sor.setLeafSize (0.003f, 0.003f, 0.003f);
  sor.filter (*cloud_filtered);

  // Transformation into ROS type
  //pcl::toPCLPointCloud2(*(cloud_filtered2), *(cloud_out));
  //pcl_conversions::moveFromPCL(*(cloud_filtered), pc2);

  //debug2_pub.publish(pc2);

		
  // Transformation into PCL type PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*(cloud_filtered), *(cloud_filtered1));

  if(pcl_ros::transformPointCloud("map", *(cloud_filtered1), *(cloud_filtered2), tf_)) 
  {

  ////////////////////////
  // PassThrough filter //
  ////////////////////////
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_filtered2);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.003, 0.9);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered2);

  pass.setInputCloud (cloud_filtered2);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.5, 0.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered2);
  
  /////////////////////////
  // Planar segmentation //
  /////////////////////////
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud_filtered2->points.size ();
  // While 50% of the original cloud is still there
  while (cloud_filtered2->points.size () > 0.5 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered2);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    if( (fabs(coefficients->values[0]) < 0.02) && 
	(fabs(coefficients->values[1]) < 0.02) && 
	(fabs(coefficients->values[2]) > 0.9) )
    {

    	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

 	height = coefficients->values[3];

    	// Extract the inliers
    	extract.setInputCloud (cloud_filtered2);
    	extract.setIndices (inliers);
    	extract.setNegative (false);
    	extract.filter (*cloud_filtered3);

    	// Transformation into ROS type
    	//pcl::toPCLPointCloud2(*(cloud_filtered3), *(cloud_out));
    	//pcl_conversions::moveFromPCL(*(cloud_out), pc2);

    	//debug_pub.publish(pc2);
    }
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered2.swap (cloud_f);
    i++;
    
  }

/*
  pass.setInputCloud (cloud_filtered2);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (height, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered2);
*/

  // Transformation into ROS type
  //pcl::toPCLPointCloud2(*(cloud_filtered2), *(cloud_out));
  //pcl_conversions::moveFromPCL(*(cloud_out), pc2);

  //debug_pub.publish(pc2);

  /////////////////////////////////
  // Statistical Outlier Removal //
  /////////////////////////////////
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_filtered2);
  sor.setMeanK (200);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered2);


  //////////////////////////////////
  // Euclidian Cluster Extraction //  
  //////////////////////////////////

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered2);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (500);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered2);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster1->points.push_back (cloud_filtered2->points[*pit]); 
    cloud_cluster1->width = cloud_cluster1->points.size ();
    cloud_cluster1->height = 1;
    cloud_cluster1->is_dense = true;
    cloud_cluster1->header.frame_id = "/map";




    if(j == 0) {
/*
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeSM (new pcl::search::KdTree<pcl::PointXYZRGB>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointXYZRGB> mls_pointsSM;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mlsSM;
 
  mlsSM.setComputeNormals (true);

  // Set parameters
  mlsSM.setInputCloud (cloud_cluster1);
  mlsSM.setPolynomialFit (true);
  mlsSM.setSearchMethod (treeSM);
  mlsSM.setSearchRadius (0.03);

  // Reconstruct
  mlsSM.process (mls_pointsSM);
*/
	// Transformation into ROS type
  	pcl::toPCLPointCloud2(*(cloud_cluster1), *(cloud_out));
  	//pcl::toPCLPointCloud2(mls_pointsSM, *(cloud_out));
  	pcl_conversions::moveFromPCL(*(cloud_out), pc2);

  	debug_pub.publish(pc2);
    }
    else {
  	// Transformation into ROS type
  	pcl::toPCLPointCloud2(*(cloud_cluster1), *(cloud_out));
 	pcl_conversions::moveFromPCL(*(cloud_out), pc2);

  	debug2_pub.publish(pc2);
    }
    j++;
  }

  // Transformation into ROS type
  //pcl::toPCLPointCloud2(*(cloud_filtered2), *(cloud_out));
  //pcl_conversions::moveFromPCL(*(cloud_out), pc2);

  //debug2_pub.publish(pc2);
  //debug2_pub.publish(*pc_map);

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
    ros::Rate loop_rate(30);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}
