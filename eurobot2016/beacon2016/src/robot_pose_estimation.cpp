#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

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


class My_Filter {
     public:
        My_Filter();
        void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
        void broadcast(void);
     private:
        ros::NodeHandle node_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber point_cloud_sub_;

  	tf::StampedTransform t;
        tf::TransformBroadcaster broadcaster;


};

My_Filter::My_Filter(){
        point_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2> ("/BEACON/filtered_pcl", 10, &My_Filter::pclCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/BEACON/pcl_debug2", 10, false);


}

void My_Filter::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
    pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud_out (new pcl::PCLPointCloud2 ());
    sensor_msgs::PointCloud2 pc2;
    std::vector<int> inliers;


    // Transformation into PCL type PointCloud2
    pcl_conversions::toPCL((*cloud), *(pcl_pc));

    // Transformation into PCL type PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*(pcl_pc), *(pcl_cloud));



  /////////////////////////////////
  // Statistical Outlier Removal //
  /////////////////////////////////
/*  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (pcl_cloud);
  sor.setMeanK (200);
  sor.setStddevMulThresh (1.0);
  sor.filter (*final);
*/

  //////////////////////////////////
  // Euclidian Cluster Extraction //  
  //////////////////////////////////

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pcl_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (5);
  ec.setMaxClusterSize (50);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pcl_cloud);
  ec.extract (cluster_indices);


   std::cout << cluster_indices.size() << std::endl;


  double x = 0.0;
  double y = 0.0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    x = 0.0;
    y = 0.0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    {
	x +=pcl_cloud->points[*pit].x;
	y +=pcl_cloud->points[*pit].y;
    }
  
  x = x / it->indices.size();
  y = y / it->indices.size();

  std::cout << "x : " << x << " y : " << y << " size : " << it->indices.size() << std::endl;

  }



    // Transformation into ROS type
    pcl::toPCLPointCloud2(*(pcl_cloud), *(cloud_out));
    pcl_conversions::moveFromPCL(*(cloud_out), pc2);


    point_cloud_publisher_.publish(pc2);
}

void My_Filter::broadcast(void)
{
/*
	t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, 0.0)),
                                ros::Time::now(), "/beacon_link", "/beacon_laser_link");

    	t.stamp_ = ros::Time::now();
    	broadcaster.sendTransform(t);
*/
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter2");

    My_Filter filter;

    // Refresh rate
    ros::Rate loop_rate(100);                                // 35 with bluetooth


    while (ros::ok()) {
    	ros::spinOnce();
        loop_rate.sleep();

        //filter.broadcast();
    }

    ros::Duration(2.0).sleep();

    ros::spin();

    return 0;
}

