#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
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


class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/BEACON/laser", 10, &My_Filter::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/BEACON/filtered_pcl", 10, false);
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    pcl::PCLPointCloud2::Ptr pcl_pc(new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud_out (new pcl::PCLPointCloud2 ());
    sensor_msgs::PointCloud2 pc2;
    std::vector<int> inliers;


    if(!tfListener_.waitForTransform(scan->header.frame_id, "/world", scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
        ros::Duration(1.0))){
    //std::cout << "a0" << std::endl;
     return;
  }

    projector_.transformLaserScanToPointCloud("/world", *scan, cloud, tfListener_);

    // Transformation into PCL type PointCloud2
    pcl_conversions::toPCL((cloud), *(pcl_pc));

    // Transformation into PCL type PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*(pcl_pc), *(pcl_cloud));


  ////////////////////////
  // PassThrough filter //
  ////////////////////////
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pcl_cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1.5, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*final);

  pass.setInputCloud (final);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.0, 2.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*final);





    // Transformation into ROS type
    pcl::toPCLPointCloud2(*(final), *(cloud_out));
    pcl_conversions::moveFromPCL(*(cloud_out), pc2);


    point_cloud_publisher_.publish(pc2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");

    My_Filter filter;

    ros::spin();

    return 0;
}

