#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
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

/*
#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_controllers/SetComplianceSlope.h"
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_msgs/MotorStateList.h"
*/

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


#define TRANSITION 	0
#define NORMAL 		1
#define BOTTOM 		2
#define MID    		3

#define TOP_SERVO	(0.35)
#define MID_SERVO	(-0.73)
#define BOT_SERVO	(-1.08)


class FireFinder {
    public:
        FireFinder(tf::TransformListener& tf);

        ros::Subscriber laser_opponent_sub_;
        ros::Subscriber find_fire_sub_;

        ros::Publisher debug_pub;
        ros::Publisher low_pub;
        ros::Publisher mid_pub;
        ros::Publisher fire_pub;
        ros::Publisher laser_opp_pub;

	ros::Publisher joint_pub;


    private:
        void laserOppCallback(const sensor_msgs::LaserScan::ConstPtr & laser);
        void findFireCallback(const std_msgs::Empty::ConstPtr & empty);
        ros::NodeHandle nh;

	laser_geometry::LaserProjection projector;
	tf::TransformListener tflistener;
	tf::TransformListener& tf_;

	pcl::PassThrough<pcl::PointXYZ> passX;
	pcl::PassThrough<pcl::PointXYZ> passY;

	pcl::PassThrough<pcl::PointXYZ> passX1;
	pcl::PassThrough<pcl::PointXYZ> passY1;

	pcl::PassThrough<pcl::PointXYZ> passX2;
	pcl::PassThrough<pcl::PointXYZ> passY2;

	pcl::PassThrough<pcl::PointXYZ> passX3;
	pcl::PassThrough<pcl::PointXYZ> passY3;


	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	std::vector<pcl::PointIndices> cluster_indices;

	pcl::PointCloud<pcl::PointXYZ> cloud_low;
	pcl::PointCloud<pcl::PointXYZ> cloud_mid;
	pcl::PointCloud<pcl::PointXYZ> opp_cloud;
	pcl::PointCloud<pcl::PointXYZ> cloud_opp_filtered;
	pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
	pcl::PointCloud<pcl::PointXYZ> tmp_pcloud;

	sensor_msgs::LaserScan save_opp_laser;

        uint8_t state;
};

FireFinder::FireFinder(tf::TransformListener& tf):
        tf_(tf)
{
    laser_opponent_sub_ = nh.subscribe < sensor_msgs::LaserScan > ("/ROBOT/laser_opponent", 1, &FireFinder::laserOppCallback, this);
    find_fire_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/find_fire", 1, &FireFinder::findFireCallback, this);

    debug_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/DEBUG/Fire_PointCloud", 5);
    low_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/ROBOT/low_PointCloud", 5);
    mid_pub = nh.advertise < sensor_msgs::PointCloud2 > ("/ROBOT/mid_PointCloud", 5);
    laser_opp_pub = nh.advertise < sensor_msgs::LaserScan > ("/ROBOT/hokuyo_opponent", 5);
    //fire_pub = nh.advertise < pcl::PointCloud<pcl::PointXYZ> > ("/ROBOT/Fire_PointCloud", 5);

    joint_pub = nh.advertise < std_msgs::Float64 > ("/hokuyo_link_controller/command", 5);


    std_msgs::Float64 tmp;
    tmp.data = TOP_SERVO;
    joint_pub.publish(tmp);

    //tf::TransformListener tmp_tflistener (const ros::NodeHandle &nh, ros::Duration max_cache_time=ros::Duration(DEFAULT_CACHE_TIME), bool spin_thread=true);
    //tflistener = 

    //tflistener.setExtrapolationLimit(ros::Duration(10.0));

    passX.setFilterFieldName("x");
    passX.setFilterLimits(-1.45, 1.45);

    passY.setFilterFieldName("y");
    passY.setFilterLimits(0.05, 1.95);

    passX1.setFilterFieldName("x");
    passX1.setFilterLimits(-1.47, -1.20);
    passY1.setFilterFieldName("y");
    passY1.setFilterLimits(0.03, 0.20);

    passX2.setFilterFieldName("x");
    passX2.setFilterLimits(1.20, 1.47);
    passY2.setFilterFieldName("y");
    passY2.setFilterLimits(0.03, 0.20);

    passX3.setFilterFieldName("x");
    passX3.setFilterLimits(-0.17, 0.17);
    passY3.setFilterFieldName("y");
    passY3.setFilterLimits(0.780, 1.120);

    //passY.setFilterLimitsNegative (true);
    //passX.setFilterLimitsNegative (true);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tmp_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree = tmp_tree;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(100);


    cloud_cluster.header.frame_id = "/world";


    state = NORMAL;
    //state = BOTTOM;



}

void FireFinder::laserOppCallback(const sensor_msgs::LaserScan::ConstPtr & laser)
{
	sensor_msgs::PointCloud2 tmp_cloud;
	sensor_msgs::LaserScan tmp_laser;
	std::vector<pcl::PointIndices>::const_iterator it;
	std::vector<int>::const_iterator pit;
	pcl::PointXYZ tmp_point;
	double x, y, max_dist, tmp_dist, first_x, first_y;
	int cpt_seg, cpt_cloud;

	ros::Time begin = ros::Time::now();

	switch(state) {
		case TRANSITION :
			ROS_ERROR("TRANSITION");
			save_opp_laser.header.stamp = ros::Time::now();
			break;
		case NORMAL :
			ROS_ERROR("NORMAL");
			save_opp_laser = *laser;
			break;
		case BOTTOM :
			ROS_ERROR("BOT");
			save_opp_laser.header.stamp = ros::Time::now();


			//begin = ros::Time::now();
			//laser->header.stamp = (ros::Time const) &begin;
			tmp_laser = *laser;
			tmp_laser.header.stamp = ros::Time::now();
			//while( !tflistener.canTransform("/hokuyo_laser", "/world", ros::Time::now()) ){
			//while( !(tf_.canTransform("/hokuyo_laser", "/world", laser->header.stamp)) ){
			while( !(tf_.canTransform("/hokuyo_laser", "/world", tmp_laser.header.stamp)) ){
				usleep(10000);
				ros::spinOnce();
			}
			ROS_ERROR("BOT2");
			try {
			projector.transformLaserScanToPointCloud("/world", tmp_laser, tmp_cloud, tf_);
			ROS_ERROR("BOT3");

			pcl::fromROSMsg(tmp_cloud, cloud_low);

			passX.setInputCloud(cloud_low.makeShared());
			passX.filter(cloud_low);

			passY.setInputCloud(cloud_low.makeShared());
			passY.filter(cloud_low);

			pcl::toROSMsg(cloud_low, tmp_cloud);
                        debug_pub.publish(tmp_cloud);

			cluster_indices.clear();
			// Segmentation
			tree->setInputCloud(cloud_low.makeShared());

			ec.setSearchMethod(tree);
			ec.setInputCloud(cloud_low.makeShared());
			ec.extract(cluster_indices);

			// Take the mean value of each cluster
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);


			cloud_cluster.clear();

			cpt_cloud = 0;
			for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
			{
				//ROS_ERROR("3");			
				cpt_seg = 0;
				x = 0.0; y = 0.0;
				max_dist = 0.0;
				tmp_dist = 0.0;
				for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
				{
					//ROS_ERROR("4");

					if(cpt_seg == 0) {
						first_x = cloud_low.points[*pit].x;
						first_y = cloud_low.points[*pit].y;
					}
					else {
						tmp_dist = sqrt( (cloud_low.points[*pit].x - first_x)*(cloud_low.points[*pit].x - first_x) + (cloud_low.points[*pit].y - first_y)*(cloud_low.points[*pit].y - first_y));
						if(max_dist < tmp_dist)
							max_dist = tmp_dist;
					}

					x += cloud_low.points[*pit].x;
					y += cloud_low.points[*pit].y;
					cpt_seg++;
				}
				//tmp_point = cloud.points[*pit];
				tmp_point.x = (x/cpt_seg);
				tmp_point.y = (y/cpt_seg);
				tmp_point.z = 0.015;

				//ROS_ERROR("dist : %f", max_dist);
				if(max_dist < 0.13) // It's not an opponent
					cloud_cluster.points.push_back(tmp_point);

				cpt_cloud++;

			}

		
			cloud_cluster.width = cloud_cluster.points.size();
			cloud_cluster.height = 1;
                        cloud_cluster.is_dense = true;


			pcl::toROSMsg(cloud_cluster, tmp_cloud);

			low_pub.publish(tmp_cloud);


			state = TRANSITION;
			}
			catch (tf::TransformException e)
			{
				ROS_WARN("My_node: %s", e.what());
			}


			break;
		case MID :
			ROS_ERROR("MID");
			save_opp_laser.header.stamp = ros::Time::now();

			tmp_laser = *laser;
			tmp_laser.header.stamp = ros::Time::now();
			//laser->header.stamp = ros::Time::now();
			//while( !tflistener.canTransform("/hokuyo_laser", "/world", ros::Time::now()) ){
			//while( !tf_.canTransform("/hokuyo_laser", "/world", laser->header.stamp) ){
			while( !(tf_.canTransform("/hokuyo_laser", "/world", tmp_laser.header.stamp)) ){
				usleep(10000);
				ros::spinOnce();
			}
			ROS_ERROR("MID2");
			try{
			projector.transformLaserScanToPointCloud("/world", tmp_laser, tmp_cloud, tf_);
			ROS_ERROR("MID3");

                        pcl::fromROSMsg(tmp_cloud, cloud_mid);
			pcl::fromROSMsg(tmp_cloud, tmp_pcloud);

                        passX1.setInputCloud(cloud_mid.makeShared());
                        passX1.filter(cloud_mid);

                        passY1.setInputCloud(cloud_mid.makeShared());
                        passY1.filter(cloud_mid);

			passX2.setInputCloud(tmp_pcloud.makeShared());
                        passX2.filter(tmp_pcloud);
			
			passY2.setInputCloud(tmp_pcloud.makeShared());
                        passY2.filter(tmp_pcloud);

			cloud_mid += tmp_pcloud;

			pcl::fromROSMsg(tmp_cloud, tmp_pcloud);

			passX3.setInputCloud(tmp_pcloud.makeShared());
                        passX3.filter(tmp_pcloud);

                        passY3.setInputCloud(tmp_pcloud.makeShared());
                        passY3.filter(tmp_pcloud);

			cloud_mid += tmp_pcloud;


			pcl::toROSMsg(cloud_mid, tmp_cloud);
                        debug_pub.publish(tmp_cloud);



                        cluster_indices.clear();
                        // Segmentation
                        tree->setInputCloud(cloud_mid.makeShared());

                        ec.setSearchMethod(tree);
                        ec.setInputCloud(cloud_mid.makeShared());
                        ec.extract(cluster_indices);

                        // Take the mean value of each cluster
                        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);


                        cloud_cluster.clear();

                        cpt_cloud = 0;
                        for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
                        {
                                //ROS_ERROR("3");                       
                                cpt_seg = 0;
                                x = 0.0; y = 0.0;
                                max_dist = 0.0;
                                tmp_dist = 0.0;
                                for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
                                {
                                        //ROS_ERROR("4");

                                        x += cloud_mid.points[*pit].x;
                                        y += cloud_mid.points[*pit].y;
                                        cpt_seg++;
                                }
                                tmp_point.x = (x/cpt_seg);
                                tmp_point.y = (y/cpt_seg);
                                tmp_point.z = 0.045;

				cloud_cluster.points.push_back(tmp_point);

                                cpt_cloud++;

                        }


                        cloud_cluster.width = cloud_cluster.points.size();
                        cloud_cluster.height = 1;
                        cloud_cluster.is_dense = true;




			pcl::toROSMsg(cloud_cluster, tmp_cloud);

                        mid_pub.publish(tmp_cloud);

			state = TRANSITION;
			}
			catch (tf::TransformException e)
			{
				ROS_WARN("My_node: %s", e.what());
			}


			break;
		default :
			ROS_ERROR("Default");
			break;
	}
	laser_opp_pub.publish(save_opp_laser);

}

void FireFinder::findFireCallback(const std_msgs::Empty::ConstPtr & empty)
{
	std_msgs::Float64 tmp;

	state = TRANSITION;
	
        tmp.data = BOT_SERVO;
        joint_pub.publish(tmp);
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(10000);

	state = BOTTOM;
	usleep(10000);
	while( state != TRANSITION ) {
		ros::spinOnce();
		usleep(10000);
	}


        tmp.data = MID_SERVO;
        joint_pub.publish(tmp);
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(10000);

	state = MID;
	usleep(10000);
        while( state != TRANSITION ) {
                ros::spinOnce();
                usleep(10000);
        }


        tmp.data = TOP_SERVO;
        joint_pub.publish(tmp);

	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();
	usleep(100000);
	ros::spinOnce();

	state = NORMAL;
}

/*
void FireFinder::laserFireCallback(const sensor_msgs::LaserScan::ConstPtr & laser)
{
 


    //ROS_ERROR("!");
    sensor_msgs::PointCloud2 tmp_cloud;
    projector.transformLaserScanToPointCloud("map", *laser, tmp_cloud, tflistener);

    pcl::fromROSMsg(tmp_cloud, opp_cloud);

    passX.setInputCloud(opp_cloud.makeShared());
    passX.filter(opp_cloud);

    passY.setInputCloud(opp_cloud.makeShared());
    passY.filter(opp_cloud);





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
*/


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

    tf::TransformListener listener(ros::Duration(10));
    FireFinder firefinder(listener);

    // Refresh rate
    ros::Rate loop_rate(30);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}
