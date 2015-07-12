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

#define STATE_1 	0 
#define STATE_2 	1 
#define STATE_3 	2 
#define STATE_4 	3 


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

		double x, y, yaw;
		double side;

		int state;
		int cpt;

};

My_Filter::My_Filter(){
	point_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2> ("/BEACON/filtered_pcl", 10, &My_Filter::pclCallback, this);
	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/BEACON/pcl_debug", 10, false);

	side = -1.0;

	x = 0.01;
	y = -0.01;
	yaw = 0.0;

	state = STATE_1;
	cpt = 0;

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


	////////////////////////
	// PassThrough filter //
	////////////////////////
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (pcl_cloud);
	pass.setFilterFieldName ("x");
	if(side > 0)
	{
		pass.setFilterLimits (side*1.4, side*1.5);
	}
	else
	{
		pass.setFilterLimits (side*1.5, side*1.4);
	}
	//pass.setFilterLimitsNegative (true);
	pass.filter (*final);

	pass.setInputCloud (final);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (0.0, 0.1);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*final);


	////////////////////////
	// PassThrough filter //
	////////////////////////
	pass.setInputCloud (pcl_cloud);
	pass.setFilterFieldName ("x");
	if(side > 0)
	{
		pass.setFilterLimits (side*1.4, side*1.5);
	}
	else
	{
		pass.setFilterLimits (side*1.5, side*1.4);
	}
	//pass.setFilterLimitsNegative (true);
	pass.filter (*final2);

	pass.setInputCloud (final2);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (1.9, 2.0);
	//pass.setFilterLimitsNegative (true); 
	pass.filter (*final2);


	double mean = 0.0;
	double mean2 = 0.0;
	int i = 0;


	switch(state) {
		case STATE_1 : 
			// Check if X is equal between both clusters : change YAW

			if(final->size() > 0)
			{
				for(i = 0; i < final->size(); i++)
				{
					mean += final->at(i).x;
				}
				mean = mean / final->size();


				if(final2->size() > 0)
				{ 
					for(i = 0; i < final2->size(); i++)
					{
						mean2 += final2->at(i).x;
					}
					mean2 = mean2 / final2->size();

					yaw += (mean2 - mean)/10;
					cpt++;

				}
				else {

				}

			}   
			else
			{
				// DO nothing
			}


			if(cpt > 40)
			{
				cpt = 0;
				state = STATE_2;
			}



			break;
		case STATE_2 : 
			// Check if Y are at a correct position : change Y

			if(final->size() > 0)
			{ 
				for(i = 0; i < final->size(); i++)
				{
					mean += final->at(i).y;
				}
				mean = mean / final->size();


				if(final2->size() > 0)
				{
					for(i = 0; i < final2->size(); i++)
					{
						mean2 += final2->at(i).y;
					}
					mean2 = mean2 / final2->size();

					y += -(0.05 - mean)/4;
					cpt++; 

				}
				else {

				}

			}
			else
			{
				// DO nothing
			}


			if(cpt > 40)
			{
				cpt = 0;
				state = STATE_3;
			}


			break;
		case STATE_3 : 
			// Check if X are at a correct position : change X

			//-1.4 // 0.0
			if(final->size() > 0)
			{
				for(i = 0;i < final->size(); i++)
				{
					mean += final->at(i).x;
				}
				mean = mean / final->size();

				x += -(side*1.45 - mean)/4;
				cpt++;
			}
			else
			{
				// DO nothing
			}

			if(cpt > 40)
			{
				cpt = 0;
				state = STATE_4;
			}


			break;
		case STATE_4 : 
			// Done

			break;
		default :

			break;
	}


	// Transformation into ROS type
	pcl::toPCLPointCloud2(*(final), *(cloud_out));
	pcl_conversions::moveFromPCL(*(cloud_out), pc2);


	point_cloud_publisher_.publish(pc2);
}

void My_Filter::broadcast(void)
{
	t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, 0.0)),
			ros::Time::now(), "/beacon_link", "/beacon_laser_link");

	t.stamp_ = ros::Time::now();
	broadcaster.sendTransform(t);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_filter");

	My_Filter filter;

	// Refresh rate
	ros::Rate loop_rate(100);                                // 35 with bluetooth


	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		filter.broadcast();
	}

	ros::Duration(2.0).sleep();

	ros::spin();

	return 0;
}

