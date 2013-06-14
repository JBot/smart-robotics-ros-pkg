#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <std_msgs/Int32.h>

class LaserScanToPointCloud{

	public:

		ros::NodeHandle n_;
		laser_geometry::LaserProjection projector_;
		tf::TransformListener listener_;
		message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
		tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;


		ros::Subscriber mode_sub;
		ros::Publisher scan_pub_;
		ros::Publisher laser_pub_;


		int current_mode;
		int previous_mode;
		sensor_msgs::LaserScan saved_laser;


		LaserScanToPointCloud(ros::NodeHandle n) : 
			n_(n),
			laser_sub_(n_, "base_scan", 10),
			laser_notifier_(laser_sub_,listener_, "base_link", 10)
	{

		laser_notifier_.registerCallback(
				boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
		laser_notifier_.setTolerance(ros::Duration(0.01));


		current_mode = 0;
		previous_mode = 0;

		mode_sub = n_.subscribe<std_msgs::Int32>("/TEMERAIRE/mode", 5, &LaserScanToPointCloud::modeCallback, this);
		scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/TEMERAIRE/point_cloud",1);
		laser_pub_ = n_.advertise<sensor_msgs::LaserScan>("/TEMERAIRE/laser_scan",1);
	}


		void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
		{

			if(current_mode == 0) {
				// Publish laser_scan

				laser_pub_.publish(*scan_in);
				previous_mode = 0;
			}
			else {

				if(previous_mode == 0) {
					saved_laser = *scan_in;
					previous_mode = 1;
				}

				sensor_msgs::PointCloud2 cloud;
				try
				{
					projector_.transformLaserScanToPointCloud(
							"base_link",*scan_in, cloud,listener_);

					cloud.header.frame_id = "scanner_link";
				}
				catch (tf::TransformException& e)
				{
					std::cout << e.what();
					return;
				}



				// Do something with cloud.
				laser_pub_.publish(saved_laser);
				scan_pub_.publish(cloud);
			}


		}

		void modeCallback(const std_msgs::Int32::ConstPtr& gait)
		{
			current_mode = gait->data;
		}




};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "TEMERAIRE_scan_to_cloud");
	ros::NodeHandle n;
	LaserScanToPointCloud lstopc(n);

	ros::spin();

	return 0;
}
