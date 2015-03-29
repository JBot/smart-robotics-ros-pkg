/* ROS includes */
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

/* C includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

/* C++ includes */
#include <iostream>
#include <string>
#include <list>


static const std::string OPENCV_WINDOW = "Image window";

class mapStitch {
	public:
		mapStitch();

		ros::Subscriber global_map_sub_;
		ros::Subscriber actual_map_sub_;

		ros::Publisher debug_map_pub;

		void loop(void);

	private:
		void globalMapCallback(const sensor_msgs::Image::ConstPtr & feedback);
		void actualMapCallback(const sensor_msgs::Image::ConstPtr & feedback);
		ros::NodeHandle nh;

		bool isGlobalExist;
		cv_bridge::CvImagePtr global_map_ptr;


};


/* Constructor */
mapStitch::mapStitch()
{
	std::string name;

	global_map_sub_ = nh.subscribe < sensor_msgs::Image > ("/HOME/image_map", 1, &mapStitch::globalMapCallback, this);
	actual_map_sub_ = nh.subscribe < sensor_msgs::Image > ("/NESTOR/image_map", 1, &mapStitch::actualMapCallback, this);

	debug_map_pub = nh.advertise < sensor_msgs::Image > ("/NESTOR/debug_map", 1);

	isGlobalExist = false;
}


void mapStitch::loop(void)
{

}


void mapStitch::globalMapCallback(const sensor_msgs::Image::ConstPtr & feedback)
{
	try
	{
		global_map_ptr = cv_bridge::toCvCopy(feedback, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Draw an example circle on the video stream
	//if (global_map_ptr->image.rows > 60 && global_map_ptr->image.cols > 60)
	// 	cv::circle(global_map_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

	ROS_ERROR("rows cols %d %d", global_map_ptr->image.rows, global_map_ptr->image.cols);

	for(int j=0;j<global_map_ptr->image.rows;j++)
	{
		for (int i=0;i<global_map_ptr->image.cols;i++)
		{
			if( global_map_ptr->image.at<uchar>(j,i) > 10 )
				global_map_ptr->image.at<uchar>(j,i) = 0; //
			else
				global_map_ptr->image.at<uchar>(j,i) = 255; //
		}
	}

	isGlobalExist = true;
}

void mapStitch::actualMapCallback(const sensor_msgs::Image::ConstPtr & feedback)
{

	if(isGlobalExist == true)
	{

		cv::Mat templ;

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(feedback, sensor_msgs::image_encodings::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		ROS_ERROR("rows cols %d %d", cv_ptr->image.rows, cv_ptr->image.cols);

		templ.create( cv_ptr->image.rows, cv_ptr->image.cols, CV_8U );


		for(int j=0;j<cv_ptr->image.rows;j++)
		{
			for (int i=0;i<cv_ptr->image.cols;i++)
			{
				if( cv_ptr->image.at<uchar>(j,i) > 10 )
					cv_ptr->image.at<uchar>(j,i) = 0; //
				else
					cv_ptr->image.at<uchar>(j,i) = 255; //

				//if( (i >= (cv_ptr->image.cols/4)) && (i < (cv_ptr->image.cols*3/4)) && (j >= (cv_ptr->image.rows/4)) && (j < (cv_ptr->image.rows*3/4)) )
				//	templ.at<uchar>(j-(cv_ptr->image.rows/4),i-(cv_ptr->image.cols/4)) = cv_ptr->image.at<uchar>(j,i);
				templ.at<uchar>(j,i) = cv_ptr->image.at<uchar>(j,i);
			}
		}


		cv_bridge::CvImage out_msg;
		cv::Mat img_display;
		cv::Mat result;
		
		cv::Point bestMatchLoc;
		double maxValLoop = 0.0;


		for(double angle = -4; angle < 4.5; angle=angle+0.5)
		{
			/// Compute a rotation matrix with respect to the center of the image
			cv::Point center = cv::Point( templ.cols/2, templ.rows/2 );
			cv::Mat rot_mat( 2, 3, CV_32FC1 );
			cv::Mat warp_rotate_dst;
			//double angle = -1.0;
			double scale = 1.0;

			/// Get the rotation matrix with the specifications above
			rot_mat = cv::getRotationMatrix2D( center, angle, scale );

			/// Rotate the warped image
			cv::warpAffine( templ, warp_rotate_dst, rot_mat, templ.size() );


			/// Source image to display
			int match_method = 5;
			cv::cvtColor( global_map_ptr->image, img_display, CV_GRAY2BGR );
			//img.copyTo( global_map_ptr->image );

			/// Create the result matrix
			int result_cols =  global_map_ptr->image.cols - warp_rotate_dst.cols + 1;
			int result_rows = global_map_ptr->image.rows - warp_rotate_dst.rows + 1;

			result.create( result_rows, result_cols, CV_32FC1 );

			/// Do the Matching and Normalize
			cv::matchTemplate( global_map_ptr->image, warp_rotate_dst, result, match_method );


			/// Localizing the best match with minMaxLoc
			double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
			cv::Point matchLoc;
			cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
			printf("Object %lf: [%d %d] \n", maxVal, maxLoc.x, maxLoc.y);

			if(maxValLoop < maxVal)
			{
				bestMatchLoc = maxLoc;
				maxValLoop = maxVal;
			}

			cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

			cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

			/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
			if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
			{ matchLoc = minLoc; }
			else
			{ matchLoc = maxLoc; }

		}

		/// Show me what you got
		cv::rectangle( img_display, bestMatchLoc, cv::Point( bestMatchLoc.x + templ.cols , bestMatchLoc.y + templ.rows ), cv::Scalar(0,0,255), 2, 8, 0 );

		printf("Object: [%d %d] \n", bestMatchLoc.x, bestMatchLoc.y);

		//out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
		out_msg.image    = img_display; // Your cv::Mat
		//cv::drawMatches(global_map_ptr->image , objectKeypoints, cv_ptr->image, sceneKeypoints, matches, cv_ptr->image );

		debug_map_pub.publish(out_msg.toImageMsg());



	}

}


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
	ros::init(argc, argv, "Map_Stitch");
	mapStitch map_stitch;
	// Refresh rate
	ros::Rate loop_rate(20); 
	while (ros::ok()) 
	{
		//command_manager.loop();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

