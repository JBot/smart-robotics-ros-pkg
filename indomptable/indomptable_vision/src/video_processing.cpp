#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);

    //cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

      Mat gray;
      cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
      // smooth it, otherwise a lot of false circles may be detected
      GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
      vector<Vec3f> circles;
      HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 200, 200, 100, 50 );
      cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
      for( size_t i = 0; i < circles.size(); i++ )
      {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
      }
    Canny( gray, gray, 16, 33, 3 );
    vector<Vec4i> lines;
    HoughLinesP( gray, lines, 1, CV_PI/180, 10.0, 240.0, 15.0 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        if( ((lines.size() - i) > 1) && (sqrt((lines[i][0]-lines[i+1][0])*(lines[i][0]-lines[i+1][0]) + (lines[i][1]-lines[i+1][1])*(lines[i][1]-lines[i+1][1])) > 125 ) && (sqrt((lines[i][0]-lines[i+1][0])*(lines[i][0]-lines[i+1][0]) + (lines[i][1]-lines[i+1][1])*(lines[i][1]-lines[i+1][1])) < 170 )) {
          line( cv_ptr->image, Point(lines[i][0], lines[i][1]),
            Point(lines[i+1][2], lines[i+1][3]), Scalar(255,255,0), 3, 8 );
          line( cv_ptr->image, Point(lines[i+1][0], lines[i+1][1]),
            Point(lines[i][2], lines[i][3]), Scalar(255,255,0), 3, 8 );

	  /* Test of line intersection */
	  int a, b, c, d, x, y;
	  /* Segment equation */
	  a = (lines[i][0] - lines[i+1][2]) / (lines[i][1] - lines[i+1][3]);
	  b = lines[i][1] - a * lines[i][0];

	  c = (lines[i+1][0] - lines[i][2]) / (lines[i+1][1] - lines[i][3]);
          d = lines[i+1][1] - c * lines[i+1][0];
	  /* Intersection point */
	  x = (d - b) / (a - c);
	  y = a * x + b;

	  ROS_WARNING("intersection : %i | %i ", x, y);  
	  
	  // draw the circle center
	  Point center2(cvRound(x), cvRound(y));
          circle( cv_ptr->image, center2, 3, Scalar(0,255,0), -1, 8, 0 );

          i++;
          break;
        }
        else {
          line( cv_ptr->image, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(255,0,0), 3, 8 );
        }
    }


/*    Mat gray = Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
    Mat out;
    cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

    // smooth it, otherwise a lot of false circles may be detected
    GaussianBlur( gray, gray, Size(9, 9), 7, 9 );

    Canny( gray, out, 16, 33, 3 );
    Mat gray2;
    cvtColor(cv_ptr->image, gray2, CV_BGR2GRAY);
    //Canny( gray2, out, 16, 33, 3 );
*/
/*
    vector<Vec3f> circles;
    HoughCircles(out, circles, CV_HOUGH_GRADIENT, 3, 506, 90, 56, 50 );

      for( size_t i = 0; i < circles.size(); i++ )
      {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
      }
*/
/*
    vector<Vec2f> lines;
    HoughLines( out, lines, 1, CV_PI/180, 100 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        line( cv_ptr->image, pt1, pt2, Scalar(0,0,255), 3, 8 );
    }
*/
/*
    vector<Vec4i> lines;
    HoughLinesP( out, lines, 1, CV_PI/180, 10.0, 140.0, 15.0 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( cv_ptr->image, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(255,0,0), 3, 8 );
    }
*/


/*    cv::namedWindow( "Source", 1 );
    cv::imshow( "Source", cv_ptr->image );
    
    cv::Mat dst = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);

    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;

    cv::findContours( cv_ptr->image, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        cv::Scalar color( rand()&255, rand()&255, rand()&255 );
        cv::drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
    }

//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::imshow(WINDOW, cv_ptr->image);
//    cv::imshow(WINDOW, dst);
*/

    
//    image_pub_.publish(cv_ptr->toImageMsg());
    waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
