#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/make_shared.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  //image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    //image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);

    //namedWindow(WINDOW);
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

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));

    //cv::imshow(WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void get_compute_image(void) {
    //cv_bridge::CvImagePtr cv_ptr;
    //cv_ptr = boost::make_shared<cv_bridge::CvImage>();

    VideoCapture cap;
    cap.open(0);
    Mat frame, gray;
    Mat out;
    for(;;) {
      cap >> frame;
      if(!frame.data) break;
      //out = Mat::zeros(frame.rows, frame.cols, CV_8UC3);
      cvtColor(frame, gray, CV_BGR2GRAY);
      // smooth it, otherwise a lot of false circles may be detected
      //GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
      //vector<Vec3f> circles;
      //HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 20, 200, 100 );
      /*for( size_t i = 0; i < circles.size(); i++ )
      {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
      }*/
      namedWindow( "circles", 1 );
      imshow( "circles", gray );

      //Canny(gray, out, 5, 7);
      //Canny( gray, out, 50, 200, 3 );
/*      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      findContours( gray, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

      // iterate through all the top-level contours,
      // draw each connected component with its own random color
      int idx = 0;
      for( ; idx >= 0; idx = hierarchy[idx][0] )
      {
        Scalar color( rand()&255, 255, 255 );
        drawContours( frame, contours, idx, color, CV_FILLED, 8, hierarchy );
      }
*/

      Mat src, dst, color_dst;
      src=imread("Screenshot.png", 0);

      Canny( src, dst, 50, 200, 3 );

      namedWindow("video", 1);
      imshow("video", frame);
//      frame.copyTo(cv_ptr->image);
//      image_pub_.publish(cv_ptr->toImageMsg());
      if(waitKey(3) >= 0) break;
    }
    destroyWindow("video");
    destroyWindow("circles");
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  ImageConverter ic;
//  while(1) {
    ic.get_compute_image();
    ros::spin();
//  }
  return 0;
}
