#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

#include "indomptable_vision/ImageResult.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>

// Maths methods
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))  
#define abs(x) ((x) > 0 ? (x) : -(x))
#define sign(x) ((x) > 0 ? 1 : -1)

// Step mooving for object min & max
#define STEP_MIN 5
#define STEP_MAX 100 

#define NO_TAKE 0 
#define TAKE_AUTONOMOUSLY 1 

#define MAX_CNT_OBJECT 20 

namespace enc = sensor_msgs::image_encodings;
using namespace cv;

static const char WINDOW[] = "Image window";

// Color tracked and our tolerance towards it
int h_CD, s_CD, v_CD, tolerance_CD;
int h_BAR, s_BAR, v_BAR, tolerance_BAR;
int click_count;
// Position of the object we overlay
CvPoint objectPos;

cv_bridge::CvImagePtr cv_ptr;

IplImage *hsv;
//IplConvKernel *kernel;
IplImage *mask;
Mat hsv_mat;
Mat mask_mat;
Mat kernel_mat;




/*
 * Get the color of the pixel where the mouse has clicked
 * We put this color as model color (the color we want to tracked)
 */
void getObjectColor(int event, int x, int y, int flags, void *param = NULL) {

    // Vars
    CvScalar pixel;
    int h = 0, s = 0, v = 0;
    int i, j;

    if(event == CV_EVENT_LBUTTONUP) {

        // Get the hsv image
        //hsv = cvCloneImage(&cv_ptr->image.operator IplImage());
        //CvtColor(cv_ptr->image, hsv, CV_BGR2HSV);

        cvtColor(cv_ptr->image, hsv_mat, CV_BGR2HSV);
        hsv = &hsv_mat.operator IplImage();

        for(i=x-1;i<x+2;i++) {
            for(j=y-1;j<y+2;j++) {
                // Get the selected pixel
                pixel = cvGet2D(hsv, y, x);
                h += (int)pixel.val[0];
                s += (int)pixel.val[1];
                v += (int)pixel.val[2];
            }
        }

        if(click_count == 0) {
            // Change the value of the tracked color with the color of the selected pixel
            h_CD = (int)(h/9);
            s_CD = (int)(s/9);
            v_CD = (int)(v/9);
            click_count++;
        }
        else {
            // Change the value of the tracked color with the color of the selected pixel
            h_BAR = (int)(h/9);
            s_BAR = (int)(s/9);
            v_BAR = (int)(v/9);
            click_count = 0;
        }

        // Release the memory of the hsv image
        //cvReleaseImage(&hsv);

    }

}


class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    ros::ServiceServer image_result_service;

    ros::Publisher object_pub;

    public:

    std_msgs::Float64 x_CD;
    std_msgs::Float64 y_CD;
    std_msgs::Float64 x_BAR;
    std_msgs::Float64 y_BAR;
    std_msgs::Int32 type_obj;

    int mode;
    int nb_img_cnt; // Number of time an object is seen (To avoid taking an object with only 1 image)

    ImageConverter()
        : it_(nh_)
    {
        image_pub_ = it_.advertise("/out/image_raw", 1);
        //image_sub_ = it_.subscribe("/image_raw/uncompressed_jo", 1, &ImageConverter::imageCb, this);
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

        image_result_service = nh_.advertiseService("/indomptable/image_result", &ImageConverter::imageResultService, this);

        object_pub = nh_.advertise < geometry_msgs::PoseStamped > ("/object_pose", 5);

        objectPos = cvPoint(-1, -1);
        h_CD = 0; s_CD = 0; v_CD = 0; tolerance_CD = 25;
        h_BAR = 0; s_BAR = 0; v_BAR = 0; tolerance_BAR = 20;

        x_CD.data = -1;
        y_CD.data = -1;
        x_BAR.data = -1;
        y_BAR.data = -1;
        type_obj.data = 0;

        mode = NO_TAKE;
        nb_img_cnt = 0;

        // Create the windows
/*        cvNamedWindow("Camera output", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
        cvMoveWindow("Camera output", 0, 100);
        cvMoveWindow("Mask", 650, 100);


        // Mouse event to select the tracked color on the original image
        cvSetMouseCallback("Camera output", getObjectColor);
*/
        //cv::namedWindow(WINDOW);
    }

    ~ImageConverter()
    {
        // Destroy the windows we have created
        cvDestroyWindow("Camera output");
        cvDestroyWindow("Mask");
        //cv::destroyWindow(WINDOW);
    }

    bool imageResultService(indomptable_vision::ImageResult::Request  &req, indomptable_vision::ImageResult::Response &res )
    {
        switch(req.type.data) {

            case 0 :
                mode = NO_TAKE;

                res.type.data = type_obj.data;
                res.x.data = x_CD.data;
                res.y.data = y_CD.data;

                break;
            case 1 :
                res.type.data = type_obj.data & 0x1;
                res.x.data = ((288)*90.0/144.0 -((double)y_CD.data)*90.0/144.0 + (50))/1000.0 - 0.02;
                res.y.data = (-((double)x_CD.data)*250.0/352.0+(167.0*278.0/352.0))/1000.0 + 0.015;
                //res.x.data = ((480)*90.0/240.0 -((double)y_CD.data)*90.0/240.0 + (50))/1000.0 - 0.02;
                //res.y.data = (-((double)x_CD.data)*278.0/640.0+(335.0*278.0/640.0))/1000.0;
                ROS_ERROR("Requesting CD position: x:%f , y:%f", x_CD.data, y_CD.data);
                break;
            case 2 :
                res.type.data = type_obj.data & 0x2;
                res.x.data = ((480)*90.0/240.0 -((double)y_BAR.data)*90.0/240.0 + (50))/1000.0;
                res.y.data = (-((double)x_BAR.data)*278.0/640.0+(335.0*278.0/640.0))/1000.0;

                ROS_ERROR("Requesting BAR position: x:%f , y:%f", x_BAR.data, y_BAR.data);
                break;
            case 3 :

                mode = TAKE_AUTONOMOUSLY;

                res.type.data = type_obj.data;
                res.x.data = x_CD.data;
                res.y.data = y_CD.data;

                ROS_ERROR("Requesting any position: x:%f , y:%f / x:%f , y:%f", -((double)x_CD.data)*278.0/640.0+(335.0*278.0/640.0), (480)*90.0/240.0 -((double)y_CD.data)*90.0/240.0 + (50), -((double)x_BAR.data)*278.0/640.0+(335.0*278.0/640.0), (480)*90.0/240.0 -((double)y_BAR.data)*90.0/240.0 + (50) );
                break;
            default :

                break;
        }
        return true;
    }

    /*
     * Transform the image into a two colored image, one color for the color we want to track, another color for the others colors
     * From this image, we get two datas : the number of pixel detected, and the center of gravity of these pixel
     */
    CvPoint binarisation(IplImage* image, int *nbPixels, int h, int s, int v, int tolerance, Scalar color) {

        int x, y;
        CvScalar pixel;
        int sommeX = 0, sommeY = 0;
        *nbPixels = 0;

        // Create the mask &initialize it to white (no color detected)
        //mask = cvCreateImage(cvGetSize(image), image->depth, 1);

        // Create the hsv image
        hsv = cvCloneImage(image);

        //cvCvtColor(image, hsv, CV_BGR2HSV);
        cvtColor(cv_ptr->image, hsv_mat, CV_BGR2HSV);

        // We create the mask
        //cvInRangeS(hsv, cvScalar(h - tolerance -1, s - tolerance, 0), cvScalar(h + tolerance -1, s + tolerance, 255), mask);
        //Mat range_low(1, 1, CV_UC3, Scalar(h - tolerance -1, s - tolerance, 0));
        //Mat range_high(1, 1, CV_UC3, Scalar(h + tolerance -1, s + tolerance, 0));

        inRange(hsv_mat, Scalar(h - tolerance -1, s - tolerance, 0), Scalar(h + tolerance -1, s + tolerance, 255), mask_mat);

        // Create kernels for the morphological operation
        //kernel = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);
        kernel_mat = getStructuringElement( MORPH_ELLIPSE, Size(3, 3), Point(2, 2) );
        Mat kernel_mat2 = getStructuringElement( MORPH_ELLIPSE, Size(7, 7), Point(2, 2) );

        // Morphological opening (inverse because we have white pixels on black background)
        erode(mask_mat, mask_mat, kernel_mat2);
        dilate(mask_mat, mask_mat, kernel_mat);
        //erode(mask_mat, mask_mat, kernel_mat2);

        mask = &mask_mat.operator IplImage();

        // We go through the mask to look for the tracked object and get its gravity center
        for(x = 0; x < mask->width; x++) {
            for(y = 0; y < mask->height; y++) {

                // If its a tracked pixel, count it to the center of gravity's calcul
                if(((uchar *)(mask->imageData + y*mask->widthStep))[x] == 255) {
                    sommeX += x;
                    sommeY += y;
                    (*nbPixels)++;
                }
            }
        }

        // Show the result of the mask image
        //cvShowImage("GeckoGeek Mask", mask);

        // We release the memory of kernels
        //cvReleaseStructuringElement(&kernel);

        // We release the memory of the mask
        //cvReleaseImage(&mask);
        // We release the memory of the hsv image
        cvReleaseImage(&hsv);

        // If there is no pixel, we return a center outside the image, else we return the center of gravity
        if(*nbPixels > 15000) {
            circle( cv_ptr->image, cvPoint((int)(sommeX / (*nbPixels)), (int)(sommeY / (*nbPixels))), 3, color, -1, 8, 0 );
//	    imshow( "Mask", mask_mat );
            return cvPoint((int)(sommeX / (*nbPixels)), (int)(sommeY / (*nbPixels)));
        }
        else
            return cvPoint(-1, -1);
    }


    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        //cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        geometry_msgs::PoseStamped tmppose;

        CvPoint objectNextPos;
        int nbPixels;
/*
        objectNextPos = binarisation(&cv_ptr->image.operator IplImage(), &nbPixels, h_CD, s_CD, v_CD, tolerance_CD, Scalar(0,255,0));
        x_CD.data = objectNextPos.x;
        y_CD.data = objectNextPos.y;
*/
	Mat gray;
	cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
        // smooth it, otherwise a lot of false circles may be detected
        GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
        vector<Vec3f> circles;
        HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 200, 200, 100, 50 );
        cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
	type_obj.data = 0;
        for( size_t i = 0; i < circles.size(); i++ )
        {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                // draw the circle center
                circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
                // draw the circle outline
                circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );

		x_CD.data = cvRound(circles[i][0]);
		y_CD.data = cvRound(circles[i][1]);

		type_obj.data = 1;

        }

/*
        if(x_CD.data < 1)
            type_obj.data = 0;
        else 
            type_obj.data = 1;
*/
        objectNextPos = binarisation(&cv_ptr->image.operator IplImage(), &nbPixels, h_BAR, s_BAR, v_BAR, tolerance_BAR, Scalar(0,0,255));
        x_BAR.data = objectNextPos.x;
        y_BAR.data = objectNextPos.y;
        if(x_BAR.data < 1)
            type_obj.data = type_obj.data;
        else
            type_obj.data = type_obj.data + 2;


//        imshow( "Camera output", cv_ptr->image );

        if(mode == TAKE_AUTONOMOUSLY) {

            if(type_obj.data > 0) {

                switch(type_obj.data) {
                    case 0 :
                        nb_img_cnt = 0;
                        break;
                    case 1 : // Only CD is seen
                        if(nb_img_cnt > MAX_CNT_OBJECT) { // Object seen X times
                            tmppose.pose.position.x = ((288)*90.0/144.0 -((double)y_CD.data)*90.0/144.0 + (50))/1000.0 - 0.02;
                            tmppose.pose.position.y = (-((double)x_CD.data)*250.0/352.0+(167.0*278.0/352.0))/1000.0 + 0.015;
                            tmppose.pose.position.z = 0.072;
                            object_pub.publish(tmppose);
                            ROS_ERROR("Taking CD : %f : %f", tmppose.pose.position.x, tmppose.pose.position.y);
                            nb_img_cnt = -40;
                        }
                        else {
                            nb_img_cnt++;
                        }
                        break;
                    case 2 : // Only BAR is seen
                        if(nb_img_cnt > MAX_CNT_OBJECT) { // Object seen X times
                            tmppose.pose.position.x = ((480)*90.0/240.0 -((double)y_BAR.data)*90.0/240.0 + (50))/1000.0;
                            tmppose.pose.position.y = (-((double)x_BAR.data)*278.0/640.0+(335.0*278.0/640.0))/1000.0;
                            tmppose.pose.position.z = 0.062;
                            object_pub.publish(tmppose);
                            ROS_ERROR("Taking BAR : %f : %f", tmppose.pose.position.x, tmppose.pose.position.y);
                            nb_img_cnt = 0;
                        }
                        else {
                            nb_img_cnt++;
                        }
                        break;

                    case 3 : // Two different objects are seen
                        if(nb_img_cnt > MAX_CNT_OBJECT) { // Object seen X times

                            if(y_CD.data > y_BAR.data) {
                                ROS_ERROR("Taking CD");
                                nb_img_cnt = 0;
                            }
                            else {
                                ROS_ERROR("Taking BAR");
                                nb_img_cnt = 0;
                            }

                        }
                        else {
                            nb_img_cnt++;
                        }
                        break;
                }

            }
            else {
                nb_img_cnt--;
		if(nb_img_cnt < 0)
			nb_img_cnt = 0;
            }

        }
        else {
            nb_img_cnt = 0;
        }



        
        /*   Mat gray;
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
        }*/
        /*
           Canny( gray, gray, 16, 33, 3 );
           vector<Vec4i> lines;
           HoughLinesP( gray, lines, 1, CV_PI/180, 10.0, 240.0, 15.0 );
         */

        /*    for( size_t i = 0; i < lines.size(); i++ )
              {
              if( ((lines.size() - i) > 1) && (sqrt((lines[i][0]-lines[i+1][0])*(lines[i][0]-lines[i+1][0]) + (lines[i][1]-lines[i+1][1])*(lines[i][1]-lines[i+1][1])) > 125 ) && (sqrt((lines[i][0]-lines[i+1][0])*(lines[i][0]-lines[i+1][0]) + (lines[i][1]-lines[i+1][1])*(lines[i][1]-lines[i+1][1])) < 170 )) {
              line( cv_ptr->image, Point(lines[i][0], lines[i][1]),
              Point(lines[i+1][2], lines[i+1][3]), Scalar(255,255,0), 3, 8 );
              line( cv_ptr->image, Point(lines[i+1][0], lines[i+1][1]),
              Point(lines[i][2], lines[i][3]), Scalar(255,255,0), 3, 8 );
         */
        /* Test of line intersection */
        //	  int a, b, c, d, x, y;
        /* Segment equation */
        /*	  a = (lines[i][0] - lines[i+1][2]) / (lines[i][1] - lines[i+1][3]);
              b = lines[i][1] - a * lines[i][0];

              c = (lines[i+1][0] - lines[i][2]) / (lines[i+1][1] - lines[i][3]);
              d = lines[i+1][1] - c * lines[i+1][0];
         */	  /* Intersection point */
        /*	  x = (d - b) / (a - c);
              y = a * x + b;

              ROS_ERROR("intersection : %i | %i ", x, y);  

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
         */

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


        image_pub_.publish(cv_ptr->toImageMsg());
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
