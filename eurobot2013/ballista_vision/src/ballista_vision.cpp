#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

//#include "indomptable_vision/ImageResult.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

/** Function Headers */

/** Global variables */
string window_name = "Capture - Color detection";


class Vision {
    public:
        Vision();
        void detectAndDisplay( Mat frame );


    private:
        CvPoint binarisation(Mat image, int *nbPixels, int h, int s, int v, int tolerance, Scalar color);

        ros::NodeHandle nh;
        
        IplImage *mask;
        Mat hsv_mat;
        Mat mask_mat;
        Mat kernel_mat;


};

Vision::Vision()
{




}






/*
 * Transform the image into a two colored image, one color for the color we want to track, another color for the others colors
 * From this image, we get two datas : the number of pixel detected, and the center of gravity of these pixel
 */
CvPoint Vision::binarisation(Mat image, int *nbPixels, int h, int s, int v, int tolerance, Scalar color) {

    int x, y;
    CvScalar pixel;
    int sommeX = 0, sommeY = 0;
    *nbPixels = 0;

    // Create the hsv image
    //hsv = cvCloneImage(image);

    cvtColor(image, hsv_mat, CV_BGR2HSV);

    // We create the mask
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

    //cvReleaseImage(&hsv);
    imshow( "Mask", mask_mat );

    // If there is no pixel, we return a center outside the image, else we return the center of gravity
    if(*nbPixels > 15000) {
        circle( image, cvPoint((int)(sommeX / (*nbPixels)), (int)(sommeY / (*nbPixels))), 3, color, -1, 8, 0 );
        imshow( "Mask", mask_mat );
        return cvPoint((int)(sommeX / (*nbPixels)), (int)(sommeY / (*nbPixels)));
    }
    else
        return cvPoint(-1, -1);
}



/** @function detectAndDisplay */
void Vision::detectAndDisplay( Mat frame )
{
    int i = 0;
    Mat hsv_frame;

    int hauteur [10] = {312, 306, 300, 300, 350, 350, 350, 350, 350, 350};
    int largeur [10] = {(112*640/165), (121*640/165), (136*640/165), (154*640/165), (100*640/165), (107*640/165), (116*640/165), (129*640/165), (146*640/165), (163*640/165)};

    // accept only char type matrices
    CV_Assert(frame.depth() != sizeof(uchar));
    cvtColor(frame, hsv_frame, CV_BGR2HSV);

    MatIterator_<Vec3b> it, end;
/*
    for( it = hsv_frame.begin<Vec3b>(), end = hsv_frame.end<Vec3b>(); it != end; ++it)
    {
        if(i == 153920) {
            cout << ((uint)((*it)[0])) << " " << ((uint)((*it)[1])) << " " << ((uint)((*it)[2])) << endl;
            if( ((uint)((*it)[0])) > 130 )
                cout << "ROUGE" << endl;
            else 
                cout << "BLEU" << endl;
        }
        i++;
    }
*/

    for( i=0; i < 10; i++) {
        it = hsv_frame.begin<Vec3b>();
        it = it + ( (hauteur[i]*640) + largeur[i]);

        if( ( ((uint)((*it)[0])) + ((uint)((*(it++))[0])) ) > 250 )
            cout << "RED" << endl;
        else 
            cout << "BLUE" << endl;

        circle( frame, cvPoint(largeur[i], hauteur[i]), 3, Scalar( 0, 0, 255 ), -1, 8, 0 );
    }
    cout << endl;
    //-- Show what you got
    imshow( window_name, frame );
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
    ros::init(argc, argv, "Ballista_Vision");
    Vision vision;

    CvCapture* capture;
    Mat frame;

    int cpt_frame = 0;

    //-- 2. Read the video stream
    capture = cvCaptureFromCAM( -1 );
    int bla = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
    cout << bla << endl;
    if( capture )
    {
        while( true )
        {
            frame = cvQueryFrame( capture );

            //-- 3. Apply the classifier to the frame
            if( !frame.empty() )
            { 
                //if(cpt_frame == 0) {
                    vision.detectAndDisplay( frame ); 
                //  cpt_frame++;
                //}
                //else {
                //    cpt_frame = 0;
                //}

                //ros::Duration(0.1).sleep();
            }
            else
            { 
                printf(" --(!) No captured frame -- Break!"); 
                break; 
            }

            int c = waitKey(1);
            if( (char)c == 'c' ) { break; }
        }
    }

    ros::Duration(2.0).sleep();
    ros::shutdown();


}
