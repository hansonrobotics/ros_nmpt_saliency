#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>
#include "FastSalience.h"
#include "LQRPointTracker.h"
#include "NMPTUtils.h"

#include "sensor_msgs/RegionOfInterest.h"

using namespace std;
using namespace cv; 

	FastSalience salTracker;
	LQRPointTracker salientSpot(2);
	vector<double> lqrpt(2,.5); 
	Mat im, im2, viz, sal ;
	ros::Publisher pub;
	
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
	    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	    im=cv_ptr->image;
		vector<KeyPoint> pts;
		salTracker.detect(im, pts);
		salTracker.getSalMap(sal);
        // Get Actual Min and Max for saliency
		double min, max; 
		Point minloc, maxloc; 
		minMaxLoc(sal, &min, &max, &minloc, &maxloc); 
        // Use the point tracker to get the moving point.
//		lqrpt[0] = maxloc.x*1.0 / sal.cols;
//		lqrpt[1] = maxloc.y*1.0 / sal.rows;
//		salientSpot.setTrackerTarget(lqrpt);
//		salientSpot.updateTrackerPosition();
//		lqrpt = salientSpot.getCurrentPosition();
        // Publish message
		sensor_msgs::RegionOfInterest roi;
		roi.x_offset = maxloc.x;
		roi.y_offset = maxloc.y;
		roi.width = 1;
		roi.height = 1;
		roi.do_rectify = false;
		pub.publish(roi);
 
   }
   catch (...)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}

int main(int argc, char **argv)
   {
     ros::init(argc, argv, "image_listener");
     ros::NodeHandle nh;
     //cvNamedWindow("view");
     //cvStartWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
     pub = nh.advertise<sensor_msgs::RegionOfInterest>("/nmpt_roi", 50);
     
     salientSpot.setTrackerTarget(lqrpt);
     ros::spin();
     //cvDestroyWindow("view");
   }
