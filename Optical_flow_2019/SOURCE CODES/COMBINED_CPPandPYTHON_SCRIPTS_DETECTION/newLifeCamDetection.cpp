#include <opencv2/opencv.hpp>
#include <vector>

#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

#include <highgui.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */

#include <iomanip>
#include <string>

#include <iostream>
#include <sstream>

//#include <unistd.h> //for "sleep" function, not used

using namespace std;
using namespace cv;




int main()
{
	int angle = 0;	

	while (true) // DETECTION LOOP
	{
		int something_not_sure_what, x_coordinate;
		
		//creating capturing object (opening camera)
		CvCapture* capture = cvCaptureFromCAM(0);
		if (!capture) {
			fprintf(stderr, "ERROR: capture is NULL \n");
			getchar();
			return -1;
		}

		// Reading image:
		//1) creating "IplImage" object
		IplImage* frame = cvQueryFrame(capture);
		if (!frame) {
			fprintf(stderr, "ERROR: frame is null...\n");
			getchar();
			break;
		}
		//2) using "IplImage" object to save image to file
		cvSaveImage("test.jpg", frame);

		//STARTING DETECTION:
		//1) reading image from file into Mat object
		cv::Mat bgr_image = cv::imread("test.jpg", cv::IMREAD_COLOR);
		cv::Mat orig_image = bgr_image.clone();

		// BLUR, nesessary for avoiding multiple detections of same object
		cv::medianBlur(bgr_image, bgr_image, 3); 

		// Convert input image to HSV
		cv::Mat hsv_image;
		cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

		// Threshold the HSV image, keep only the red pixels
		cv::Mat lower_red_hue_range;
		cv::Mat upper_red_hue_range;
		cv::inRange(hsv_image, cv::Scalar(0, 150, 150), cv::Scalar(10, 255, 255), lower_red_hue_range);
		cv::inRange(hsv_image, cv::Scalar(170, 150, 150), cv::Scalar(180, 255, 255), upper_red_hue_range);

		// Combine the above two images
		cv::Mat red_hue_image;
		cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

		cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

		// Use the Hough transform to detect circles in the combined threshold image
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 100, 20, 0, 0);

		
		if (circles.size() > 0) //IF something detected:
		{
		
			cv::Point center(round(circles[0][0]), round(circles[0][1]));
			int radius = round(circles[0][2]);
			//Printing Coordinates of centers
			cout << "Coordinates of circle are: X = " << center.x << ", Y = " << center.y << endl; 
			cv::circle(orig_image, center, radius, cv::Scalar(255, 0, 0), 5);

			x_coordinate = center.x;
			cout << "x_coordinate: " << center.x << endl;

			//determine direction (left or right) and 
			if(x_coordinate > 380)//object is in the right
			{
				angle = angle + x_coordinate/100; //move left 
				// calling SERVO script
    				string value = "python servo.py " + to_string(angle);
				something_not_sure_what = system (value.c_str()); 
			}

			if (x_coordinate < 280)//object is in the left
			{
				angle = angle - x_coordinate/100; // move right
				// calling SERVO script
    				string value = "python servo.py " + to_string(angle);
				something_not_sure_what = system (value.c_str()); 
			}
            
			//otherwise, do not move

		}// END of "SOMETHING DETECTED" condition

		// showing windows with images, removed in the final version		
		//string windowname;
		//windowname = "Detected red circles on the input image";
		//cv::namedWindow(windowname, cv::WINDOW_AUTOSIZE);
		//cv::imshow(windowname, orig_image);//showing window with image 

		if (circles.size() > 0)
			cout << "Color Detected" << endl;
		else
			cout << "Nothing" << endl;

			
		cvReleaseCapture(&capture); //stop capturing


		printf("Angle is:  %d\n", angle );

	//	sleep(1);//TIME delay, not used

	} //end of DETECTION LOOP

	cv::waitKey(0);

	return 0;
}
