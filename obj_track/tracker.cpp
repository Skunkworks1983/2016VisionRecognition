#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "Message.h"
#include "wiringPi.h"
#include <signal.h>
#include <netdb.h>
#include <list>
#include <cmath>

#define SERVER "roborio-1983-frc.local"
//#define SERVER "10.19.83.26"
#define PORT 17800
#define PORT_STR "PORT"

#define TOP_HEIGHT -1	//TODO: define
#define ANGLE -1		//TODO: define
#define MIN_AREA 2500
#define AREA_PERCENTAGE .3
#define ASPECT_RATIO (22.5 / 18)
#define INVALID 4200

using namespace cv;
using namespace std;

static volatile sig_atomic_t sig_caught = 0;

void handle_sighup(int signum){
  if(signum == SIGHUP){
    sig_caught = 1;
  }
}

struct Shape{
  int maxX;
  int maxY;
  int minX;
  int minY;
  double whitePercentage;
  
  int getArea(){
    return (maxX - minX) * (maxY - minY);
  }
  
  int getMidX(){
    return (maxX + minX) /2;
  }
  int getMidY(){
    return (maxY + minY) /2;
  }
  
  float getAspectRatio(){
     return (maxY - minY) / (maxX - minX);
  }
  
  float getPercentError(){
     return fabs(ASPECT_RATIO - this->getAspectRatio()) / ASPECT_RATIO;
  }
};

void sendMessage(int file_descriptor, addrinfo *info, int midX, int midY){
  Message msg;
  msg.posX = midX;
  msg.posY = midY;

  if (sendto(file_descriptor, (unsigned char*) &msg, sizeof(msg), 0, info->ai_addr, info->ai_addrlen) == -1) {
    printf("sendto failed\n");
  }
}

int main(int argc, char** argv) {
	signal(SIGHUP, handle_sighup);
  
	std::list<unsigned long> times;
	
	bool hasDisplay = argc > 1;
	
	cout << "Start of program\n";
	VideoCapture cap(0); //capture the video from webcam
	printf("after cap\n");

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	system("gpio export 18 out");

	if (wiringPiSetupSys() == -1) {
		printf("ERROR Cannot Open wiringPi\n");
		exit(1);
	}
	int width = 640;
	int height = 480;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
	
	if(hasDisplay){
	  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	}
	
	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0;
	int iHighS = 100;

	int iLowV = 214;
	int iHighV = 255;

	if(hasDisplay){

	  //Create trackbars in "Control" window
	  createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	  createTrackbar("HighH", "Control", &iHighH, 179);

	  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	  createTrackbar("HighS", "Control", &iHighS, 255);

	  createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	  createTrackbar("HighV", "Control", &iHighV, 255);

	}
	int iLastX = -1;
	int iLastY = -1;

	//Capture a temporary image from the camera
	Mat imgTmp;
	cap.read(imgTmp);

	printf("Beginning read loop\n");

	struct addrinfo hints, *servinfo, *p;

	int s, i, slen = sizeof(sockaddr_in);
	
printf("1\n");
	

	memset((char *) &hints, 0, sizeof(hints));
	//hints.sin_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_family = AF_UNSPEC;
	hints.ai_flags = AI_NUMERICSERV;
	hints.ai_protocol = 0;
	//hints.sin_port = htons(PORT);
	
printf("2\n");

	int rv;
	
	if ( (rv = getaddrinfo(SERVER, "17800", &hints, &servinfo)) != 0) {
	  printf("getaddrinfo() failed %s\n", gai_strerror(rv));
	}
printf("3\n");

	if ((s = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol)) == -1) {
	  printf("Could not create socket, exiting...\n");
	  exit(0);
	}
	
	printf("4\n");
	
	//pinMode(18, OUTPUT);
	digitalWrite(18, 1);

	while (true) {
		Mat imgOriginal;
		
		timeval val;
		gettimeofday(&val, NULL);
		unsigned long long time = val.tv_sec * 1000000 + val.tv_usec;
		
		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}
		
		gettimeofday(&val, NULL);		
		times.push_back((val.tv_sec * 1000000 + val.tv_usec) - time);
		time = val.tv_sec * 1000000 + val.tv_usec;
		
		cv::flip(imgOriginal, imgOriginal, 0);
		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV),
				Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//morphological opening (removes small objects from the foreground)
		/*erode(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded,
				getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
*/
		//morphological closing (removes small holes from the foreground)
		//dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		//erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		gettimeofday(&val, NULL);		
		times.push_back((val.tv_sec * 1000000 + val.tv_usec) - time);
		time = val.tv_sec * 1000000 + val.tv_usec;
		
		Mat sum;
		//cv::reduce(imgThresholded, sum, 0, CV_REDUCE_SUM, CV_32S);

		std::vector<std::vector<cv::Point> > contours;
		cv::Mat contourOutput = imgThresholded.clone();
		cv::findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		
		gettimeofday(&val, NULL);		
		times.push_back((val.tv_sec * 1000000 + val.tv_usec) - time);
		time = val.tv_sec * 1000000 + val.tv_usec;
		
		int midX = -1, midY = -1;
		std::vector<Shape> shapes;

		if (contours.size() > 0) {
			float distance = -1;

			for (int i = 0; i < contours.size(); i++) {
				vector<Point> poly;
				cv::approxPolyDP(contours[i], poly, 5, true);

				double whiteSample = 0, whitePercentage = 0;
				
				Shape shape;
				
				int maxX = 0;
				int maxY = 0;
				int minX = 640;
				int minY = 480;

				int x = poly[0].x;
				int y = poly[0].y;

				for (size_t k2 = 0; k2 < poly.size(); ++k2) {
					maxX = (maxX < poly[k2].x ? poly[k2].x : maxX);
					maxY = (maxY < poly[k2].y ? poly[k2].y : maxY);
					minX = (minX > poly[k2].x ? poly[k2].x : minX);
					minY = (minY > poly[k2].y ? poly[k2].y : minY);
				}
				
				shape.minX = minX;
				shape.minY = minY;
				shape.maxX = maxX;
				shape.maxY = maxY;
				
				if(hasDisplay){
				line(imgOriginal, Point(minX, minY), Point(minX, maxY),
						Scalar(255, 0, 0), 2);
				line(imgOriginal, Point(minX, maxY), Point(maxX, maxY),
						Scalar(255, 0, 0), 2);
				line(imgOriginal, Point(maxX, maxY), Point(maxX, minY),
						Scalar(255, 0, 0), 2);
				line(imgOriginal, Point(maxX, minY), Point(minX, minY),
						Scalar(255, 0, 0), 2);
				}
				
				unsigned indexLeft = i - 1 < 0 ? contours.size() : i - 1;
				unsigned indexRight = i + 1 == contours.size() ? 0 : i + 1;
				Point mid = Point((poly[indexLeft].x + poly[indexRight].x) / 2,
						(poly[indexLeft].y + poly[indexRight].y) / 2);

				int originX = x;
				int originY = y;
				if(hasDisplay){

				for (size_t k2 = 0; k2 < poly.size(); ++k2) {
					const Point& p = poly[k2];
					line(imgOriginal, Point(x, y), p, Scalar(0, 0, 255), 2);
					x = p.x;
					y = p.y;
				}
				line(imgOriginal, Point(x, y), Point(originX, originY),
						Scalar(0, 0, 255), 2);
				}
				  
				Rect roi(minX, minY, (maxX-minX), (maxY-minY));
				Mat crop = imgThresholded(roi);
				
				for(int xi = 0; xi < crop.rows; xi++){
				  for(int yi = 0; yi < crop.cols; yi++){
				    if(crop.at<uchar>(xi,yi) > 0){
				      whiteSample++;
				    }
				  }
				}
				whitePercentage = whiteSample / (crop.rows * crop.cols);
				
				shape.whitePercentage = whitePercentage;
								
				midX = (maxX + minX)/2;
				midY = (maxY + minY)/2;

				//printf("poly.size(): %d\n", poly.size());
				if(hasDisplay){
				  line(imgOriginal, Point(midX, 0),
						  Point(midX, 480), Scalar(0, 255, 0), 2);
				}
				
				shapes.push_back(shape);
			}
			
			Shape *bestShape = NULL;
			
			for(int i = 0; i < shapes.size(); i++){
			  Shape *shape = &shapes[i];
			  if(shape->getArea() > MIN_AREA){
			  if(shape->whitePercentage <= AREA_PERCENTAGE + .1 && shape->whitePercentage >= AREA_PERCENTAGE -.1){
			    
			    if(bestShape == NULL){
			      bestShape = shape;
			    }else{
				if(shape->getArea() / shape->getPercentError() > bestShape->getArea() / bestShape->getPercentError()){
				  bestShape = shape;
				}
			      }
			    }
			  }
			    
			  }
			
			gettimeofday(&val, NULL);		
			times.push_back((val.tv_sec * 1000000 + val.tv_usec) - time);
			time = val.tv_sec * 1000000 + val.tv_usec;
		
			if(bestShape != NULL){
			  if(hasDisplay){
			    line(imgOriginal, Point(bestShape->getMidX(), 0), Point(bestShape->getMidX(), 480), Scalar(0, 255, 255), 4);
			  }

			  sendMessage(s, servinfo, bestShape->getMidX(), bestShape->getMidY());
			}else{
			  sendMessage(s, servinfo, INVALID, INVALID);
			}
		} else {
			//printf("NO CONTOURS!!\n");
			sendMessage(s, servinfo, INVALID, INVALID);
		}
		if(hasDisplay){
		  imshow("Original", imgOriginal); //show the original image
		  imshow("Threshold", imgThresholded);
		
		  if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
				{
			cout << "esc key is pressed by user" << endl;
			break;
		  }
		}
		if(sig_caught){
		  printf("Caught a SIGHUP!\n");
		  break;
		}
	}

	/*for(int i = 0; i < times.size(); i++){
	  printf("i: %d first: %ld", i, times.front());
	  times.pop_front();
	  printf("\tsecond: %ld", times.front());
	  times.pop_front();
	  printf("\tthird: %ld\n", times.front());
	  times.pop_front();
	  printf("\tfourth: %ld\n", times.front());
	  times.pop_front();
	}*/
	digitalWrite(18, 0);

	return 0;
}
