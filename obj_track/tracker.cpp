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
#include "Shape.h"
#include "wiringPi.h"
#include <signal.h>
#include <netdb.h>
#include <list>
#include <cmath>

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define SERVER "roborio-1983-frc.local"
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

void handle_sighup(int signum) {
	if (signum == SIGHUP) {
		sig_caught = 1;
	}
}

void sendMessage(int file_descriptor, addrinfo *info, Shape *left = NULL,
		Shape *middle = NULL, Shape *right = NULL) {
	Message msg;
	if (left != NULL) {
		msg.posX1 = (left->maxX + left->minX) / 2;
		msg.posY1 = (left->maxY + left->minY) / 2;
	}

	if (middle != NULL) {
		msg.posX2 = (middle->maxX + middle->minX) / 2;
		msg.posY2 = (middle->maxY + middle->minY) / 2;
	}

	if (right != NULL) {
		msg.posX3 = (right->maxX + right->minX) / 2;
		msg.posY3 = (right->maxY + right->minY) / 2;
	}

	if (sendto(file_descriptor, (unsigned char*) &msg, sizeof(msg), 0,
			info->ai_addr, info->ai_addrlen) == -1) {
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

	if (hasDisplay) {
		namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	}

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0;
	int iHighS = 100;

	int iLowV = 214;
	int iHighV = 255;

	if (hasDisplay) {

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

	memset((char *) &hints, 0, sizeof(hints));
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_family = AF_UNSPEC;
	hints.ai_flags = AI_NUMERICSERV;
	hints.ai_protocol = 0;

	int rv;

	if ((rv = getaddrinfo(SERVER, "17800", &hints, &servinfo)) != 0) {
		printf("getaddrinfo() failed %s\n", gai_strerror(rv));
		exit(0);
	}

	if ((s = socket(servinfo->ai_family, servinfo->ai_socktype,
			servinfo->ai_protocol)) == -1) {
		printf("Could not create socket, exiting...\n");
		exit(0);
	}

	digitalWrite(18, 1);

	Mat imgOriginal;
	Mat imgHSV;
	Mat imgThresholded;
	Shape *shape;

	while (true) {
		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		cv::flip(imgOriginal, imgOriginal, 0);

		//Convert the captured frame from BGR to HSV
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		//Mask the image
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV),
				Scalar(iHighH, iHighS, iHighV), imgThresholded);

		std::vector<std::vector<cv::Point> > contours;
		cv::Mat contourOutput = imgThresholded.clone();
		cv::findContours(contourOutput, contours, CV_RETR_LIST,
				CV_CHAIN_APPROX_SIMPLE);

		std::vector<Shape*> bestShapes;

		if (contours.size() == 0) {
			sendMessage(s, servinfo);
			continue;
		}
		float distance = -1;
		int midX = -1, midY = -1;

		for (int i = 0; i < contours.size(); i++) {
			vector<Point> poly;
			cv::approxPolyDP(contours[i], poly, 5, true);

			double whiteSample = 0;
			double whitePercentage = 0;

			// Maybe move to a temp heap allocator
			shape = new Shape();

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

			if (hasDisplay) {
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
			if (hasDisplay) {

				for (size_t k2 = 0; k2 < poly.size(); ++k2) {
					const Point& p = poly[k2];
					line(imgOriginal, Point(x, y), p, Scalar(0, 0, 255), 2);
					x = p.x;
					y = p.y;
				}
				line(imgOriginal, Point(x, y), Point(originX, originY),
						Scalar(0, 0, 255), 2);
			}

			Rect roi(minX, minY, (maxX - minX), (maxY - minY));
			Mat crop = imgThresholded(roi);

			for (int xi = 0; xi < crop.rows; xi++) {
				for (int yi = 0; yi < crop.cols; yi++) {
					if (crop.at < uchar > (xi, yi) > 0) {
						whiteSample++;
					}
				}
			}
			whitePercentage = whiteSample / (crop.rows * crop.cols);

			const bool wpLessThanMin = shape->whitePercentage
					<= AREA_PERCENTAGE - .1;
			const bool wpGreaterThanMax = shape->whitePercentage
					>= AREA_PERCENTAGE + .1;
			const bool areaLessThanMin = shape->getArea() < MIN_AREA;

			if (wpLessThanMin) {
				delete shape;
				break;
			}
			if (wpGreaterThanMax) {
				delete shape;
				break;
			}
			if (areaLessThanMin) {
				delete shape;
				break;
			}

			shape->whitePercentage = whitePercentage;

			bestShapes.push_back(shape);
		}

		if (hasDisplay) {
			midX = (maxX + minX) / 2;
			midY = (maxY + minY) / 2;

			line(imgOriginal, Point(midX, 0), Point(midX, 480),
					Scalar(0, 255, 0), 2);
		}

		// Find best 3 shapes
		Shape * best1;
		Shape * best2;
		Shape * best3;

		for (int i = 0; i < bestShapes.size(); i++) {
			Shape *tmp = bestShapes[i];
			if (tmp->isBetter(best1)) {
				best1 = tmp;
			} else if (tmp->isBetter(best2)) {
				best2 = tmp;
			} else if (tmp->isBetter(best3)) {
				best3 = tmp;
			}
		}

		if (best1 != NULL) {
			if (hasDisplay) {
				line(imgOriginal, Point(best1->getMidX(), 0),
						Point(best1->getMidX(), 480), Scalar(0, 255, 255), 4);
			}

			sendMessage(s, servinfo, best1, best2, best3);
		} else {
			sendMessage(s, servinfo);
		}
	} //while true
	else {
		sendMessage(s, servinfo, INVALID, INVALID);
	}
	if (hasDisplay) {
		imshow("Original", imgOriginal); //show the original image
		imshow("Threshold", imgThresholded);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}
	if (sig_caught) {
		printf("Caught a SIGHUP!\n");
		break;
	}

	digitalWrite(18, 0);

	return 0;
}
