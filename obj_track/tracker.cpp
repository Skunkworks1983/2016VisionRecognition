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
#include "Message.h"

#include "wiringPi.h"
#include <signal.h>
#include <list>
#include <cmath>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define SERVER "roborio-1983-frc.local"
#define SERVER "localhost"
#define PORT 17800
#define PORT_STR "PORT"

#define TOP_HEIGHT -1	//TODO: define
#define ANGLE -1		//TODO: define
#define INVALID 4200

#define CAP_WIDTH 640
#define CAP_HEIGHT 480

#include "Shape.h"
#include "DebugDisplay.h"
#include "MessageSender.h"

using namespace cv;
using namespace std;

static volatile sig_atomic_t sig_caught = 0;

void handle_sighup(int signum) {
	if (signum == SIGHUP) {
		sig_caught = 1;
	}
}

void selectBest3Shapes(std::vector<Shape*> shapeList, Shape **best1, Shape **best2, Shape **best3)
{
  for (int i = 0; i < shapeList.size(); i++) 
  {
    Shape *tmp = shapeList[i];
    if (tmp->isBetter(*best1)) 
    {
	    *best1 = tmp;
    } 
    else if (tmp->isBetter(*best2)) 
    {
	    *best2 = tmp;
    } 
    else if (tmp->isBetter(*best3)) 
    {
	    *best3 = tmp;
    }
  }
}

void extractViableShapes(DebugDisplay *dd, cv::Mat imgThresh, std::vector<cv::Point> contour, std::vector<Shape*> shapeList)
{
      vector<Point> poly;
      cv::approxPolyDP(contour, poly, 5, true);

      // Maybe move to a temp heap allocator
      Shape *shape = new Shape(poly);
      dd->drawBoundingBox(shape);

//      unsigned indexLeft = i - 1 < 0 ? contours.size() : i - 1;
//      unsigned indexRight = i + 1 == contours.size() ? 0 : i + 1;
      Point mid = Point((shape->minX + shape->maxX) / 2,
		      (shape->minY + shape->maxY) / 2);

      dd->drawContours(poly);

      shape->calcWhitePercentage(imgThresh);
      
      // delete shape if it is not viable
      if (!shape->shapeIsViable())
      {
	delete shape;
	return;
      }
      
      // if we made it to here, the shape is a valid candidate
      // add it to the list for consideration
      shapeList.push_back(shape);
      dd->drawCenter(shape);
}

int main(int argc, char** argv) 
{
  signal(SIGHUP, handle_sighup);

  std::list<unsigned long> times;

  bool hasDisplay = argc > 1;
  hasDisplay=1;

  cout << "Start of program\n";
  VideoCapture cap(0); //capture the video from webcam
  printf("after cap\n");

  if (!cap.isOpened())  // if not success, exit program
  {
    cout << "Cannot open the web cam" << endl;
    return -1;
  }

  system("gpio export 18 out");

  if (wiringPiSetupSys() == -1) 
  {
    printf("ERROR Cannot Open wiringPi\n");
    exit(1);
  }
  int width = CAP_WIDTH;
  int height = CAP_HEIGHT;
  cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);

  int iLowH = 0;
  int iHighH = 179;
  iHighH=50;

  int iLowS = 0;
  int iHighS = 100;

  int iLowV = 214;
  int iHighV = 255;
  iLowV=0;

  int iLastX = -1;
  int iLastY = -1;

  //Capture a temporary image from the camera
  Mat imgTmp;
  cap.read(imgTmp);

  printf("Beginning read loop\n");

  MessageSender *ms = new MessageSender(SERVER, "17800");
  
  digitalWrite(18, 1);

  Mat imgOriginal;
  Mat imgHSV;
  Mat imgThresholded;
  Shape *shape;
  DebugDisplay *dDisp = NULL;
  if (hasDisplay)
  {
    dDisp = new DebugDisplay(&imgOriginal, &imgThresholded, &iLowH, &iHighH, &iLowS, &iHighS, &iLowV, &iHighV);
  }
  else
  {
    dDisp = new DebugDisplay(NULL, NULL, &iLowH, &iHighH, &iLowS, &iHighS, &iLowV, &iHighV);
  }

  while (!dDisp->shouldExit()) 
  {
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

    if (contours.size() == 0) 
    {
      ms->sendMessage(NULL,NULL,NULL);
      continue;
    }

    float distance = -1;
    int midX = -1, midY = -1;

    for (int i = 0; i < contours.size(); i++) 
    {
      extractViableShapes(dDisp,imgThresholded, contours[i],bestShapes);
    }

    // Find best 3 shapes
    Shape * best1=NULL;
    Shape * best2=NULL;
    Shape * best3=NULL;
 
    selectBest3Shapes(bestShapes, &best1, &best2, &best3);

    ms->sendMessage(best1, best2, best3);
    if (best1 != NULL) 
    {
      dDisp->drawBestPos(best1);
    } 
    dDisp->showImages();
  }
  if (sig_caught) {
    printf("Caught a SIGHUP!\n");
    //break;
  }

  digitalWrite(18, 0);

  return 0;
}
