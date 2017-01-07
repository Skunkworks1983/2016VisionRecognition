#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Shape.h"
#include "DebugDisplay.h"

DebugDisplay::DebugDisplay(cv::Mat *imgOrig, cv::Mat *imgThresh, int *lowHue, int *highHue, int *lowSat, int *highSat, int *lowVal, int *highVal)
{
  exitRequested=false;

  this->imgOrig=imgOrig;
  this->imgThresh=imgThresh;
  if (!imgOrig)
  {
    return;
  }
  cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  //Create trackbars in "Control" window
  cv::createTrackbar("LowH", "Control", lowHue, 179); //Hue (0 - 179)
  cv::createTrackbar("HighH", "Control", highHue, 179);

  cv::createTrackbar("LowS", "Control", lowSat, 255); //Saturation (0 - 255)
  cv::createTrackbar("HighS", "Control", highSat, 255);

  cv::createTrackbar("LowV", "Control", lowVal, 255); //Value (0 - 255)
  cv::createTrackbar("HighV", "Control", highVal, 255);
};

void DebugDisplay::drawBoundingBox(Shape *s)
{
	if (imgOrig) 
	{
		cv::line(*imgOrig, cv::Point(s->minX, s->minY), cv::Point(s->minX, s->maxY),
		  cv::Scalar(255, 0, 0), 2);
		cv::line(*imgOrig, cv::Point(s->minX, s->maxY), cv::Point(s->maxX, s->maxY),
		  cv::Scalar(255, 0, 0), 2);
		cv::line(*imgOrig, cv::Point(s->maxX, s->maxY), cv::Point(s->maxX, s->minY),
		  cv::Scalar(255, 0, 0), 2);
		cv::line(*imgOrig, cv::Point(s->maxX, s->minY), cv::Point(s->minX, s->minY),
		  cv::Scalar(255, 0, 0), 2);
	}
}

void DebugDisplay::drawContours(std::vector<cv::Point> poly)
{
  int x = poly[0].x;
  int y = poly[0].y;
  int originX = x;
  int originY = y;
  
  if (imgOrig) 
  {

    for (size_t k2 = 0; k2 < poly.size(); ++k2) 
    {
      const cv::Point& p = poly[k2];
      cv::line(*imgOrig, cv::Point(x, y), p, cv::Scalar(0, 0, 255), 2);
      x = p.x;
      y = p.y;
    }
    cv::line(*imgOrig, cv::Point(x, y), cv::Point(originX, originY),
      cv::Scalar(0, 0, 255), 2);
  }
}

void DebugDisplay::drawCenter(Shape *s)
{
  if (imgOrig) 
  {
    int midX = (s->maxX + s->minX) / 2;
    int midY = (s->maxY + s->minY) / 2;

    cv::line(*imgOrig, cv::Point(midX, 0), cv::Point(midX, 480),
      cv::Scalar(0, 255, 0), 2);
  }
}

void DebugDisplay::drawBestPos(Shape *best)
{
  if (imgOrig) 
  {
    cv::line(*imgOrig, cv::Point(best->getMidX(), 0),
      cv::Point(best->getMidX(), 480), cv::Scalar(0, 255, 255), 4);
  }
}

void DebugDisplay::showImages()
{
  if (imgOrig) 
  {
    cv::imshow("Original", *imgOrig); //show the original image
    cv::imshow("Threshold", *imgThresh);

    if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
      std::cout << "esc key is pressed by user" << std::endl;
      exitRequested=true;      
    }
  }
}

bool DebugDisplay::shouldExit()
{
  return exitRequested;
}