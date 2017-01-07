#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Shape.h"
#include <cmath>

Shape::Shape(cv::vector<cv::Point> p)
{
  int maxX = 0;
  int maxY = 0;
  int minX = 10000; // arbitrary number larger than resolution
  int minY = 10000; // arbitrary number larger than resolution

  for (size_t k2 = 0; k2 < p.size(); ++k2) 
  {
    maxX = (maxX < p[k2].x ? p[k2].x : maxX);
    maxY = (maxY < p[k2].y ? p[k2].y : maxY);
    minX = (minX > p[k2].x ? p[k2].x : minX);
    minY = (minY > p[k2].y ? p[k2].y : minY);
  }

  this->minX = minX;
  this->minY = minY;
  this->maxX = maxX;
  this->maxY = maxY;
}
int Shape::getArea() {
	return (maxX - minX) * (maxY - minY);
}

int Shape::getMidX() {
	return (maxX + minX) / 2;
}
int Shape::getMidY() {
	return (maxY + minY) / 2;
}

float Shape::getAspectRatio() {
	return (maxY - minY) / (maxX - minX);
}

float Shape::getPercentError() {
	return fabs(ASPECT_RATIO - this->getAspectRatio()) / ASPECT_RATIO;
}

bool Shape::isBetter(Shape *shape) {
	return this->getArea() / this->getPercentError()
			> shape->getArea() / shape->getPercentError();
}
void Shape::calcWhitePercentage(cv::Mat imgThresh)
{
  double whiteSample = 0;
  
  cv::Rect roi(this->minX, this->minY, (this->maxX - this->minX), (this->maxY - this->minY));
  cv::Mat crop = imgThresh(roi);

  for (int xi = 0; xi < crop.rows; xi++) 
  {
    for (int yi = 0; yi < crop.cols; yi++) 
    {
      if (crop.at < uchar > (xi, yi) > 0) 
      {
	whiteSample++;
      }
    }
  }
  this->whitePercentage = whiteSample / (crop.rows * crop.cols);
}
bool Shape::shapeIsViable()
{
  // if the white percentage is outside the acceptable range
  // the shape is not a valid candidate.
  if (this->whitePercentage <= AREA_PERCENTAGE - .1) 
  {
    return (false);
  }
  if (this->whitePercentage >= AREA_PERCENTAGE + .1) 
  {
    return (false);	
  }
  
  // if the area is too small, the shape is also not a valid candidate
  if (this->getArea() < MIN_AREA) 
  {
    return (false);
  }
  return (true);
}