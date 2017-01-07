#ifndef SHAPE_H
#define SHAPE_H

#define ASPECT_RATIO (22.5 / 18)
#define MIN_AREA 2500
#define AREA_PERCENTAGE .3

class Shape {
  public:
    int maxX;
    int maxY;
    int minX;
    int minY;
    double whitePercentage;

    Shape(cv::vector<cv::Point> p);
    
    int getArea();

    int getMidX();
    int getMidY();

    float getAspectRatio();

    float getPercentError();

    bool isBetter(Shape *shape);
    void calcWhitePercentage(cv::Mat imgThresh);
    bool shapeIsViable();
};

#endif
