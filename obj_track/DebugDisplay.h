#ifndef DEBUGDISPLAY
#define DEBUGDISPLAY

class DebugDisplay
{
  private:
    cv::Mat *imgOrig;
    cv::Mat *imgThresh;
    bool exitRequested;
  public:

    DebugDisplay(cv::Mat *imgOrig, cv::Mat *imgThresh, int *lowHue, int *highHue, int *lowSat, int *highSat, int *lowVal, int *highVal);
    void drawBoundingBox(Shape *s);
    void drawContours(std::vector<cv::Point> poly);
    void drawCenter(Shape *s);
    void drawBestPos(Shape *best);
    void showImages();
    bool shouldExit();

};

#endif
