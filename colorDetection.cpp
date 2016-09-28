#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;
static const uint BLUE   = 1;
static const uint RED    = 2;
static const uint GREEN  = 3;
static const uint YELLOW = 4;

cv::Mat getAreasbyColor(cv::Mat hsv_image,uint color=0)
{
    int lowH,lowS,lowV;
    int highH,highS,highV;
    cv::Mat ret;
    switch(color)
    {
        case BLUE:
            lowH  = 90;
            highH = 120;
            lowS  = 84;
            highS = 255;
            lowV  = 0;
            highV = 255;
            break; 
        case RED:
            cv::Mat lower_red_hue_range,upper_red_hue_range;
            cv::inRange(hsv_image, cv::Scalar(0, 100, 0), cv::Scalar(10, 255, 255), lower_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(160, 90, 40), cv::Scalar(180, 255, 255), upper_red_hue_range);
            cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, ret);
            break;

    }
    return ret;
}


int main( int argc, char** argv )
{
    VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    int iLowH1 = 0;
    int iHighH1 = 179;

    int iLowH2 = 0;
    int iHighH2 = 179;
    
    int iLowS1 = 0;
    int iHighS1 = 255;
    
    int iLowS2 = 0;
    int iHighS2 = 255;

    int iLowV1 = 0;
    int iHighV1 = 255;
    
    int iLowV2 = 0;
    int iHighV2 = 255;

//Create trackbars in "Control" window
    cvCreateTrackbar("LowH1", "Control", &iLowH1, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH1", "Control", &iHighH1, 179);

    cvCreateTrackbar("LowS1", "Control", &iLowS1, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS1", "Control", &iHighS1, 255);

    cvCreateTrackbar("LowV1", "Control", &iLowV1, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV1", "Control", &iHighV1, 255);
    
    
    cvCreateTrackbar("LowH2", "Control", &iLowH2, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH2", "Control", &iHighH2, 179);

    cvCreateTrackbar("LowS2", "Control", &iLowS2, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS2", "Control", &iHighS2, 255);

    cvCreateTrackbar("LowV2", "Control", &iLowV2, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV2", "Control", &iHighV2, 255);

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        Mat imgHSV;

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;
        imgThresholded = getAreasbyColor(imgHSV,RED);
        cv::Mat lower_red_hue_range,upper_red_hue_range;
        inRange(imgHSV, Scalar(iLowH1, iLowS1, iLowV1), Scalar(iHighH1, iHighS1, iHighV1), lower_red_hue_range); //Threshold the image
        inRange(imgHSV, Scalar(iLowH2, iLowS2, iLowV2), Scalar(iHighH2, iHighS2, iHighV2), upper_red_hue_range); //Threshold the image
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, imgThresholded);

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

    return 0;

}

