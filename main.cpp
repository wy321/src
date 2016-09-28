#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;    

    //Create trackbars in "Control" window
public:  
    int iLowH;
    int iHighH;

    int iLowS;
    int iHighS;

    int iLowV;
    int iHighV;

  //Create trackbars in "Control" window
    ImageConverter() : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        iLowH = 0;
        iHighH = 179;

        iLowS = 0;
        iHighS = 255;

        iLowV = 0;
        iHighV = 255;

        namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
            //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 179);

        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);

        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        Mat imgHSV;
        cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        
        Mat imgThresholded;
        // Green Cylinder

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ERODE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_DILATE, Size(5, 5)) );

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ERODE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_DILATE, Size(5, 5)) );

        imshow("Thresholded Image", imgThresholded); //show the thresholded image

        cv::waitKey(3);
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }

  cv::Mat histogram(std::string const& name, cv::Mat1b const& image)
  {
      // Set histogram bins count
      int bins = image.cols;
      int const hist_height = image.rows;
      cv::Mat hist_image(cv::Mat::zeros(hist_height, bins,CV_8U));
      cv::Mat array(cv::Mat::zeros(1,image.cols,CV_32S));

      cv::reduce(image,array,0,CV_REDUCE_SUM,CV_32S);
      //minMaxLoc(hist, 0, &max_val)
      // visualize each bin
      for(int b = 0; b < bins; b++) {
          float const binVal = array.at<int>(b)/255;
          cv::line( hist_image
                , cv::Point(b, 0), cv::Point(b, binVal)
                , cv::Scalar::all(255)
                );
        }
      return hist_image;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

