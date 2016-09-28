/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  Cristian Rodriguez <u5419700@anu.edu.au>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "cylinder/cylMsg.h"
#include "cylinder/cylDataArray.h"

//OpenCV2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

#include "tape.hpp"
#include "cylinder.hpp"

static const std::string RGB_WINDOW   = "RGB window";
static const std::string DEPTH_WINDOW = "DEPTH_window";


using namespace sensor_msgs;
using namespace message_filters;

float median(std::vector<float> depths)
{
    if(depths.size() > 0) {
        float median;
        size_t size = depths.size();

        sort(depths.begin(), depths.end());

        if (size  % 2 == 0)
            median = (depths[size / 2 - 1] + depths[size / 2]) / 2;
        else 
            median = depths[size / 2];

        return median;
    }
}

class CylinderDetector
{
private:
    boost::mutex m;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;

    ros::Publisher cyl_pub_;
    cylinder::cylDataArray data;
    sensor_msgs::ImageConstPtr depthImage;
    sensor_msgs::ImageConstPtr rgbImage;    
    
    std::vector<cv::Rect> regions;
    int red1_high_H;
    int red1_high_S;
    int red1_high_V;
    int red1_low_H;
    int red1_low_S;
    int red1_low_V;
    
    int red2_high_H;
    int red2_high_S;
    int red2_high_V;
    int red2_low_H;
    int red2_low_S;
    int red2_low_V;    
    
    int blue_high_H;
    int blue_high_S;
    int blue_high_V;
    int blue_low_H;
    int blue_low_S;
    int blue_low_V;
    
    int green_high_H;
    int green_high_S;
    int green_high_V;
    int green_low_H;
    int green_low_S;
    int green_low_V;
    
    int yellow_high_H;
    int yellow_high_S;
    int yellow_high_V;
    int yellow_low_H;
    int yellow_low_S;
    int yellow_low_V;
    
public:
    CylinderDetector(cv::FileStorage colorCfg_): it_(nh_)
    {
        red1_high_H = colorCfg_["Red1.High.H"];
        red1_high_S = colorCfg_["Red1.High.S"];
        red1_high_V = colorCfg_["Red1.High.V"];        
        
        red1_low_H = colorCfg_["Red1.Low.H"];
        red1_low_S = colorCfg_["Red1.Low.S"];
        red1_low_V = colorCfg_["Red1.Low.V"];
        
        red2_high_H = colorCfg_["Red2.High.H"];
        red2_high_S = colorCfg_["Red2.High.S"];
        red2_high_V = colorCfg_["Red2.High.V"];        
        
        red2_low_H = colorCfg_["Red2.Low.H"];
        red2_low_S = colorCfg_["Red2.Low.S"];
        red2_low_V = colorCfg_["Red2.Low.V"];
        
        blue_high_H = colorCfg_["Blue.High.H"];
        blue_high_S = colorCfg_["Blue.High.S"];
        blue_high_V = colorCfg_["Blue.High.V"];        
        
        blue_low_H = colorCfg_["Blue.Low.H"];
        blue_low_S = colorCfg_["Blue.Low.S"];
        blue_low_V = colorCfg_["Blue.Low.V"];
        
        green_high_H = colorCfg_["Green.High.H"];
        green_high_S = colorCfg_["Green.High.S"];
        green_high_V = colorCfg_["Green.High.V"];        
        
        green_low_H = colorCfg_["Green.Low.H"];
        green_low_S = colorCfg_["Green.Low.S"];
        green_low_V = colorCfg_["Green.Low.V"];
        
        yellow_high_H = colorCfg_["Yellow.High.H"];
        yellow_high_S = colorCfg_["Yellow.High.S"];
        yellow_high_V = colorCfg_["Yellow.High.V"];        
        
        yellow_low_H = colorCfg_["Yellow.Low.H"];
        yellow_low_S = colorCfg_["Yellow.Low.S"];
        yellow_low_V = colorCfg_["Yellow.Low.V"];
        
        cv::namedWindow(RGB_WINDOW);
        cv::namedWindow(DEPTH_WINDOW);
        message_filters::Subscriber<Image> image_sub_(nh_,"/camera/rgb/image_color",5);
        message_filters::Subscriber<Image> depth_sub_(nh_,"/camera/depth_registered/image",5);
        typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        Synchronizer<MySyncPolicy>* sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub_, depth_sub_);
        sync->registerCallback(boost::bind(&CylinderDetector::callback, this, _1, _2));

        cyl_pub_ = nh_.advertise<cylinder::cylDataArray>("cylinderTopic",1);
        ros::Rate spin_rate(30);

        while( ros::ok() ) {
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    ~CylinderDetector()
    {
        cv::destroyWindow(RGB_WINDOW);
        cv::destroyWindow(DEPTH_WINDOW);
    }

    void callback(const sensor_msgs::ImageConstPtr& rgbImage, const sensor_msgs::ImageConstPtr& depthImage)
    {
        cv_bridge::CvImagePtr rgbPtr,depthPtr;
        std::vector<Tape> tapes;
        std::vector<Cylinder> cylinders;
        try
        {
            rgbPtr   = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::TYPE_8UC3);
            depthPtr = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit;
        }
        cv::Rect cropRoI(0,rgbPtr->image.rows/2,rgbPtr->image.cols,rgbPtr->image.rows/2);
        rgbPtr->image = rgbPtr->image(cropRoI);
        depthPtr->image = depthPtr->image(cropRoI);
        cv::imshow(RGB_WINDOW, rgbPtr->image);
        cv::imshow(DEPTH_WINDOW, depthPtr->image);
        cv::Mat imgHSV;
        cv::waitKey(3);
        cvtColor(rgbPtr->image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        std::vector<cv::Rect> redAreas = getAreasbyColor(imgHSV,RED);
        std::vector<cv::Rect> blueAreas = getAreasbyColor(imgHSV,BLUE);
        std::vector<cv::Rect> yellowAreas = getAreasbyColor(imgHSV,YELLOW);
        std::vector<cv::Rect> greenAreas = getAreasbyColor(imgHSV,GREEN);

        for( uint i=0; i< redAreas.size(); i++)
        {
            double centerX = redAreas[i].x + (double)(redAreas[i].width)/2;
            double centerY = redAreas[i].y + (double)(redAreas[i].height)/2;
            cv::Mat depthI = depthPtr->image(redAreas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c)*0.0 == 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
    //             double depth = 0.0;
                    depth = median(depths);
                }
//             std::cout << "Median " << depth << std::endl;
            Tape aux = {centerX,centerY,depth,RED};
            tapes.push_back(aux);
        }
        for( uint i=0; i< greenAreas.size(); i++)
        {
            double centerX = greenAreas[i].x + (double)(greenAreas[i].width)/2;
            double centerY = greenAreas[i].y + (double)(greenAreas[i].height)/2;
            cv::Mat depthI = depthPtr->image(greenAreas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c)*0.0 == 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
    //             double depth = 0.0;
                depth = median(depths);
            }
            Tape aux = {centerX,centerY,depth,GREEN};
            tapes.push_back(aux);
        }
        for( uint i=0; i< blueAreas.size(); i++)
        {
            double centerX = blueAreas[i].x + (double)(blueAreas[i].width)/2;
            double centerY = blueAreas[i].y + (double)(blueAreas[i].height)/2;
            cv::Mat depthI = depthPtr->image(blueAreas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
    //            std::cout << "med depth" << depthI.at<float>(depthI.cols/2,depthI.rows/2) << std::endl;
//                 std::cout << "Green " << depthI(RoI) << std::endl; 
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c)*0.0 == 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
    //             double depth = 0.0;
                depth = median(depths);
            }
            Tape aux = {centerX,centerY,depth,BLUE};
            tapes.push_back(aux);
        }
        for( uint i=0; i< yellowAreas.size(); i++)
        {
            double centerX = yellowAreas[i].x + (double)(yellowAreas[i].width)/2;
            double centerY = yellowAreas[i].y + (double)(yellowAreas[i].height)/2;
            cv::Mat depthI = depthPtr->image(yellowAreas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
    //            std::cout << "med depth" << depthI.at<float>(depthI.cols/2,depthI.rows/2) << std::endl;
//                 std::cout << "Green " << depthI(RoI) << std::endl; 
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c)*0.0 == 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
                depth = median(depths);
            }
            Tape aux = {centerX,centerY,depth,YELLOW};
            tapes.push_back(aux);
        }
        for(std::vector<Tape>::iterator ii=tapes.begin(); ii != tapes.end(); ii++)
        {
            Tape tap =  (*ii);
            std::cout << tap << std::endl;
            /**
            * Join tapes that belong to the same cylinder.
            */
            if(cylinders.empty())
            {
                Cylinder Aux;
                Aux.addTape(tap);
                cylinders.push_back(Aux);
            }
            else
            {
                bool isNewCylinder = true;
                for(std::vector<Cylinder>::iterator jj=cylinders.begin(); jj != cylinders.end() ;jj++)
                {
                    Cylinder cyl = (*jj);
                    if(cyl.addTape(tap) == true)
                    {
                        isNewCylinder = false;
                        break;
                    }
                }    
                if(isNewCylinder == true)
                {                
                    Cylinder Aux;
                    Aux.addTape(tap);
                    cylinders.push_back(Aux);
                }
            }
        }
        
        // Publisher!
        cylinder::cylDataArray arrayMsg;
        for (std::vector<Cylinder>::iterator ii = cylinders.begin(); ii != cylinders.end(); ii++)
        {
            if((*ii).getTape().size()==4 && (*ii).getLabel()!= 100 && (*ii).getZw() != 0.0)
            {
                cylinder::cylMsg aux;
                aux.header.stamp = ros::Time::now();
                aux.header.frame_id = "cylinderTopic";
                aux.Zrobot = (*ii).getZw();
                aux.Xrobot = (*ii).getXw();                
                aux.label  = (*ii).getLabel();
                const float covAux[] = { 0.0025000000000000000f, 0.0f, 0.0f, 0.0025000000000000000f};
                std::vector<float> data( covAux,covAux + sizeof( covAux ) / sizeof( covAux[0] ) );
                aux.covariance = data;
                arrayMsg.cylinders.push_back(aux);
            }
        }
        arrayMsg.header.stamp = ros::Time::now();
        arrayMsg.header.frame_id = "cylinderTopic";
        cyl_pub_.publish(arrayMsg);
        

    }

    std::vector<cv::Rect> getAreasbyColor(cv::Mat hsv_image,uint color=0)
    {
        cv::Mat imgThresholded;
        switch(color)
        {
        case GREEN:
            cv::inRange(hsv_image, cv::Scalar(green_low_H, green_low_S, green_low_V), cv::Scalar(green_high_H, green_high_S, green_high_V), imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
//             cv::imshow("GREEN",imgThresholded);
            break;
        case YELLOW:
            cv::inRange(hsv_image, cv::Scalar(yellow_low_H, yellow_low_S, yellow_low_V), cv::Scalar(yellow_high_H, yellow_high_S, yellow_high_V), imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
//             cv::imshow("YELLOW",imgThresholded);
            break;
        case BLUE:
            cv::inRange(hsv_image, cv::Scalar(blue_low_H, blue_low_S, blue_low_V), cv::Scalar(blue_high_H, blue_high_S, blue_high_V), imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
//             cv::imshow("BLUE",imgThresholded);
            break;
        case RED:
            cv::Mat lower_red_hue_range,upper_red_hue_range;
            cv::inRange(hsv_image, cv::Scalar(red1_low_H, red1_low_S, red1_low_V), cv::Scalar(red1_high_H, red1_high_S, red1_high_V), lower_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(red2_low_H, red2_low_S, red2_low_V), cv::Scalar(red2_high_H, red2_high_S,  red2_high_V), upper_red_hue_range);
            cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
//             cv::imshow("RED",imgThresholded);
            break;
        }

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat canny_output;
        cv::Canny(imgThresholded, canny_output, 100, 100*2, 3 );
        cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        /// Approximate contours to polygons + get bounding rects.
        std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
        std::vector<cv::Rect> boundRect( contours.size() ), ret;

        for(uint i=0; i < contours.size(); i++ ) {
            approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
            boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
        }

        std::vector<cv::Point2d> pts;
        for(uint i=0; i<boundRect.size(); i++)
        {
            cv::Mat region   = hsv_image(boundRect[i]);
            if(region.cols<=0 && region.rows<=0)
                continue;
            float proportion = (float)(region.cols)/(float)(region.rows);
            float area       = region.cols*region.rows;
            if(proportion > 1.6 && proportion < 2.4 && area > 200 && area < 5000)
            {

                bool flag = 1;
                for(uint j=0; j<pts.size(); j++)
                {
                    int deltaCx = (boundRect[i].width/2 + boundRect[i].x) - pts[j].x;
                    int deltaCy = (boundRect[i].height/2 + boundRect[i].y) - pts[j].y;
                    if( -10 <= deltaCx && deltaCx <= 10)
                    {
                        if(-10 <=  deltaCy && deltaCy <= 10)
                        {
                            flag = 0;
                        }
                    }
                }
                if(flag==1)
                {
                    cv::Point2d auxPt;
                    auxPt.x = boundRect[i].width/2 + boundRect[i].x;
                    auxPt.y = boundRect[i].height/2 + boundRect[i].y;
                    pts.push_back(auxPt);
                    ret.push_back(boundRect[i]);
                }
            }
        }

        return ret;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    std::string strSettingsFile =argv[1];
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << std::endl << "Wrong path to settings. Path must be absolut or relative to package directory." << std::endl;
        return 1;
    }
    CylinderDetector cylinder(fsSettings);

    return 0;
}
