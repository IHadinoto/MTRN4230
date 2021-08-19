#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <iostream>
#include <algorithm>
#include <iterator>
#include <tuple>
#include <cmath>

bool compareYX(const cv::Point2f& p1, const cv::Point2f& p2) {
    return p1.y > p2.y;
}

void getCyanCornerstone(cv::Mat originalHsv, std::vector<cv::Point2f> &cornerLocs) {
    cv::Mat thresholdedHsvFrame;
    // Detect the object based on HSV Range Values
    cv::inRange(originalHsv, cv::Scalar(78, 155, 246), cv::Scalar(96, 193, 255), thresholdedHsvFrame);

    cv::Mat cannyOutput;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    //detect edges using canny
    cv::Canny(thresholdedHsvFrame, cannyOutput, 0, 255, 3);
    //find contours to be able to get their centres
    cv::findContours(cannyOutput,contours,hierarchy,
        cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // get the moments
    std::vector<cv::Moments> mu(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        mu[i] = cv::moments(contours[i], false);
    }

    // get the centroid of figures.
    std::vector<cv::Point2f> mc(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        mc[i] = cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
    }
    if (mc.size() != 0) {
        cornerLocs[1] = mc[0];
    }
}

void getMagentaCornerstones(cv::Mat originalHsv, std::vector<cv::Point2f> &cornerLocs) {
    cv::Mat thresholdedHsvFrame;
    // Detect the object based on HSV Range Values
    cv::inRange(originalHsv, cv::Scalar(144, 186, 167), cv::Scalar(149, 225, 255), thresholdedHsvFrame);

    cv::Mat cannyOutput;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    //detect edges using canny
    cv::Canny(thresholdedHsvFrame, cannyOutput, 0, 255, 3);
    //find contours to be able to get their centres
    cv::findContours(cannyOutput,contours,hierarchy,
        cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // get the moments
    std::vector<cv::Moments> mu(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        mu[i] = cv::moments(contours[i], false);
    }
    std::vector<cv::Point2f> magentaLocs;
    // get the centroid of figures.
    std::vector<cv::Point2f> mc(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        mc[i] = cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
        magentaLocs.push_back(mc[i]);
    }

    // Sort magenta centroids
    for (int j=0; j<magentaLocs.size(); j++) {
        float xDiff = std::abs(magentaLocs[j].x - cornerLocs[1].x);
        float yDiff = std::abs(magentaLocs[j].y - cornerLocs[1].y);
        if (xDiff > 50) {
            if (yDiff > 50) {
                cornerLocs[3] = magentaLocs[j];
            } else {
                cornerLocs[2] = magentaLocs[j];
            }
        } else if (yDiff > 50) {
            cornerLocs[0] = magentaLocs[j];
        }
    }
}

cv::Mat transformPerspective(cv::Mat original_bgr_frame, std::vector<cv::Point2f> &cornerLocs) {
    cv::Mat lambda(2,4,CV_32FC1), outputImage;
    cv::Point2f inputQuad[4];
    for (int i=0; i<4; i++) {
        inputQuad[i] = cornerLocs[i];
    }
    cv::Point2f outputQuad[4] = {cv::Point2f(0.0,0.0), cv::Point2f(500.0,0.0), cv::Point2f(500.0,500.0), cv::Point2f(0.0,500.0)};
    lambda = getPerspectiveTransform(inputQuad, outputQuad);
    // Apply the Perspective Transform just found to the src image
    warpPerspective(original_bgr_frame, outputImage, lambda, cv::Size(500,500));
    return outputImage;
}

class BlockDetector {
    public:
        void establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic);
        void publishData(std::vector<float> pos, cv::Mat image);
        void callback(const sensor_msgs::ImageConstPtr& msg);
    private:
        ros::Publisher mPub;
        image_transport::Subscriber mSub;
        image_transport::Publisher imagePub;
};

// Establishes the channels
void BlockDetector::establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic) {
    mPub = nodeHandle.advertise<std_msgs::Float32MultiArray>(pubTopic, 1000);
    image_transport::ImageTransport it(nodeHandle);
    mSub = it.subscribe(subTopic, 1, &BlockDetector::callback, this);
    imagePub = it.advertise("camera_image", 1);
}

// // Publishes block centroid locations
void BlockDetector::publishData(std::vector<float> pos, cv::Mat image) {
    std_msgs::Float32MultiArray positions;
    positions.layout.dim.push_back(std_msgs::MultiArrayDimension());
    positions.layout.dim[0].label = "x_pos";
    positions.layout.dim[0].size = pos.size();
    positions.layout.dim[0].stride = pos.size();
    positions.layout.data_offset = 0;
    positions.data.clear();
    positions.data = pos;

    mPub.publish(positions);

    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    imagePub.publish(imageMsg);
}

void BlockDetector::callback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat original_bgr_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    cv::Mat original_hsv_frame, thresholded_hsv_frame,transformed_hsv_frame;
    // Convert from BGR to HSV colorspace
    cv::cvtColor(original_bgr_frame, original_hsv_frame, cv::COLOR_BGR2HSV);
    
    std::vector<cv::Point2f> cornerLocs {cv::Point2f(0.0,0.0),cv::Point2f(0.0,0.0),cv::Point2f(0.0,0.0),cv::Point2f(0.0,0.0)};
    getCyanCornerstone(original_hsv_frame, cornerLocs);
    getMagentaCornerstones(original_hsv_frame, cornerLocs);
    cv::Mat transformed_image = transformPerspective(original_bgr_frame, cornerLocs);
    // Detect the object based on HSV Range Values
    cv::cvtColor(transformed_image, transformed_hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(transformed_hsv_frame, cv::Scalar(0, 0, 0), cv::Scalar(179, 0, 255), thresholded_hsv_frame);
    
    int erosion_size = 3;
    cv::Mat erosion_kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*erosion_size + 1, 2*erosion_size+1));
    cv::erode(thresholded_hsv_frame, thresholded_hsv_frame, erosion_kernel, cv::Point(-1,-1), 2);

    // Floodfill from point (0, 0)
    cv::Mat im_floodfill = thresholded_hsv_frame.clone();
    floodFill(im_floodfill, cv::Point(0,0), cv::Scalar(255));
    // Invert floodfilled image
    cv::Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);

    // Combine the two images to get the foreground.
    cv::Mat im_out = (thresholded_hsv_frame | im_floodfill_inv);
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    //detect edges using canny
    cv::Canny(im_out, canny_output, 50, 150, 3);
    //find contours to be able to get their centres
    cv::findContours(canny_output,contours,hierarchy,
        cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // get the moments
    std::vector<cv::Moments> mu(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        mu[i] = cv::moments(contours[i], false);
    }

    // get the centroid of figures.
    std::vector<cv::Point2f> mc(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        mc[i] = cv::Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
    }
    
    std::vector<cv::Point2f> centroids;
    int numRows = transformed_image.rows;
    for(int i = 0; i < contours.size(); i++) {
        if (i%2 == 0) {
            cv::Point2f treatedCentroid;
            treatedCentroid.x = mc[i].x;
            treatedCentroid.y = numRows - mc[i].y;
            centroids.push_back(treatedCentroid);
            
        }
    }
    if (centroids.size() != 0) {
        std::sort(centroids.begin(), centroids.end(), compareYX);
    }
    // assuming a 500 mm by 500 mm workspace
    float distPerPixelX = 500/transformed_image.cols;
    float distPerPixelY = 500/transformed_image.rows;
    std::vector<float> pos;
    for (int i=0; i<centroids.size(); i++) {
        pos.push_back(centroids[i].x * distPerPixelX);
        pos.push_back(centroids[i].y * distPerPixelY);
    }
    // cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));
    // for(int i = 0; i<contours.size(); i++) {
    //     cv::Scalar color = cv::Scalar(167,151,0); // B G R values
    //     cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
    //     cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );
    // }
    // cv::imshow("contours and centroids", drawing);
    // cv::waitKey(1000);
    publishData(pos, transformed_image);
}

int main( int argc, char** argv ) {
    // Initialise node
    ros::init(argc, argv, "block_detector");
    ros::NodeHandle nodeHandle;
    ros::Rate loopRate(3);
    
    // Let other nodes launch
    ros::Duration(5).sleep();

    // Initialise channel and loop
    BlockDetector bd;
    bd.establishChannels(nodeHandle, "block_positions", "camera/image_raw");
    ros::spin();
}
