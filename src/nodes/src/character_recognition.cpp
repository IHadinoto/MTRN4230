#include <stdio.h>
#include <string>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/opencv.hpp>

#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include <iostream>
#include <algorithm>
#include <iterator>
#include <tuple>
#include <sstream>

using namespace std;
using namespace cv;

class CharacterRecognition {
    public:
        void establishChannels(ros::NodeHandle nodeHandle, string pubTopic, string subTopicImg, string subTopicPos, string subTopicChar);
        void publishData(int CharacterLocation);
        void publishString(string msgString);
        void position_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        void character_callback(const std_msgs::String::ConstPtr& msg);

        vector<vector<float>> positions;
        char character;
        vector<char> CharacterArray;
        Mat im;
    private:
        ros::Publisher mPub;
        // 3 Callbacks for each subscriber
        ros::Subscriber mSubPos;
        image_transport::Subscriber mSubImg;
        ros::Subscriber mSubChar;
        bool hasRead = false;
};

// Establishes the channels
void CharacterRecognition::establishChannels(ros::NodeHandle nodeHandle, string pubTopic, string subTopicPos, string subTopicImg, string subTopicChar) {
    mPub = nodeHandle.advertise<std_msgs::String>(pubTopic, 1000);
    image_transport::ImageTransport it(nodeHandle);
    // 3 Channels for each subscriber
    mSubPos = nodeHandle.subscribe(subTopicPos, 1, &CharacterRecognition::position_callback, this);
    mSubImg = it.subscribe(subTopicImg, 1, &CharacterRecognition::image_callback, this);
    mSubChar = nodeHandle.subscribe(subTopicChar, 1, &CharacterRecognition::character_callback, this);
}

// Subscribing to Position
void CharacterRecognition::position_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (!hasRead) {
        vector<float> rawMessage = msg->data;
        int numVals = msg->layout.dim[0].stride;
        if(numVals == 0) {
            return;
        }
        // Obtaining Position
        vector<float> currentPos;
        // Sent in 1D array x1,y1,x2,y2,...
        positions.clear();
        for (int i=0; i < numVals; i++) {
            if (i%2 == 0) {
                currentPos.push_back(rawMessage[i]);
            } else if (i%2 == 1) {
                currentPos.push_back(rawMessage[i]);
                positions.push_back(currentPos);
                currentPos.clear();
            }
        }
    }
}

// Subscribing to Image
void CharacterRecognition::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    if (!hasRead) {
        im = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if(im.empty()) {
            return;
        }

        // Converting to Gray
        cvtColor(im,im,COLOR_BGR2GRAY);
        im = im > 50; // Lower for more white
    
        //imshow("Image",im);
        //waitKey(0);

        // Dilating image
        //GaussianBlur(im, im, cv::Size(3, 3), 0);
        erode(im, im, getStructuringElement(MORPH_RECT, Size(5, 5)));
        dilate(im, im, getStructuringElement(MORPH_RECT, Size(3, 3)));

        dilate(im, im, getStructuringElement(MORPH_RECT, Size(3, 3)));
        erode(im, im, getStructuringElement(MORPH_RECT, Size(3, 3)));

        erode(im, im, getStructuringElement(MORPH_RECT, Size(3, 3)));
        dilate(im, im, getStructuringElement(MORPH_RECT, Size(3, 3)));

        dilate(im, im, getStructuringElement(MORPH_RECT, Size(3, 3)));
        erode(im, im, getStructuringElement(MORPH_RECT, Size(3, 3)));

        //imshow("Image",im);
        //waitKey(1000);

        // Create Tesseract object
        tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();

        // Initialize OCR engine to use English (eng) and The LSTM OCR engine.
        ocr->Init("/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/camera_package/src/tessdata", "eng", tesseract::OEM_TESSERACT_ONLY);
        // Set Page segmentation mode to PSM_AUTO (3)
        ocr->SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
        // Set image data
        ocr->SetImage(im.data, im.cols, im.rows, 1, im.step);
        // Set resolution to prevent warnings
        ocr->SetSourceResolution(70);
        // Set variable data
        ocr->SetVariable("tessedit_char_whitelist", "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
        // Run Tesseract OCR on image
        string outText;
        outText = string(ocr->GetUTF8Text());

        // print recognized text
        //cout << "READ TEXT" << endl << outText << endl;

        // Copy to CharacterArray
        int i = 0;
        int TextLength = int(outText.size());
        CharacterArray.clear();
        while (i<TextLength) {
            if (outText[i] != ' ' && outText[i] != '\n' ) {
                CharacterArray.push_back(outText[i]);
                //cout << outText[i];
            }
            i++;
        }
        //cout << endl;
        
        // Destroy used object and release memory
        ocr->End();
    }
}

// Subscribing to Character
void CharacterRecognition::character_callback(const std_msgs::String::ConstPtr& msg){
    std::string data = msg->data.c_str();
    character = data[0];
    int CharacterLocation = -1;
    // Obtaining Character
    // Iterate through CharacterArray and look for character
    for (int i=0;  i<CharacterArray.size(); i++) {
        // Look for InputCharacter
        //cout << character << " " << CharacterArray[i] << endl;
        if (character == CharacterArray[i]) {
            CharacterLocation = i;
            break;
        }
    }

    // Display location of found character
    if (CharacterLocation != -1) {
        publishData(CharacterLocation);
        return;
    } else {
        publishString("");
        return;
    }
}

// Publishing Coordinates
void CharacterRecognition::publishData(int CharacterLocation) {
    vector<float> coordinates = positions[CharacterLocation];
    stringstream ss;
    ss << int(coordinates[0]) << " " << int(coordinates[1]);
    std_msgs::String msg;
    msg.data = ss.str();
    hasRead = true;
    mPub.publish(msg);
}

// Publishes message
void CharacterRecognition::publishString(string msgString) {
    std_msgs::String message;
    message.data = msgString;
    mPub.publish(message);
}

int main(int argc, char* argv[]) {
    // subscribing to the topic block_positions
    ros::init(argc, argv, "character_recognition");
    ros::NodeHandle n;

    // Command line argument for comparison
    //tesseract Blocks5.jpeg stdout --tessdata-dir tessdata -l eng --oem 0 --psm 6

    CharacterRecognition CR;
    CR.establishChannels(n, "camera_out", "block_positions", "camera_image", "camera_in"); //Change requested_char

    // move all of the actual processing code into the while loop below (keep the stuff that saves things to variables inside the callback just so they do something)
    ros::spin();
    return EXIT_SUCCESS;
}

/*UDWNZRGAXPYOCJIHLFEQMVSTKIB
302.521 21.0701
141.504 22.4936
389.542 48.0557
220.568 54.0307
477.08 103.074
302.604 104.167
141.303 104.401
389.536 141.986
220.517 147.371
141.785 196.648
302.526 197.575
477.105 198.052
220.559 239.04
389.6 242.107
141.68 287.319
302.552 290.049
477.076 295.689
220.631 339.16
389.444 349.242
302.462 369.746
477.004 378.071
220.649 429.138
397.667 439.667
141.504 465.027
302.531 465.033*/
