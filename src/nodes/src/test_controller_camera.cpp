// ========================================================================== //
// ================================ Libraries =============================== //
// ========================================================================== //

    #include "ros/ros.h"
    #include "std_msgs/String.h"
    #include <iostream>
    #include <string>
    #include <time.h>
    #include <sstream>

/* ========================================================================== */
/* ================================ Constants =============================== */
/* ========================================================================== */

    const std::string PUB_TOPIC {"camera_out"};
    const std::string SUB_TOPIC {"camera_in"};

/* ========================================================================== */
/* ================================ Node Class ============================== */
/* ========================================================================== */

    // Class definition to establish communication with a Camera node
    class Camera {
        public:
            void establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic);
            void publishData(std::string data);
            void callback(const std_msgs::String::ConstPtr& message);
        private:
            ros::Publisher mPub;
            ros::Subscriber mSub;
    };

    // Establishes the channels
    void Camera::establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic) {
        mPub = nodeHandle.advertise<std_msgs::String>(pubTopic, 1000);
        mSub = nodeHandle.subscribe<std_msgs::String>(subTopic, 1000, &Camera::callback, this);
    }
    
    // Publishes a character to the Camera node
    void Camera::publishData(std::string data) {
        std_msgs::String message;
        message.data = data;
        // std::cout << "Camera sent " << data << std::endl;
        mPub.publish(message);
    }
    
    // Waits for the Camera node to return a message
    void Camera::callback(const std_msgs::String::ConstPtr& message) {
        std::string data = message->data.c_str();

        ros::Duration(2).sleep();
        srand (time(NULL));
        int xCoord = rand() % 500;
        ros::Duration(1).sleep();
        srand (time(NULL));
        int yCoord = rand() % 500;
        
        std::stringstream ss;
        ss << xCoord << " " << yCoord;
        data = ss.str();

        publishData(data);
    }

/* ========================================================================== */
/* ============================== Main Function ============================= */
/* ========================================================================== */

    // Main function
    int main(int argc, char **argv) {
        // Initialise node
        ros::init(argc, argv, "test_controller_camera");
        ros::NodeHandle nodeHandle;
        ros::Rate loopRate(5); // Hz
        
        // Initialise channel and loop
        Camera camera;
        camera.establishChannels(nodeHandle, PUB_TOPIC, SUB_TOPIC);
        ros::spin();
    }