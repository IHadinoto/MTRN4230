/* ========================================================================== */
/* ================================ Libraries =============================== */
/* ========================================================================== */

    #include "ros/ros.h"
    #include "std_msgs/String.h"
    #include <iostream>
    #include <string>

/* ========================================================================== */
/* ================================ Constants =============================== */
/* ========================================================================== */

    const std::string PUB_TOPIC {"user_out"};
    const std::string SUB_TOPIC {"user_in"};

/* ========================================================================== */
/* =============================== UserIO Class ============================= */
/* ========================================================================== */

    // Class definition to establish communication with a UserIO node
    class UserIO {
        public:
            void establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic);
            void getInput();
            void setOutput(const std_msgs::String::ConstPtr& message);
        private:
            ros::Publisher mPub;
            ros::Subscriber mSub;
    };

    // Establishes the channels
    void UserIO::establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic) {
        mPub = nodeHandle.advertise<std_msgs::String>(pubTopic, 1000);
        mSub = nodeHandle.subscribe<std_msgs::String>(subTopic, 1000, &UserIO::setOutput, this);
    }
    
    // Gets input from terminal and publishes
    void UserIO::getInput() {
        std::string data {""};
        getline(std::cin, data);
        std_msgs::String message;
        message.data = data;
        mPub.publish(message);
    }
    
    // Outputs a message to the terminal
    void UserIO::setOutput(const std_msgs::String::ConstPtr& message) {
        std::string data = message->data.c_str();
        if(data == "getInput") getInput(); // a request to get input
        else std::cout << data; // without newline
    }

/* ========================================================================== */
/* ============================== Main Function ============================= */
/* ========================================================================== */

    // Main function
    int main(int argc, char **argv) {
        // Initialise node
        ros::init(argc, argv, "user_IO");
        ros::NodeHandle nodeHandle;
        ros::Rate loopRate(5); // Hz

        // Initialise channel and loop
        UserIO userIO;
        userIO.establishChannels(nodeHandle, PUB_TOPIC, SUB_TOPIC);
        ros::spin();
    }