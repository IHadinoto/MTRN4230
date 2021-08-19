// ========================================================================== //
// ================================ Libraries =============================== //
// ========================================================================== //

    #include "ros/ros.h"
    #include "std_msgs/String.h"
    #include <iostream>
    #include <string>

/* ========================================================================== */
/* ================================ Constants =============================== */
/* ========================================================================== */

    const std::string PUB_TOPIC {"user_in"};
    const std::string SUB_TOPIC {"user_out"};

/* ========================================================================== */
/* ================================ Node Class ============================== */
/* ========================================================================== */

    // Class definition to esatblish communication with a node
    class Node {
        public:
            void establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic);
            void publishData(std::string data);
            void callback(const std_msgs::String::ConstPtr& message);
        private:
            ros::Publisher mPub;
            ros::Subscriber mSub;
    };

    // Establishes the channels
    void Node::establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic) {
        mPub = nodeHandle.advertise<std_msgs::String>(pubTopic, 1000);
        mSub = nodeHandle.subscribe<std_msgs::String>(subTopic, 1000, &Node::callback, this);
    }

    // Publishes a character to the node
    void Node::publishData(std::string data) {
        std_msgs::String message;
        message.data = data;
        mPub.publish(message);
    }

    // Waits for the node to return a message
    void Node::callback(const std_msgs::String::ConstPtr& message) {
        std::string data = message->data.c_str();
        ros::Duration(1).sleep();
        std::cout << "[DEMO_IO] " << data << std::endl << "" << std::endl;
    }

/* ========================================================================== */
/* ============================== Main Function ============================= */
/* ========================================================================== */

    // Main function
    int main(int argc, char **argv) {
        // Initialise node
        ros::init(argc, argv, "demo_IO");
        ros::NodeHandle nodeHandle;
        ros::Rate loopRate(5); // Hz
        
        for(int i = 0; i < 25; i++) std::cout << "\n" << std::endl;
        ros::Duration(1).sleep();

        // Initialise channel and loop
        Node node;
        node.establishChannels(nodeHandle, PUB_TOPIC, SUB_TOPIC);

        // Test input output
        std::cout << "\n\n[DEMO_IO] Tell IO node to output 'Timothy'" << std::endl;
        ros::Duration(1).sleep();
        node.publishData("[IO_NODE] Timothy\n");
        ros::Duration(1).sleep();
        std::cout << "[DEMO_IO] Take user's input from the console and print it out" << std::endl;
        ros::Duration(1).sleep();
        node.publishData("[IO_NODE] ");
        ros::Duration(1).sleep();
        node.publishData("getInput");

        ros::spin();
    }