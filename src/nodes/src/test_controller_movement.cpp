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

    const std::string PUB_TOPIC {"movement_out"};
    const std::string SUB_TOPIC {"movement_in"};

/* ========================================================================== */
/* ================================ Node Class ============================== */
/* ========================================================================== */

    // Class definition to establish communication with a Movement node
    class Movement {
        public:
            void establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic);
            void publishData(std::string data);
            void callback(const std_msgs::String::ConstPtr& message);
        private:
            ros::Publisher mPub;
            ros::Subscriber mSub;
    };

    // Establishes the channels
    void Movement::establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic) {
        mPub = nodeHandle.advertise<std_msgs::String>(pubTopic, 1000);
        mSub = nodeHandle.subscribe<std_msgs::String>(subTopic, 1000, &Movement::callback, this);
    }
    
    // Publishes a character to the Movement node
    void Movement::publishData(std::string data) {
        std_msgs::String message;
        message.data = data;
        // std::cout << "Movement sent " << data << std::endl;
        mPub.publish(message);
    }
    
    // Waits for the Movement node to return a message
    void Movement::callback(const std_msgs::String::ConstPtr& message) {
        std::string data = message->data.c_str();
        // std::cout << "Movement received " << data << std::endl;

        // DO STUFF START
        ros::Duration(3).sleep();
        data = ""; // if successful
        // data = "Cannot reach" // if unsuccessful
        // DO STUFF FINISH

        publishData(data);
    }

/* ========================================================================== */
/* ============================== Main Function ============================= */
/* ========================================================================== */

    // Main function
    int main(int argc, char **argv) {
        // Initialise node
        ros::init(argc, argv, "test_controller_movement");
        ros::NodeHandle nodeHandle;
        ros::Rate loopRate(5); // Hz
        
        // Initialise channel and loop
        Movement movement;
        movement.establishChannels(nodeHandle, PUB_TOPIC, SUB_TOPIC);
        ros::spin();
    }