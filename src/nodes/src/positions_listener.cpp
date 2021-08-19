#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <iostream>
#include <algorithm>
#include <iterator>
#include <sstream>

// Accessing the position values being sent
void callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::vector<float> rawMessage = msg->data;
    std::vector<std::vector<float>> positions;
    int numVals = msg->layout.dim[0].stride;
    std::vector<float> currentPos;
    std::stringstream ss;
    for (int i=0; i < numVals; i++) {
        if (i%2 == 0) {
            currentPos.push_back(rawMessage[i]);
            ss << rawMessage[i] << " ";
        } else if (i%2 == 1) {
            currentPos.push_back(rawMessage[i]);
            positions.push_back(currentPos);
            std::vector<float> currentPos;
            ss << rawMessage[i] << " ";
        }

    }
    // // Tried to print out nicely but this process cant keep up with the rate at which values get published so I print everything
    // // out as 1 big string
    // for (std::vector<int> pos : positions) {
    //     std::cout << "x: " << pos[0] << ", " << "y: " << pos[1] << std::endl;
    // }
    std::cout << ss.str() << std::endl;
}

// subscribing to the topic block_positions
int main(int argc, char **argv) {
    ros::init(argc, argv, "positions_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("block_positions", 1000, callback);
    ros::spin();
}