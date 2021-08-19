    #include "ros/ros.h"
    #include "std_msgs/String.h"
    #include <iostream>
    #include <string>
    #include <stdlib.h>

/* ========================================================================== */
/* ================================ Constants =============================== */
/* ========================================================================== */

    const std::string USER_PUB_TOPIC {"user_in"};
    const std::string USER_SUB_TOPIC {"user_out"};
    const std::string MOVEMENT_PUB_TOPIC {"movement_in"};
    const std::string MOVEMENT_SUB_TOPIC {"movement_out"};
    const std::string CAMERA_PUB_TOPIC {"camera_in"};
    const std::string CAMERA_SUB_TOPIC {"camera_out"};
    const std::string ENVIRONMENT_PUB_TOPIC {"environment_in"};
    const std::string ENVIRONMENT_SUB_TOPIC {"environment_out"};
    const std::string CHARACTERS {"ABCDEFGHIJKLMNOPQRSTUVWXYZ"};

/* ========================================================================== */
/* ======================== Class Forward Declarations ====================== */
/* ========================================================================== */

    // Class definition for controller
    class Node; // forward declaration
    class Controller {
        public:
            Controller() : mSpawned {false}, mFinished {false}, mDefinedString {""}, mCurrentPosition {0} {};
            void setNodes(Node* userIO, Node* camera, Node* movement, Node* environment);
            void initiate();
            void printSuccess(std::string message);
            void printWaiting(std::string message);
            void printFailure(std::string message);
            void step(std::string nodeName, std::string data);
        private:
            bool mSpawned, mFinished;
            std::string mDefinedString;
            int mCurrentPosition;
            Node *mUserIO, *mCamera, *mMovement, *mEnvironment;
    };

    // Class definition to esatblish communication with a node
    class Node {
        public:
            Node(Controller* controller, std::string name) :
                mController {controller}, mName {name} {};
            void establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic);
            void publishData(std::string data);
            void callback(const std_msgs::String::ConstPtr& message);
        private:
            Controller *mController;
            std::string mName;
            ros::Publisher mPub;
            ros::Subscriber mSub;
    };

/* ========================================================================== */
/* ============================= Controller Class =========================== */
/* ========================================================================== */

    // Gets the camera and movement nodes
    void Controller::setNodes(Node* userIO, Node* camera, Node* movement, Node* environment) {
        mUserIO = userIO;
        mCamera = camera;
        mMovement = movement;
        mEnvironment = environment;
    }

    // Read user-defined string
    void Controller::initiate() {
        printWaiting("Spawning in the environment...\n");
        mEnvironment->publishData("spawn");
    }

    // Prints the waiting message to the console
    void Controller::printWaiting(std::string message) {
        mUserIO->publishData("[WAITING] " + message);
    }

    // Prints the success message to the console
    void Controller::printSuccess(std::string message) {
        mUserIO->publishData("[SUCCESS] " + message);
    }

    // Prints the failure message to the console
    void Controller::printFailure(std::string message) {
        mUserIO->publishData("[FAILURE] " + message);
        if(mSpawned) {
            printWaiting("System is shutting down...\n");
            mEnvironment->publishData("delete");
        } else {
            printSuccess("System has shut down!\n\n");
            exit(0);
        }
    }

    // Iterates through the user-defined string
    void Controller::step(std::string nodeName, std::string data) {
        std::string character {mDefinedString[mCurrentPosition]};

        // 1. Spawns the environment
        if(nodeName == "environment" && data == "spawned" && !mSpawned) {
            printSuccess("Spawned in the environment!\n");
            printWaiting("Enter an alphabetical string with 1-10 unique uppercase letters: ");
            mSpawned = true;
            mUserIO->publishData("getInput");
            return;
        }

        // 2. Gets input from IO
        if(nodeName == "userIO" && mDefinedString == "" && mSpawned) {
            mDefinedString = data;

            // 2.1. Makes sure that the string is of an appropriate size
            if(mDefinedString.length() == 0) {
                printFailure("User-defined string is empty!\n");
                mFinished = true;
                return;
            } else if(mDefinedString.length() > 10) {
                printFailure("User-defined string is more than 10 characters!\n");
                mFinished = true;
                return;
            }

            // 2.2. Checks if defined string is alphabetical
            for(int i = 0; i < mDefinedString.length(); i++) {
                bool found {false};
                for(int j = 0; !found && j < CHARACTERS.length(); j++)
                    if(mDefinedString[i] == CHARACTERS[j])
                        found = true;
                if(!found) {
                    printFailure("User-defined string is not completely uppercase and/or alphabetical!\n");
                    mFinished = true;
                    return;
                }
            }

            // 2.3. Checks if defined string has unique characters
            std::string uniqueCharacters {""};
            for(int i = 0; i < mDefinedString.length(); i++)
                if((int) uniqueCharacters.find(mDefinedString[i]) == -1)
                    uniqueCharacters.push_back(mDefinedString[i]);
                else {
                    printFailure("User-defined string does not have unique characters!\n");
                    mFinished = true;
                    return;
                }
            
            // 2.4. If all is well, send character to camera node
            printSuccess("User has entered '"+mDefinedString+"'!\n");
            printWaiting("Camera node is processing the environment...\n");
            character = mDefinedString[mCurrentPosition];
            mCamera->publishData(character);
            return;
        }

        // 3. If received coordinates from the camera
        if(nodeName == "camera" && mCurrentPosition < mDefinedString.length()) {
            
            // 3.1. The camera is unsuccessful
            if(data == "") {
                printFailure("Camera node could not detect '"+character+"'!\n");
                mFinished = true;
                return;
            }
            
            // 3.2. The camera is successful, and send coordinates to movement
            printSuccess("Camera node has detected '"+character+"' at " + data + "!\n");
            printWaiting("Movement node is moving the robot arm...\n");
            mMovement->publishData(data);
            return;
        }

        // 4. Movement has finished moving, and wants another character
        if(nodeName == "movement" && mCurrentPosition < mDefinedString.length()) {
            
            // 4.1. If the movement node was unsuccessful
            if(data != "") {
                printFailure(data+"\n");
                mFinished = true;
                return;
            }

            // 4.2. If the movement node was successful
            printSuccess("Movement node has moved '"+character+"'!\n");
            character = mDefinedString[++mCurrentPosition];
            if(mCurrentPosition < mDefinedString.length())
                printWaiting("Camera node is processing the environment...\n");
            mCamera->publishData(character);
            return;
        }

        // 5. Has deleted the environment
        if(nodeName == "environment" && data != "spawned" && mSpawned) {
            
            // 5.1. If the environment has been deleted successfully
            if(data == "deleted") {
                printSuccess("Deleted environment!\n");
                printSuccess("System has shut down!\n\n");
                mSpawned = false;
                exit(0);
                return;
            }

            // 5.2. If the environment could not be deleted
            if(data == "failed") {
                printFailure("Could not delete environment!\n");
                return;
            }
        }

        // 6. If the string is empty (finished)
        if(mCurrentPosition >= mDefinedString.length() && mSpawned && !mFinished)  {
            printSuccess("System has finished arranging the user-defined, alphanbetical string!\n");
            printWaiting("System is shutting down...\n");
            mFinished = true;
            mEnvironment->publishData("delete");
            return;
        }
    }

/* ========================================================================== */
/* ================================ Node Class ============================== */
/* ========================================================================== */

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
        mController->step(mName, data);
    }

/* ========================================================================== */
/* ============================== Main Function ============================= */
/* ========================================================================== */

    // Main function
    int main(int argc, char **argv) {
        // Initialise controller
        ros::init(argc, argv, "controller");
        ros::NodeHandle nodeHandle;
        ros::Rate loopRate(5); // Hz
        
        // Initialise channel classes
        Controller controller;
        Node userIO(&controller, "userIO");
        Node camera(&controller, "camera");
        Node movement(&controller, "movement");
        Node environment(&controller, "environment");
        controller.setNodes(&userIO, &camera, &movement, &environment);
        userIO.establishChannels(nodeHandle, USER_PUB_TOPIC, USER_SUB_TOPIC);
        camera.establishChannels(nodeHandle, CAMERA_PUB_TOPIC, CAMERA_SUB_TOPIC);
        movement.establishChannels(nodeHandle, MOVEMENT_PUB_TOPIC, MOVEMENT_SUB_TOPIC);
        environment.establishChannels(nodeHandle, ENVIRONMENT_PUB_TOPIC, ENVIRONMENT_SUB_TOPIC);

        // Starts the first step and loops the node
        ros::Duration(3).sleep(); // let other nodes launch
        for(int i = 0; i < 25; i++) std::cout << "\n" << std::endl; // custom clear
        ros::Duration(1).sleep(); // pause after clear
        controller.printSuccess("System has turned on!\n");
        controller.initiate();
        ros::spin();
    }