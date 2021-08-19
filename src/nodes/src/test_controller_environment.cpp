// ========================================================================== //
// ================================ Libraries =============================== //
// ========================================================================== //

    #include "ros/ros.h"
    #include "std_msgs/String.h"
    #include <iostream>
    #include <string>
    #include <stdlib.h>
    #include "std_srvs/Empty.h"
    #include "gazebo_msgs/SpawnModel.h"
    #include "gazebo_msgs/DeleteModel.h"
    #include <sstream>
    #include <fstream>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ========================================================================== //
// ================================ Constants =============================== //
// ========================================================================== //

    // Topics and Ros node name
    const std::string PUB_TOPIC {"environment_out"};
    const std::string SUB_TOPIC {"environment_in"};
    static const std::string ROS_NODE_NAME = "test_controller_environment";

    // Position of models
    static const std::array<double, 4> CORNER_X {0.0, 0.0, 0.5, 0.5};
    static const std::array<double, 4> CORNER_Y {0.0, 0.5, 0.0, 0.5};
    static const std::array<double, 26> LETTER_X {0.0};
    static const std::array<double, 26> LETTER_Y {0.47};

        // Workspace cosntants
    static constexpr double WORKSPACE_ORIGIN_X = 0.3;
    static constexpr double WORKSPACE_ORIGIN_Y = 0.0;
    static constexpr double WORKSPACE_ORIGIN_Z = 0.17; //= 0.15;

    // Model constants
    static constexpr double MODEL_ORIGIN_X = 0.8;
    static constexpr double MODEL_ORIGIN_Y = 0.0;
    static constexpr double MODEL_ORIGIN_Z = 0.775;

    // Names and paths of models
    static const std::string BOX_NAME        = "spawnedBox";
    static const std::string BOX_MODEL_PATH  = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/gazebo/models/box/box.sdf";

// ========================================================================== //
// =============================== Model Class ============================== //
// ========================================================================== //

    // Class for models
    class Model {
        public:
            Model(ros::NodeHandle nh);
            bool spawn_model(std::string model_name, double x, double y, double z);
            bool delete_model(std::string model_name);
        private:
            ros::NodeHandle node_handle;
    };

    // Constructor for model class
    Model::Model(ros::NodeHandle nh) {
        node_handle = nh;
    }

    // Creates a model (adapted from Max Kelly, from MTRN4230, 21T2, Lab09 Demo)
    bool Model::spawn_model(std::string model_name, double x, double y, double z) {
        // Convert the position of the model into geometry_msgs
        geometry_msgs::Pose model_pose;
        model_pose.position.x = MODEL_ORIGIN_X + WORKSPACE_ORIGIN_X + x;
        model_pose.position.y = MODEL_ORIGIN_Y + WORKSPACE_ORIGIN_Y + y;
        model_pose.position.z = MODEL_ORIGIN_Z + z;

        // Set the orientation of the model (0,0,0)
        tf2::Quaternion quaternion;
        quaternion.setRPY(0,0,0);
        model_pose.orientation = tf2::toMsg(quaternion);

        // Spawns the model
        ros::ServiceClient spawnModel = node_handle.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        spawnModel.waitForExistence();
        gazebo_msgs::SpawnModel srv;
        srv.request.model_name = model_name;
        std::ifstream ifs;
        ifs.open(BOX_MODEL_PATH);
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        srv.request.model_xml = buffer.str();
        srv.request.initial_pose = model_pose;
        srv.request.robot_namespace = "/";
        srv.request.reference_frame = "world";
        spawnModel.call(srv);
        return srv.response.success;
    }

    // Removes a model from the Gazebo environment
    bool Model::delete_model(std::string model_name) {
        ros::ServiceClient deleteModel = node_handle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
        deleteModel.waitForExistence();
        gazebo_msgs::DeleteModel srv;
        srv.request.model_name = model_name;
        deleteModel.call(srv);
        return srv.response.success;
    }

/* ========================================================================== */
/* ================================ Node Class ============================== */
/* ========================================================================== */

    // Class definition to establish communication with a Environment node
    class Environment {
        public:
            Environment(ros::NodeHandle nodeHandle);
            void establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic);
            void publishData(std::string data);
            void callback(const std_msgs::String::ConstPtr& message);
        private:
            ros::Publisher mPub;
            ros::Subscriber mSub;
            Model *model;
    };

    // Constructor for the Environment class
    Environment::Environment(ros::NodeHandle nodeHandle) {
        model = new Model(nodeHandle);
    }

    // Establishes the channels
    void Environment::establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic) {
        mPub = nodeHandle.advertise<std_msgs::String>(pubTopic, 1000);
        mSub = nodeHandle.subscribe<std_msgs::String>(subTopic, 1000, &Environment::callback, this);
    }
    
    // Publishes a character to the Camera node
    void Environment::publishData(std::string data) {
        std_msgs::String message;
        message.data = data;
        mPub.publish(message);
    }
    
    // Waits for the Environment node to return a message
    void Environment::callback(const std_msgs::String::ConstPtr& message) {
        std::string data = message->data.c_str();

        if(data == "spawn") {
            ros::Duration(1).sleep();
            data = "spawned";
        } else if(data == "delete") {
            ros::Duration(1).sleep();
            data = "deleted";
        } else {
            data = "failed";
        }

        publishData(data);
    }

// ========================================================================== //
// ============================== Main Function ============================= //
// ========================================================================== //

    // Main function
    int main(int argc, char** argv) {
        ros::init(argc, argv, ROS_NODE_NAME);
        ros::NodeHandle nodeHandle;
        ros::Rate loopRate(5); // Hz
        
        // Initialise channel and loop
        Environment environment(nodeHandle);
        environment.establishChannels(nodeHandle, PUB_TOPIC, SUB_TOPIC);
        ros::spin();
        return 0;
    }