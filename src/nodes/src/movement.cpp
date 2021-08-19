// ========================================================================== //
// ================================ Libraries =============================== //
// ========================================================================== //

    #include "ros/ros.h"
    #include "std_srvs/Empty.h"
    #include <sstream>
    #include <fstream>
    #include "std_msgs/String.h"
    #include <iostream>
    #include <string>

    #include <moveit/move_group_interface/move_group_interface.h>
    #include <moveit_msgs/DisplayRobotState.h>
    #include <moveit_msgs/DisplayTrajectory.h>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* ========================================================================== */
/* ================================ Constants =============================== */
/* ========================================================================== */

    // Topic Names
    const std::string PUB_TOPIC {"movement_out"};
    const std::string SUB_TOPIC {"movement_in"};

    // Paths
    constexpr double pi = 3.1415926535;
    static const std::string ROS_NODE_NAME   = "movement";
    static const std::string PLANNING_GROUP  = "manipulator";
    static const std::string ON_CLIENT_PATH  = "/ur5e_epick/epick/on";
    static const std::string OFF_CLIENT_PATH = "/ur5e_epick/epick/off";
    static const std::string NEUTRAL_POSE = "home";

    // Workspace cosntants
    static constexpr double WORKSPACE_ORIGIN_X = 0.3;
    static constexpr double WORKSPACE_ORIGIN_Y = 0.0;
    static constexpr double WORKSPACE_ORIGIN_Z = 0.17; //= 0.15;

    // Model constants
    static constexpr double MODEL_ORIGIN_X = 0.8;
    static constexpr double MODEL_ORIGIN_Y = 0.0;
    static constexpr double MODEL_ORIGIN_Z = 0.775;

    // Robot constants
    static constexpr double ELEVATION = 0.20;

// ========================================================================== //
// ============================= Movement Class ============================= //
// ========================================================================== //

    // Class to control the movement of the robot arm
    class Mover {
        public:
            void move_to_neutral();
            void move_to_pose(double x, double y, double z);
            void move_xy(double x, double y);
            void move_down();
            void move_up();
        private:
            moveit::planning_interface::MoveGroupInterface movement_group = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
            moveit::planning_interface::MoveGroupInterface::Plan movement_plan = moveit::planning_interface::MoveGroupInterface::Plan{};
            const moveit::core::JointModelGroup *joint_model_group = movement_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    };

    // Moves the robot arm to its neutral position
    void Mover::move_to_neutral() {
        movement_group.setNamedTarget(NEUTRAL_POSE);
        movement_group.plan(movement_plan);
        movement_group.move();
    }

    // Move the robot to a specific position via cartesian planenr
    void Mover::move_to_pose(double x, double y, double z) {
        geometry_msgs::Pose target_pose = movement_group.getCurrentPose().pose;
        std::vector<geometry_msgs::Pose> waypoints;

        // Specify pose
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        // Execute movement
        waypoints.push_back(target_pose);
        moveit_msgs::RobotTrajectory trajectory;
        movement_group.computeCartesianPath(waypoints, 0.01, 0, trajectory);
        movement_group.execute(trajectory);
    }

    // Moves to a specific xy position
    void Mover::move_xy(double x, double y) {
        auto position = movement_group.getCurrentPose().pose.position;
        move_to_pose(x+WORKSPACE_ORIGIN_X, y+WORKSPACE_ORIGIN_Y, WORKSPACE_ORIGIN_Z+ELEVATION);
    }

    // Moves the robot down
    void Mover::move_down() {
        auto position = movement_group.getCurrentPose().pose.position;
        move_to_pose(position.x, position.y, WORKSPACE_ORIGIN_Z);
    }

    // Moves the robot up
    void Mover::move_up() {
        auto position = movement_group.getCurrentPose().pose.position;
        move_to_pose(position.x, position.y, WORKSPACE_ORIGIN_Z+ELEVATION);
    }

// ========================================================================== //
// ============================== Gripper Class ============================= //
// ========================================================================== //

    // Class for the gripper
    class Gripper {
        public:
            Gripper(ros::NodeHandle nh);
            void turn_on();
            void turn_off();
        private:
            ros::NodeHandle node_handle;
            ros::ServiceClient on_client;
            ros::ServiceClient off_client;
            std_srvs::Empty srv;
    };
    
    // Constructor for the gripper class
    Gripper::Gripper(ros::NodeHandle nh) {
        on_client = nh.serviceClient<std_srvs::Empty>(ON_CLIENT_PATH);
        off_client = nh.serviceClient<std_srvs::Empty>(OFF_CLIENT_PATH);
        node_handle = nh;
        turn_off(); // turn the gripper off at the start
    }

    // Turns the gripper on
    void Gripper::turn_on() {
        on_client.call(srv);
    }
    
    // Turns the gripper off
    void Gripper::turn_off() {
        off_client.call(srv);
    }

// ========================================================================== //
// ============================== Arranger Class ============================ //
// ========================================================================== //

    // Keeps tracks of the arrangement
    class Arranger {
        public:
            Arranger(Mover *m, Gripper *g);
            void arrange(double x, double y);
        private:
            Mover *mover;
            Gripper *gripper;
            double arrangement_x;
            double arrangement_y;
    };

    // Constructor for the Arranger class
    Arranger::Arranger(Mover *m, Gripper *g) {
        mover = m;
        gripper = g;
        arrangement_x = 0.05;
        arrangement_y = 0.48;
    }

    // Arranges the wooden piece
    void Arranger::arrange(double x, double y) {
        mover->move_to_neutral();
        mover->move_xy(x, y);
        mover->move_down();
        gripper->turn_on();
        mover->move_up();
        mover->move_xy(arrangement_x, arrangement_y);
        mover->move_down();
        gripper->turn_off();
        mover->move_up();
        mover->move_to_neutral();
        arrangement_y -= 0.05;
    }

/* ========================================================================== */
/* ================================ Node Class ============================== */
/* ========================================================================== */

    // Class definition to establish communication with a Movement node
    class Movement {
        public:
            Movement(ros::NodeHandle nodeHandle);
            void establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic);
            void publishData(std::string data);
            void callback(const std_msgs::String::ConstPtr& message);
        private:
            ros::Publisher mPub;
            ros::Subscriber mSub;
            Mover *mover;
            Gripper *gripper;
            Arranger *arranger;
    };

    // Constructor
    Movement::Movement(ros::NodeHandle nodeHandle) {
        ros::AsyncSpinner spinner = ros::AsyncSpinner(1);
        spinner.start();
        mover = new Mover();
        gripper = new Gripper(nodeHandle);
        arranger = new Arranger(mover, gripper);
        mover->move_to_neutral();
        spinner.stop();
    }

    // Establishes the channels
    void Movement::establishChannels(ros::NodeHandle nodeHandle, std::string pubTopic, std::string subTopic) {
        mPub = nodeHandle.advertise<std_msgs::String>(pubTopic, 1000);
        mSub = nodeHandle.subscribe<std_msgs::String>(subTopic, 1000, &Movement::callback, this);
    }
    
    // Publishes a character to the Movement node
    void Movement::publishData(std::string data) {
        std_msgs::String message;
        message.data = data;
        mPub.publish(message);
    }
    
    // Waits for the Movement node to return a message
    void Movement::callback(const std_msgs::String::ConstPtr& message) {
        std::string data = message->data.c_str();
        
        // Continuous spinning to operate robot
        ros::AsyncSpinner spinner = ros::AsyncSpinner(1);
        spinner.start();

        // Extract position values (in mm)
        int x_pos = std::stoi(data.substr(0, data.find(" ")));
        int y_pos = std::stoi(data.substr(data.find(" "), data.length()-data.find(" ")));

        // Error checking
        if(x_pos < 0 && x_pos > 500) {
            spinner.stop();
            publishData("The x position exceeds the defined workspace!");
        } else if(y_pos < 0 && y_pos > 500) {
            spinner.stop();
            publishData("The y position exceeds the defined workspace!");
        }

        // Pick up letter and arrange
        arranger->arrange(x_pos / 1000.0, y_pos / 1000.0);

        // Stop spinner and publish success
        spinner.stop();
        publishData("");
    }

/* ========================================================================== */
/* ============================== Main Function ============================= */
/* ========================================================================== */

    // Main function
    int main(int argc, char **argv) {
        // Initialise node
        ros::init(argc, argv, ROS_NODE_NAME);
        ros::NodeHandle nodeHandle;
        ros::Rate loopRate(5); // Hz

        // Initialise channel and loop
        Movement movement(nodeHandle);
        movement.establishChannels(nodeHandle, PUB_TOPIC, SUB_TOPIC);

        // Run until we CTRL-C
        while(ros::ok())
            ros::spinOnce();
        return 0;
    }