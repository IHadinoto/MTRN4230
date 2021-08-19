// ========================================================================== //
// ================================ Libraries =============================== //
// ========================================================================== //

    #include "ros/ros.h"
    #include "std_srvs/Empty.h"
    #include "gazebo_msgs/SpawnModel.h"
    #include "gazebo_msgs/DeleteModel.h"
    #include <sstream>
    #include <fstream>

    #include <moveit/move_group_interface/move_group_interface.h>
    #include <moveit_msgs/DisplayRobotState.h>
    #include <moveit_msgs/DisplayTrajectory.h>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ========================================================================== //
// ================================ Constants =============================== //
// ========================================================================== //

    // Name constants
    constexpr double pi = 3.1415926535;
    static const std::string ROS_NODE_NAME   = "lab09_demo";
    static const std::string BOX_NAME        = "spawnedBox";
    static const std::string BOX_MODEL_PATH  = "//home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/models/box.sdf";
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
        arrangement_x = 0.01;
        arrangement_y = 0.01;
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
        arrangement_y += 0.05;

    }

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

// ========================================================================== //
// ============================== Main Function ============================= //
// ========================================================================== //

    // Main function
    int main(int argc, char** argv) {
        
        // Set up for ROS
        ros::init(argc, argv, ROS_NODE_NAME);
        ros::NodeHandle nh = ros::NodeHandle{};
        
        // Continuous spinning
        ros::AsyncSpinner spinner = ros::AsyncSpinner(1);
        spinner.start();
        
        // Initialise the gripper, mover, and arranger nodes
        Mover mover;
        mover.move_to_neutral();
        Gripper gripper(nh);
        Arranger arranger(&mover, &gripper);

        // Spawn models
        Model model = Model(nh);
        model.spawn_model(BOX_NAME, 0.2, 0.2, 0.01);

        // Arrange
        arranger.arrange(0.2, 0.2);

        // Remove the model
        mover.move_to_neutral();
        model.delete_model(BOX_NAME);
        
        ros::shutdown();
        return 0;
    }