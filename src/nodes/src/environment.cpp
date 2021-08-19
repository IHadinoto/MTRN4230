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
    #include <vector>
    #include <algorithm>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    #include <ctime>

// ========================================================================== //
// ================================ Constants =============================== //
// ========================================================================== //

    // Topics and Ros node name
    const std::string PUB_TOPIC {"environment_out"};
    const std::string SUB_TOPIC {"environment_in"};
    static const std::string ROS_NODE_NAME = "environment";

    // Position of models
    static const std::array<double, 4> CORNER_X = {0.0, 0.0, 0.5, 0.5};
    static const std::array<double, 4> CORNER_Y = {0.0, 0.5, 0.0, 0.5};
    // static const std::array<double, 26> LETTER_X = {0.275, 0.350, 0.425, 0.125, 0.200, 0.275, 0.350, 0.425, 0.125, 0.200, 0.275, 0.350, 0.425, 0.125, 0.200, 0.275, 0.350, 0.425, 0.125, 0.200, 0.275, 0.350, 0.425, 0.125, 0.200, 0.275};
    // static const std::array<double, 26> LETTER_Y = {0.435, 0.420, 0.405, 0.390, 0.375, 0.360, 0.345, 0.330, 0.315, 0.300, 0.285, 0.270, 0.255, 0.240, 0.225, 0.210, 0.195, 0.180, 0.165, 0.150, 0.135, 0.120, 0.105, 0.090, 0.075, 0.060};
    static const std::array<double, 26> LETTER_X = {0.125, 0.200, 0.275, 0.125, 0.200, 0.275, 0.350, 0.425, 0.125, 0.200, 0.275, 0.350, 0.425, 0.125, 0.200, 0.275, 0.350, 0.425, 0.125, 0.200, 0.275, 0.350, 0.425, 0.125, 0.200, 0.275};
    static const std::array<double, 26> LETTER_Y = {0.445, 0.440, 0.435, 0.365, 0.360, 0.355, 0.350, 0.345, 0.285, 0.280, 0.275, 0.270, 0.265, 0.205, 0.200, 0.195, 0.190, 0.185, 0.125, 0.120, 0.115, 0.110, 0.105, 0.045, 0.040, 0.035};

    // Workspace cosntants
    static constexpr double WORKSPACE_ORIGIN_X = 0.3;
    static constexpr double WORKSPACE_ORIGIN_Y = 0.0;
    static constexpr double WORKSPACE_ORIGIN_Z = 0.17; //= 0.15;

    // Model constants
    static constexpr double MODEL_ORIGIN_X = 0.8;
    static constexpr double MODEL_ORIGIN_Y = 0.0;
    static constexpr double MODEL_ORIGIN_Z = 0.775;

    // Names of models
    static const std::string CORNER_1_NAME = "cornerstone 1";
    static const std::string CORNER_2_NAME = "cornerstone 2";
    static const std::string CORNER_3_NAME = "cornerstone 3";
    static const std::string CORNER_4_NAME = "cornerstone 4";
    static const std::string A_NAME = "A";
    static const std::string B_NAME = "B";
    static const std::string C_NAME = "C";
    static const std::string D_NAME = "D";
    static const std::string E_NAME = "E";
    static const std::string F_NAME = "F";
    static const std::string G_NAME = "G";
    static const std::string H_NAME = "H";
    static const std::string I_NAME = "I";
    static const std::string J_NAME = "J";
    static const std::string K_NAME = "K";
    static const std::string L_NAME = "L";
    static const std::string M_NAME = "M";
    static const std::string N_NAME = "N";
    static const std::string O_NAME = "O";
    static const std::string P_NAME = "P";
    static const std::string Q_NAME = "Q";
    static const std::string R_NAME = "R";
    static const std::string S_NAME = "S";
    static const std::string T_NAME = "T";
    static const std::string U_NAME = "U";
    static const std::string V_NAME = "V";
    static const std::string W_NAME = "W";
    static const std::string X_NAME = "X";
    static const std::string Y_NAME = "Y";
    static const std::string Z_NAME = "Z";

    // Name of paths
    static const std::string CORNER_1_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/cylinder/purple_1/cylinder_purple.sdf";
    static const std::string CORNER_2_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/cylinder/purple_2/cylinder_purple.sdf";
    static const std::string CORNER_3_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/cylinder/purple_3/cylinder_purple.sdf";
    static const std::string CORNER_4_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/cylinder/cyan/cylinder_cyan.sdf";
    static const std::string A_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_A/cube_A.sdf";
    static const std::string B_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_B/cube_B.sdf";
    static const std::string C_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_C/cube_C.sdf";
    static const std::string D_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_D/cube_D.sdf";
    static const std::string E_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_E/cube_E.sdf";
    static const std::string F_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_F/cube_F.sdf";
    static const std::string G_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_G/cube_G.sdf";
    static const std::string H_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_H/cube_H.sdf";
    static const std::string I_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_I/cube_I.sdf";
    static const std::string J_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_J/cube_J.sdf";
    static const std::string K_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_K/cube_K.sdf";
    static const std::string L_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_L/cube_L.sdf";
    static const std::string M_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_M/cube_M.sdf";
    static const std::string N_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_N/cube_N.sdf";
    static const std::string O_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_O/cube_O.sdf";
    static const std::string P_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_P/cube_P.sdf";
    static const std::string Q_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_Q/cube_Q.sdf";
    static const std::string R_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_R/cube_R.sdf";
    static const std::string S_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_S/cube_S.sdf";
    static const std::string T_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_T/cube_T.sdf";
    static const std::string U_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_U/cube_U.sdf";
    static const std::string V_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_V/cube_V.sdf";
    static const std::string W_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_W/cube_W.sdf";
    static const std::string X_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_X/cube_X.sdf";
    static const std::string Y_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_Y/cube_Y.sdf";
    static const std::string Z_PATH = "/home/mtrn4230/major-project-mtrn4230_h12a_momsfriendlyrobotcompany/src/lab09_gazebo/models/char_blocks/Char_Z/cube_Z.sdf";

    // Vectors containing names and paths
    const std::vector<std::array<std::string,2>> ALL_LETTERS = {
        {A_NAME, A_PATH}, {B_NAME, B_PATH}, {C_NAME, C_PATH}, {D_NAME, D_PATH}, {E_NAME, E_PATH}, {F_NAME, F_PATH}, {G_NAME, G_PATH}, {H_NAME, H_PATH},
        {I_NAME, I_PATH}, {J_NAME, J_PATH}, {K_NAME, K_PATH}, {L_NAME, L_PATH}, {M_NAME, M_PATH}, {N_NAME, N_PATH}, {O_NAME, O_PATH}, {P_NAME, P_PATH}, 
        {Q_NAME, Q_PATH}, {R_NAME, R_PATH}, {S_NAME, S_PATH}, {T_NAME, T_PATH}, {U_NAME, U_PATH}, {V_NAME, V_PATH}, {W_NAME, W_PATH}, {X_NAME, X_PATH},
        {Y_NAME, Y_PATH}, {Z_NAME, Z_PATH}
    };

// ========================================================================== //
// =============================== Model Class ============================== //
// ========================================================================== //

    // Class for models
    class Model {
        public:
            Model(ros::NodeHandle nh);
            bool spawn_model(std::string model_name, std::string model_path, double x, double y);
            bool delete_model(std::string model_name);
        private:
            ros::NodeHandle node_handle;
    };

    // Constructor for model class
    Model::Model(ros::NodeHandle nh) {
        node_handle = nh;
    }

    // Creates a model (adapted from Max Kelly, from MTRN4230, 21T2, Lab09 Demo)
    bool Model::spawn_model(std::string model_name, std::string model_path, double x, double y) {
        // Convert the position of the model into geometry_msgs
        geometry_msgs::Pose model_pose;
        model_pose.position.x = MODEL_ORIGIN_X + WORKSPACE_ORIGIN_X + x;
        model_pose.position.y = MODEL_ORIGIN_Y + WORKSPACE_ORIGIN_Y + y;
        model_pose.position.z = MODEL_ORIGIN_Z + 0.01;

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
        ifs.open(model_path);
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
// ============================= Helper Functions =========================== //
// ========================================================================== //

    // Spawns everything
    void spawnEverything(Model *model) {
        // Spawn cornerstones
        model->spawn_model(CORNER_1_NAME, CORNER_1_PATH, CORNER_X[0], CORNER_Y[0]);
        model->spawn_model(CORNER_2_NAME, CORNER_2_PATH, CORNER_X[1], CORNER_Y[1]);
        model->spawn_model(CORNER_3_NAME, CORNER_3_PATH, CORNER_X[2], CORNER_Y[2]);
        model->spawn_model(CORNER_4_NAME, CORNER_4_PATH, CORNER_X[3], CORNER_Y[3]);

        // Shuffle letters
        std::srand(std::time (0));
        std::vector<int> letterOrder;
        for(int i = 0; i < (int) ALL_LETTERS.size(); i++)
            letterOrder.push_back(i);
        std::random_shuffle(letterOrder.begin(), letterOrder.end());

        // Spawn letters
        for(int i = 0; i < (int) ALL_LETTERS.size(); i++)
            model->spawn_model(ALL_LETTERS[i][0], ALL_LETTERS[i][1], LETTER_X[letterOrder[i]], LETTER_Y[letterOrder[i]]);
    }

    // Delete everything
    void deleteEverything(Model *model) {
        // Delete corner stones
        model->delete_model(CORNER_1_NAME);
        model->delete_model(CORNER_2_NAME);
        model->delete_model(CORNER_3_NAME);
        model->delete_model(CORNER_4_NAME);

        // Deletes all the letters
        for(int i = 0; i < (int) ALL_LETTERS.size(); i++)
                model->delete_model(ALL_LETTERS[i][0]);
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
            deleteEverything(model);
            spawnEverything(model);
            data = "spawned";
        } else if(data == "delete") {
            deleteEverything(model);
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