#include <condition_variable>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_action_msg/inverse_kinematics_msgAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const kinematics_action_msg::inverse_kinematics_msgResultConstPtr & result);
void handleGoalActiveEvent();
void handleFeedbackEvent(const kinematics_action_msg::inverse_kinematics_msgFeedbackConstPtr & feedback);
void serializeIKSolution(std::ostringstream & str, const moveit_msgs::RobotState & robot_state);

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "kinematics_action_client");

    /**
    * Viene istanziato un oggetto di tipo SimpleActionClient parametrizzato sul tipo dell'azione, che
    * riceve in ingrestro il nome del server a cui connettersi e un'opzione booleana per far attivare un thread.
    */
    actionlib::SimpleActionClient<kinematics_action_msg::inverse_kinematics_msgAction> client("inverse_kinematics_server", true);

    /**
    * Attesa del server
    */
    client.waitForServer();
    
    /**
    * ----------------------------------------------------------------------------------------------
    * Viene istanziato un oggetto del tipo dell'azione e vengono settati i campi del goal.
    */
    kinematics_action_msg::inverse_kinematics_msgGoal goal;

    goal.end_effector_pose.position.x = 1.0;
    goal.end_effector_pose.position.y = 0.5;
    goal.end_effector_pose.position.z = 1.0;
    
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);

    goal.end_effector_pose.orientation = tf2::toMsg(quaternion);
    // ----------------------------------------------------------------------------------------------
    
    /**
    * Invio del goal al server.La funzione riceve in ingrestro il goal, una callback per il completamento,
    * una per il goal attivo e una per la gestione dei feedback.
    */
    client.sendGoal(goal, &handleGoalCompletionEvent, &handleGoalActiveEvent, &handleFeedbackEvent);

    /**
    * Il client ora attende il completamento dell'obiettivo prima di continuare.
    */
    if(!client.waitForResult(ros::Duration(30.0)))
        ROS_ERROR("L'inversione cinematica non è stata calcolata nel tempo allocato");    

    ros::shutdown();
    return 0;
}

void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const kinematics_action_msg::inverse_kinematics_msgResultConstPtr & result) {
    
    std::ostringstream str;

    if(state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {

        int solutions_num = result->ik_solutions.size();

        str << "Obiettivo raggiunto! " << state.getText() << std::endl;

        /**
        * Per ognuna delle soluzioni trovate vengono serializzati i risultati.
        */
        for(int i = 0; i < solutions_num; i++) {
            serializeIKSolution(str, result->ik_solutions[i]);
            str << std::endl;
        }

        ROS_INFO_STREAM(str.str());
        
        ros::NodeHandle nh;



        ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));
        robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

        
        std::string planning_group_name;
    
        if(!nh.getParam("planning_group_name", planning_group_name)) {
            ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        }

        const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);

        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name = joint_model_group->getVariableNames();

        ROS_INFO("Publishing solutions...");

        ros::Duration sleep_time(2.0);

        for(int i=0; i < solutions_num; i++) {
            sleep_time.sleep();

            joint_state_msg.position = result->ik_solutions[i].joint_state.position;
            joint_state_msg.header.stamp = ros::Time::now();

            joint_state_publisher.publish(joint_state_msg);
        }       

        ROS_INFO("All solutions published");           
    }
    else {
        str << "Obiettivo fallito!" << state.getText() << std::endl;
        ROS_INFO_STREAM(str.str());
    }    
}

void handleGoalActiveEvent() {
    ROS_INFO("Richiesta di inversione cinematica inviata al server.");
}

void handleFeedbackEvent(const kinematics_action_msg::inverse_kinematics_msgFeedbackConstPtr & feedback) {
    
    std::ostringstream str;

    str << "Ricevito IK feedback: ";

    serializeIKSolution(str, feedback->ik_feedback);

    ROS_INFO_STREAM(str.str());
}

void serializeIKSolution(std::ostringstream & str, const moveit_msgs::RobotState & robot_state) {
    
    int joints_num = robot_state.joint_state.position.size();

    str << "[";

    for(int i = 0; i < joints_num; i++) {
        str << robot_state.joint_state.position[i];

        if(i != joints_num - 1)
            str << ", ";
    }

    str << "]" << std::endl;
}