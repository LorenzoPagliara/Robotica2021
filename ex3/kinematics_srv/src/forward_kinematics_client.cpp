#include <ros/ros.h>
#include <kinematics_srv_msg/forward_kinematics_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/GetPositionFK.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "fk_service_client");
    ros::NodeHandle nh;
    
    /* 
    * Viene istanziato un oggetto di tipo ServiceClient parametrizzato sul tipo del servizio, che
    * riceve in ingresso il nome del servizio.
    */
    ros::ServiceClient client = nh.serviceClient<kinematics_srv_msg::forward_kinematics_msg>("forward_kinematics");

    /**
    * Viene istanziato un oggetto del tipo del servizio.
    */ 

    kinematics_srv_msg::forward_kinematics_msg fk_service;

    /**
    * Viene istanziato un oggetto del tipo RobotModelLoader che cercherà la descrizione del robot sul server dei parametri ROS. 
    * Quando si lancia il file demo.launch di un pacchetto robot_name_moveit_config nel parameter server il parametro robot_description 
    * conterrà l’urdf del robot, ovvero la sua descrizione.
    */
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));
    
    /**
    * Grazie al RobotModelLoader è possibile ottenere il modello cinematico del robot rappresentato dall'oggetto RobotModel.
    */
    robot_model::RobotModelConstPtr robot_model = robot_model_loader->getModel();

    /**
    * A partire dal modello cinematico è possibile creare un oggetto RobotState che rappresenta lo stato del robot
    * e tiene conto di posizioni, velocità, accelerazione, etc.
    *
    * Il RobotState così definito avrà valori di default.
    */
    moveit::core::RobotState robot_state(robot_model);

    /**
    * --------------------------------------------------------------------------------------------------
    * A questo punto si sceglie un particolare planning group del robot, nel caso specifico
    * viene recuperato dal parameter server, e il client setta uno stato per i giunti appartenenti
    * a tale gruppo. 
    */
    std::string planning_group_name;
    
    if(!nh.getParam("planning_group_name", planning_group_name)) {
        ROS_ERROR("'planning_group_name' non è definito sul parameter server!");
        return false;
    }

    const std::vector<double> joint_positions = {0, 0.5, 0.001, 0.001, -1, -4};

    /**
    * L'oggetto JointModelGroup rappresenta il modello del robot per il particolare gruppo.
    */
    const robot_state::JointModelGroup * joint_model_group = robot_model->getJointModelGroup(planning_group_name);

    robot_state.setJointGroupPositions(joint_model_group, joint_positions);
    // --------------------------------------------------------------------------------------------------

    /**
    * Viene settata la richiesta del client con lo stato del robot settato dallo stesso.
    */
    moveit::core::robotStateToRobotStateMsg(robot_state, fk_service.request.robot_state);

    /**
    * Viene effettuata la chiamata al servizio.
    */
    if(!client.call(fk_service))
        ROS_ERROR("Could not compute forward kinematics");

    /**
    * --------------------------------------------------------------------------------------------------
    * Il client inoltre si occupa di gestire la risposta del server e di stamparla a video.
    */
    tf2::Quaternion quaternion;
    tf2::fromMsg(fk_service.response.end_effector_pose.orientation, quaternion);
    
    tf2::Matrix3x3 matrix(quaternion);
    tf2Scalar roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    std::cout << "Cinematica diretta con solver personalizzato:"  << std::endl;
    std::cout << "Posizione (XYZ): ["; 
    std::cout << fk_service.response.end_effector_pose.position.x << ", ";
    std::cout << fk_service.response.end_effector_pose.position.y << ", ";
    std::cout << fk_service.response.end_effector_pose.position.z << "]" << std::endl;
    std::cout << "Orientamento (RPY): [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;;

    // --------------------------------------------------------------------------------------------------

    
    /** 
    * --------------------------------------------------------------------------------------------------
    * In questo pezzo di codice si ripete il procedimento appena visto utilizzando il servizio
    * /compute_fk per il controllo dei risultati.
    */
    client = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");

    /**
    * Salvataggio dei link del planning group.
    */
    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    /**
    * Settaggio dei campi della richiesta.
    */
    moveit_msgs::GetPositionFK move_group;
    move_group.request.header.frame_id = link_names[0];
    move_group.request.fk_link_names.push_back(link_names.back());
    moveit::core::robotStateToRobotStateMsg(robot_state, move_group.request.robot_state);

    if(!client.call(move_group))
        ROS_ERROR("Could not compute forward kinematics");

    tf2::fromMsg(move_group.response.pose_stamped[0].pose.orientation, quaternion);
    
    tf2::Matrix3x3 rotation_matrix(quaternion);
    rotation_matrix.getRPY(roll, pitch, yaw);

    std::cout << "Cinematica diretta con move_group solver:"  << std::endl;
    std::cout << "Posizione (XYZ): ["; 
    std::cout << move_group.response.pose_stamped[0].pose.position.x << ", ";
    std::cout << move_group.response.pose_stamped[0].pose.position.y << ", ";
    std::cout << move_group.response.pose_stamped[0].pose.position.z << "]" << std::endl;
    std::cout << "Orientamento (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";

    // --------------------------------------------------------------------------------------------------

    ros::shutdown();
    return 0;
}
