#include <ros/ros.h>
#include <kinematics_srv_msg/forward_kinematics_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>

bool fkSolution(kinematics_srv_msg::forward_kinematics_msgRequest & req, kinematics_srv_msg::forward_kinematics_msgResponse & res);

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "fk_service_server");
    ros::NodeHandle nh;
    
    /* 
    * Viene istanziato un oggetto di tipo ServiceServer che riceve in ingresso il 
    * nome del servizio e la callback di gestione delle richieste.
    */
    ros::ServiceServer server = nh.advertiseService("forward_kinematics", fkSolution);

    ROS_INFO("Server di cinematica diretta avviato");

    ros::spin();

    ros::shutdown();
    return 0;
}

bool fkSolution(kinematics_srv_msg::forward_kinematics_msgRequest & req, kinematics_srv_msg::forward_kinematics_msgResponse & res) {
    
    /**
    * Viene istanziato un oggetto del tipo RobotModelLoader che cercherà la descrizione del robot sul server dei parametri ROS. 
    * Quando si lancia il file demo.launch di un pacchetto robot_name_moveit_config nel parameter server il parametro robot_description 
    * conterrà l’urdf del robot, ovvero la sua descrizione.
    */
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

    /**
    * Grazie al RobotModelLoader è possibile ottenere il modello cinematico del robot rappresentato dall'oggetto RobotModel.
    */
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    /**
    * A partire dal modello cinematico è possibile creare un oggetto RobotState che rappresenta lo stato del robot
    * e tiene conto di posizioni, velocità, accelerazione, etc.
    *
    * Il RobotState così definito avrà valori di default.
    */
    moveit::core::RobotState robot_state(kinematic_model);

    /**
    * Viene aggiornato il RobotState a partire da quello della richiesta.
    */
    moveit::core::robotStateMsgToRobotState(req.robot_state, robot_state);

    /**
    * Si aggiornano poi le trasformazioni del frame di riferimento per i link. 
    * Questa chiamata è necessaria prima di utilizzare le trasformazioni dei collegamenti per le trasformazioni delle coordinate.
    */
    robot_state.updateLinkTransforms();

    /**
    * ---------------------------------------------------------------------------------------------------------------------------
    * Si legge dal parameter server il planning group e si crea un oggetto JointModelGroup che rappresenta il modello del robot per 
    * il particolare gruppo.
    */
    ros::NodeHandle nh;
    std::string planning_group_name;
    
    if(!nh.getParam("planning_group_name", planning_group_name)) {
        ROS_ERROR("'planning_group_name' non è definito sul parameter server!");
        return false;
    }

    const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);
    // ---------------------------------------------------------------------------------------------------------------------------

    /**
    * Si leggono i link appartenenti al gruppo.
    */
    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();


    /**
    * Si calcola la cinematica diretta rispetto all'ultimo link del gruppo (end-effector).
    */
    Eigen::Isometry3d forward_kinematics = robot_state.getGlobalLinkTransform(link_names.back());

    /**
    * Si setta la risposta con i valori appena calcolati.
    */
    tf::poseEigenToMsg(forward_kinematics, res.end_effector_pose);

    return true;
}