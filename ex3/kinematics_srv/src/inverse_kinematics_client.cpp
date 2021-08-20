#include <ros/ros.h>
#include <kinematics_srv_msg/inverse_kinematics_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinematics_service_client");
    ros::NodeHandle nh;
    
    /* 
    * Viene istanziato un oggetto di tipo ServiceClient parametrizzato sul tipo del servizio, che
    * riceve in ingresso il nome del servizio.
    */
    ros::ServiceClient client = nh.serviceClient<kinematics_srv_msg::inverse_kinematics_msg>("inverse_kinematics");

    /**
    * ----------------------------------------------------------------------------------------------
    * Viene istanziato un oggetto del tipo del servizio e vengono settati i campi della richiesta.
    */
    kinematics_srv_msg::inverse_kinematics_msg ik_service;

    ik_service.request.end_effector_pose.position.x = 1;
    ik_service.request.end_effector_pose.position.y = 0;
    ik_service.request.end_effector_pose.position.z = 1;
    
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);

    ik_service.request.end_effector_pose.orientation = tf2::toMsg(quaternion);
    // ----------------------------------------------------------------------------------------------

    /**
    * Viene effettuata la chiamata al servizio.
    */
    if(!client.call(ik_service))
        ROS_ERROR("Non è stato possibile ottenere una soluzione.");

    
    std::cout << "Soluzione di inversione cinematica: [";

    /**
    * ----------------------------------------------------------------------------------------------
    * Viene gestita e stampata a video la risposta del server.
    */
    int n_joints = ik_service.response.robot_state.joint_state.position.size();

    for(int i = 0; i < n_joints; i++) {
        std::cout << ik_service.response.robot_state.joint_state.position[i];

        if(i != n_joints - 1)
            std::cout << ", ";
    }

    std::cout << "]" << std::endl;
    // ----------------------------------------------------------------------------------------------

    ros::shutdown();
    return 0;
}