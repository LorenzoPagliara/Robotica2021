#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_dp_redundancy_resolution/workspace_trajectory.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "trajectory_planner");
    ros::NodeHandle nh;

    /**
    * Lettura del planning group dal parameter server.
    */    
    std::string planning_group_name;
    if(!nh.getParam("planning_group_name", planning_group_name)) {
        ROS_ERROR("Non è stato possibile ottenere il 'planning_group_name' dal parameter server");
    }

    /** ---------------------------------------- SETUP ---------------------------------------- **/

    /**
    * Classe client per usare la ROS interface fornita dal move_group node.
    */
    moveit::planning_interface::MoveGroupInterface move_group(planning_group_name);
    
    /** ---------------------------------------- VISUALIZATION ---------------------------------------- **/
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Fanuc Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();
  
    /** ---------------------------------------- PLANNING ---------------------------------------- **/
    visual_tools.prompt("Premere 'next' nel RvizVisualToolsGui per iniziare la demo");

    /**
    * Lettura del path del file .traj.
    */
    std::string package_path = ros::package::getPath("trajectory_planner");
    std::string trajectory_file_path = package_path + "/data/circular_yz.traj";

    /**
    * Lettura del file .traj.
    */
    moveit_dp_redundancy_resolution::WorkspaceTrajectory ws_trajectory("circular_yz", trajectory_file_path);
    moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory ws_trajectory_msg;
    ws_trajectory.getWorkspaceTrajectoryMsg(ws_trajectory_msg);

    /**
    * Creazione di un oggetto RobotTrajectory.
    */
    moveit_msgs::RobotTrajectory trajectory;

    /**
    * Computazione del path cartesiano. La parametrizzazione temporale è fatta in automatico da computeCartesianPath.
    */
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(ws_trajectory_msg.waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("fanuc20ia demo", "Visualizzazione path: (%.2f%% raggiunto)", fraction * 100.0);

    /**
    * Visualizzazione del path in RVIZ.
    */
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XXLARGE);
    visual_tools.publishPath(ws_trajectory_msg.waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < ws_trajectory_msg.waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(ws_trajectory_msg.waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);
    visual_tools.trigger();

    /** ---------------------------------------- RQT_MULTIPLOT ---------------------------------------- **/

    ros::Publisher trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("plot_planned_trajectory", 10000, true);

    ros::Duration sleep_time(0.05);

    for(int i=0; i < trajectory.joint_trajectory.points.size(); i++) {
        trajectory_msgs::JointTrajectoryPoint jtp = trajectory.joint_trajectory.points[i];
        trajectory_publisher.publish(jtp);

        sleep_time.sleep();
    }

    visual_tools.prompt("Premere 'next' nel RvizVisualToolsGui per terminare il planner");

    ros::shutdown();
    exit(0);
}
