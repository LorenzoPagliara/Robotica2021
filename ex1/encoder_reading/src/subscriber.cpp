#include <ros/ros.h>
#include <encoder_reading_msg/joint_positions.h>

void messageReceived(const encoder_reading_msg::joint_positions & msg){
    std::cout << "Subscriber reading " << msg.text.data << std::endl;
    std::cout << "1: " << msg.joint_positions.data[0] << std::endl;
    std::cout << "2: " << msg.joint_positions.data[1] << std::endl;
    std::cout << "3: " << msg.joint_positions.data[2] << std::endl;
    std::cout << "4: " << msg.joint_positions.data[3] << std::endl;
    std::cout << "5: " << msg.joint_positions.data[4] << std::endl;
    std::cout << "6: " << msg.joint_positions.data[5] << std::endl;
}

int main(int argc, char **argv) {
   
    ros::init(argc, argv, "subscriber");

    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("joint_positions", 1000, messageReceived);

    ros::spin();
    ros::shutdown();
    return 0;

}