#include <ros/ros.h>
#include <encoder_reading_msg/joint_positions.h>


int main(int argc, char **argv) {
   
    ros::init(argc, argv, "publisher");

    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<encoder_reading_msg::joint_positions>("joint_positions", 1000);
    ros::Rate loop_rate(1);

    encoder_reading_msg::joint_positions msg;
    msg.text.data = "Joint position:";
    msg.joint_positions.data.resize(6);
    msg.joint_positions.data[0] = 1;
    msg.joint_positions.data[1] = 0;
    msg.joint_positions.data[2] = 0.5;
    msg.joint_positions.data[3] = 0.3;
    msg.joint_positions.data[4] = 1;
    msg.joint_positions.data[5] = 1.5;

    while (ros::ok()) {
        std::cout << msg.text.data << std::endl;
        std::cout << "1: " << msg.joint_positions.data[0] << std::endl;
        std::cout << "2: " << msg.joint_positions.data[1] << std::endl;
        std::cout << "3: " << msg.joint_positions.data[2] << std::endl;
        std::cout << "4: " << msg.joint_positions.data[3] << std::endl;
        std::cout << "5: " << msg.joint_positions.data[4] << std::endl;
        std::cout << "6: " << msg.joint_positions.data[5] << std::endl;

        publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}
