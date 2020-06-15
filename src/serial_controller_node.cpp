#include <ros/ros.h>
#include <serial/serial.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"serial_controller_node");
    ros::NodeHandle nh;
    auto pub = new ros::Publisher;
    *pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state",1);

    serial::Serial COM;
    COM.setBaudrate(115200);
    COM.setPort("/dev/ttyUSB0");
    COM.open();

    uint8_t* buffer = new uint8_t;
    auto display_msg = new moveit_msgs::DisplayRobotState ;

    display_msg->state.is_diff = 1;
    display_msg->state.joint_state.header.frame_id = "base_link";
    display_msg->state.joint_state.name = {"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

    if (COM.isOpen())
    {
        std::cout << COM.getPort() << " has been connected!" << std::endl;
    }
    int count = 0;
    while(ros::ok())
    {
        if(COM.available())
        {
            int size = COM.available();
            COM.read(buffer,size);
            std::cout << "COUNT " << count << std::endl;
            for (int i = 0; i < size;i=i+8)
            {
                std::cout << *(double*)(buffer+i) << std::endl;
                display_msg->state.joint_state.position.push_back(*(double*)(buffer+i));
            }
            count ++;
            std::cout << display_msg->state.joint_state.position.back() << std::endl;
            pub->publish(*display_msg);
            display_msg->state.joint_state.position.clear();
        }
    }
    COM.close();
    return 0;
}
