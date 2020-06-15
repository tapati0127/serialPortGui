#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <string>
#include <std_msgs/Float64MultiArray.h>
serial::Serial COM;

void connect (const std_msgs::String::ConstPtr &portname)
{
    if(portname->data.back()=='1')
    {
        std::string a = portname->data;
        a.pop_back();
        COM.setPort(a);
        COM.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        COM.setTimeout(to);
        COM.open();
        if (!COM.isOpen()) ROS_ERROR("Error");
        else ROS_INFO("CONNECTED!");
    }
    else 
    {
        COM.close();
        if (COM.isOpen()) ROS_ERROR("Error");
        else ROS_INFO("CLOSED!");
    }
    
}
void plan (const std_msgs::Float64MultiArray::ConstPtr &data)
{
    auto buffer = new uint8_t[12*8];
    auto a = new double[12];
    for(int i = 0; i < 12; i++)
    {
        *(a+i) = data->data.at(i);
    }
    memcpy(buffer,a,96);
    for(int i = 0; i < 96; i=i+8)
    {
        std::cout << *(double*)(buffer+i) <<std::endl;
    }
    
    if(COM.isOpen())
    {
        try{
            std::cout << COM.write(buffer,96) << std::endl;
            std::cout << "Planning" << std::endl;
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR("ERROR!");
        }
        
    }
    delete[]  buffer;
    delete[] a;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"serial_controller_node_1");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("portname",1,connect);
    ros::Subscriber sub1 = nh.subscribe("set_robot_state",1,plan);
    uint8_t* buffer = new uint8_t[48];
    auto display_msg = new moveit_msgs::DisplayRobotState ;
    auto pub = new ros::Publisher;
    *pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state",1);
    display_msg->state.is_diff = 1;
    display_msg->state.joint_state.header.frame_id = "base_link";
    display_msg->state.joint_state.name = {"joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"};

    while(ros::ok())
    {
        if(COM.available())
        {
            int size = COM.available();
            COM.read(buffer,size);

            for (int i = 0; i < size;i=i+8)
            {
                display_msg->state.joint_state.position.push_back(*(double*)(buffer+i));
            }

            //std::cout << display_msg->state.joint_state.position.back() << std::endl;
            pub->publish(*display_msg);
            display_msg->state.joint_state.position.clear();
        }
        ros::spinOnce();
    }

}
