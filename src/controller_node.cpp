#include "stanley_pkg/controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh("~");

    stanley::Stanley Baslat(nh);

    ros::spin();
    return 0;
}