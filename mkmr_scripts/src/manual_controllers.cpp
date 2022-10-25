#include <mkmr_scripts/ManualControllers.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "manual_controllers");
   ros::NodeHandle nh;
   ros::NodeHandle priv_nh("~");
   ManualControllers mc(nh, priv_nh);
   ros::spin();
}
