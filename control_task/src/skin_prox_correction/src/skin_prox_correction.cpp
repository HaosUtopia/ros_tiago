#include <skin_prox_correction_class.h>

int main(int argc, char** argv)
{

   ros::init(argc, argv, "skin_prox_correction");
   ros::NodeHandle node;

   ros::AsyncSpinner spinner(2);
   spinner.start();

   SkinProxCorrection ProxCorrection(node);

   while(ros::ok())
   {
     if(ProxCorrection.flag == false)
     {
       ProxCorrection.flag = ProxCorrection.Position_correction();
       ros::Duration(0.5).sleep();
       ROS_INFO("!!!");
     }
   }

   ros::waitForShutdown();
   return 0;
}
