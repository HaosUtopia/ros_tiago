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
     ProxCorrection.Position_correction();
   }

   ros::waitForShutdown();
   return 0;
}
