#include <carmaker_autorunner/carmaker_autorunner.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "carmaker_autorunner_node");
    ros::NodeHandle nh;

    CarMakerAutorunner carmaker_autorunner(nh);
    carmaker_autorunner.Run();

    return 0;
}