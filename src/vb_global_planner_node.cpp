// ROS Node of implement visibility_global_planner

#include "vb_global_planner/vb_global_planner.h"


/* ---------------------------------------------------------------------------- */

int main(int argc, char** argv) {
    ros::init(argc, argv, "vb_planner");
    VB_Planner vb_planner;
    vb_planner.Loop();
    return 0;
}




