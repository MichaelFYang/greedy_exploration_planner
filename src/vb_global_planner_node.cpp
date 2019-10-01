// ROS Node of implement visibility_global_planner

#include "visibility_global_planner/visibility_global_planner"


/* ---------------------------------------------------------------------------- */

int main(int argc, char** argv) {
    ros::init(argc, argv, "vb_planner");
    VB_Planner vb_planner;
    vb_planner.Loop();
    return 0;
}




