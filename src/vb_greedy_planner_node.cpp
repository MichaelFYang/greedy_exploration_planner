// ROS Node of implement visibility_greedy_planner

#include "vb_greedy_planner/vb_greedy_planner.h"


/* ---------------------------------------------------------------------------- */

int main(int argc, char** argv) {
    ros::init(argc, argv, "vb_planner");
    VB_Planner vb_planner;
    vb_planner.Loop();
    return 0;
}




