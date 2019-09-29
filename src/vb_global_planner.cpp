
#include "visibility_global_planner/vb_global_planner.h"

/* ---------------------------------------------------------------------------- */

VB_Planner::VB_Planner() {}

void VB_Planner::Loop() {
    goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("_",1);
    point_cloud_sub_ = nh_.subscribe("_",1,&VB_Planner::CloudHandler,this);
    odom_sub_ = nh_.subscribe("_",1,&VB_Planner::OdomHandler,this);
    ros::spin();
}

void VB_Planner::PrincipalAnalysis() {
    //  Output: a 2D vector with the domain direction -- Normlize to Norm 1
    // Using PCA -> eigen vector with maximum eigen value
    Eigen::Matrix3f eigen_vectors;
    pcl::PCA<pcl::PointXYZI> pca(*laser_cloud_);
    eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f vector;
    vector(0) = eigen_vectors(0,0);
    vector(1) = eigen_vectors(0,1);
    vector(2) = eigen_vectors(0,2);
    float norm;
    // TODO !!!
    


}

void VB_Planner::ElasticRawCast() {
    // Input: PointCloud; and Principal direction vector
    // Output: update goal waypoint -- the travel distance to an obstacle

}


void VB_Planner::CloudHandler(const sensor_msgs::PointCloud2ConstPtr laser_msg) {
    // Take laser cloud -> update Principal Direction
    laser_cloud_->clear();
    pcl::fromROSMsg(*laser_msg, *laser_cloud_);
    this->PrincipalAnalysis(); // update 
    this->ElasticRawCast(); // update waypoint 
    this->HandleWaypoint(); // Log frame id, etc. -> goal 
}

