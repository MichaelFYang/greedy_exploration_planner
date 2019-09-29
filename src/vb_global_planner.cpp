
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
    float norm = sqrt(vector(0)*vector(0) + vector(1)*vector(1));
    principal_direction_.x = vector(0) / norm;
    principal_direction_.y = vector(1) / norm;
    // TODO !!!
    
}

void VB_Planner::ElasticRawCast() {
    // Input: PointCloud; and Principal direction vector
    // Output: update goal waypoint -- the travel distance to an obstacle
}

void VB_Planner::OdomHandler(const nav_msgs::Odometry odom_msg) {
    odom_ = odom_msg;
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geo_quat = odometry_msg->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);
    robot_heading.x = cos(yaw);
    robot_heading.y = sin(yaw);


}


void VB_Planner::CloudHandler(const sensor_msgs::PointCloud2ConstPtr laser_msg) {
    // Take laser cloud -> update Principal Direction
    laser_cloud_->clear();
    pcl::fromROSMsg(*laser_msg, *laser_cloud_);
    this->PrincipalAnalysis(); // update 
    this->ElasticRawCast(); // update waypoint 
    this->HandleWaypoint(); // Log frame id, etc. -> goal 
}

