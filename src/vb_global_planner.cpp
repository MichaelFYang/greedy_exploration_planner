
#include "visibility_global_planner/vb_global_planner.h"

/* ---------------------------------------------------------------------------- */

VB_Planner::VB_Planner() {
    // ROS Parameter Read 
    if (!nh_.getParam("ray_cast_revolution",raw_cast_revolution_)) {
        raw_cast_revolution_ = 20;
    }
    if (!nh_.getParam("max_sensor_range",max_sensor_range_)) {
        max_sensor_range_ = 100.0;
    }
    if (!nh_.getParam("obs_count_thred",obs_count_thred_)) {
        obs_count_thred_ = 50;
    }
}

void VB_Planner::Loop() {
    // Loop Subscriber
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
    if (principal_direction_ * robot_heading_ < 0) {
        principal_direction_.x *= -1;
        principal_direction_.x *= -1;
    }
}

void VB_Planner::ElasticRawCast() {
    // Input: PointCloud; and Principal direction vector
    // Output: update goal waypoint -- the travel distance to an obstacle
    int counter = 0;
    float center_x = odom_.pose.pose.position.x;
    float center_y = odom_.pose.pose.position.y;
    Point center_pos = this->CPoint(center_x, center_y)
    Point check_pos = center_pos;
    while(counter < obs_count_thred_ && this->Norm(check_pos - center_pos) < max_sensor_range_) {
        check_pos += principal_direction_;
        if (this->HitObstacle(check_pos)) {
            counter += 1;
        }
    }
    goal_waypoint_.point.x = check_;point.x;
    goal_waypoint_.point.y = check_;point.y;
}

bool VB_Planner::HitObstacle(Point p) {
    // Credit: Chao C.,
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    pcl::PointXYZI cloud_point;
    kdtree_collision_cloud->radiusSearch(point, collision_check_dist, pointSearchInd, pointSearchSqDis);

    return false;
}

void VB_Planner::OdomHandler(const nav_msgs::Odometry odom_msg) {
    // Credit: CMU SUB_T dfs_behavior_planner, Chao C.,
    // https://bitbucket.org/cmusubt/dfs_behavior_planner/src/master/src/dfs_behavior_planner/dfs_behavior_planner.cpp

    odom_ = odom_msg;
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geo_quat = odometry_msg->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);
    robot_heading_.x = cos(yaw);
    robot_heading_.y = sin(yaw);

}

Point VB_Planner::CPoint(float x, float y) {
    Point p;
    p.x = x;
    p.y = y;
    return p;
}


void VB_Planner::CloudHandler(const sensor_msgs::PointCloud2ConstPtr laser_msg) {
    // Take laser cloud -> update Principal Direction
    laser_cloud_->clear();
    pcl::fromROSMsg(*laser_msg, *laser_cloud_);
    kdtree_collision_cloud_->setInputCloud(laser_cloud_);
    this->PrincipalAnalysis(); // update 
    this->ElasticRawCast(); // update waypoint 
    this->HandleWaypoint(); // Log frame id, etc. -> goal 

}



/* ---------------------------------------------------------------------------- */
