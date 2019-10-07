
#include "vb_global_planner/vb_global_planner.h"

/* ---------------------------------------------------------------------------- */

VB_Planner::VB_Planner() {
    // ROS Parameter Read 
    if (!nh_.getParam("ray_cast_revolution",raw_cast_resolution_)) {
        raw_cast_resolution_ = 100;
    }
    if (!nh_.getParam("max_sensor_range",max_sensor_range_)) {
        max_sensor_range_ = 15.0;
    }
    if (!nh_.getParam("obs_count_thred",obs_count_thred_)) {
        obs_count_thred_ = 5;
    }
    if (!nh_.getParam("collision_radius",collision_radius_)) {
        collision_radius_ = 0.5;
    }
    if (!nh_.getParam("robot_frame_id",robot_frame_id_)) {
        robot_frame_id_ = "X1/base_link";
    }
    if (!nh_.getParam("angle_resolution",angle_resolution_)) {
        angle_resolution_ = M_PI / 18.0;
    }
    // get topic name parameter
    if (!nh_.getParam("vb_goal_topic",goal_topic_)) {
        goal_topic_ = "/way_point";
    }
    if (!nh_.getParam("vb_laser_topic",laser_topic_)) {
        laser_topic_ = "/velodyne_cloud_registered";
    }
    if (!nh_.getParam("vb_odom_topic",odom_topic_)) {
        odom_topic_ = "/integrated_to_map";
    }
    // initial cloud
    laser_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_filtered_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    kdtree_collision_cloud_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    // initial old principal direciton
    old_principla_direction_ = this->CPoint(0,0);
    std::cout<<"Initialize Successfully"<<std::endl;
}

Point VB_Planner::CPoint(float x, float y) {
    Point p;
    p.x = x;
    p.y = y;
    return p;
}

void VB_Planner::Loop() {
    // Loop Subscriber
    rviz_direct_pub_ = nh_.advertise<nav_msgs::Path>("/vb_planner/PCA_direction",1);
    goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>(goal_topic_,1);
    point_cloud_sub_ = nh_.subscribe(laser_topic_,1,&VB_Planner::CloudHandler,this);

    odom_sub_ = nh_.subscribe(odom_topic_,1,&VB_Planner::OdomHandler,this);

    ros::Rate rate(1);
    while(ros::ok())
    {
        ros::spinOnce();
        //process
        rate.sleep();
    }

    //ros::spin();
}

// Point VB_Planner::PrincipalAnalysis() {
//     //  Output: a 2D vector with the domain direction -- Normlize to Norm 1
//     // Using PCA -> eigen vector with maximum eigen value
//     Eigen::Matrix3f eigen_vectors;
//     pcl::PCA<pcl::PointXYZI> cpca;
//     cpca.setInputCloud(laser_cloud_);
//     eigen_vectors = cpca.getEigenVectors();
//     Eigen::Vector3f vector;

//     vector(0) = eigen_vectors(0,0);
//     vector(1) = eigen_vectors(0,1);
//     vector(2) = eigen_vectors(0,2);
//     float norm = sqrt(vector(0)*vector(0) + vector(1)*vector(1));
//     principal_direction_.x = vector(0) / norm;
//     principal_direction_.y = vector(1) / norm;
//     if (old_principla_direction_ == this->CPoint(0,0) && principal_direction_ * robot_heading_ < 0) {
//         principal_direction_.x *= -1;
//         principal_direction_.y *= -1;
//     }else if (principal_direction_ * old_principla_direction_ < 0) {
//         principal_direction_.x *= -1;
//         principal_direction_.y *= -1;
//     }
//     second_direction_left_ = this->CPoint(-principal_direction_.y,principal_direction_.x);
//     second_direction_right_ = this->CPoint(principal_direction_.y,-principal_direction_.x);

//     return principal_direction_;
// }

void VB_Planner::OpenDirectionAnalysis() {
    int num_direct = direct_stack_.size();
    Point open_direct = robot_heading_;
    float high_score = 0;
    direct_score_stack_.reserve(num_direct);
    for (int i=0; i<num_direct; i++) {
        direct_score_stack_[i] = this->RawCast(direct_stack_[i]);
    }
    for (int i=0; i<num_direct; i++) {
        float score = direct_score_stack_[i];
        if (score > high_score) {
            high_score = score;
            open_direct = direct_stack_[i];
        }
    }
    open_direction_ = open_direct;
}

void VB_Planner::ElasticRawCast() {
    // TODO -> Right now is the simple version of ray casting
    int counter = 0;
    Point center_pos = this->CPoint(robot_pos_.x, robot_pos_.y);
    Point check_pos_principal = center_pos;
    // principal direction
    while(counter < obs_count_thred_ && this->Norm(check_pos_principal - center_pos) < max_sensor_range_) {
        check_pos_principal.x += open_direction_.x / raw_cast_resolution_;
        check_pos_principal.y += open_direction_.y / raw_cast_resolution_;

        if (this->HitObstacle(check_pos_principal)) {
            counter += 1;
        }
    }
    goal_waypoint_.point.x = check_pos_principal.x;
    goal_waypoint_.point.y = check_pos_principal.y;
    goal_waypoint_.point.z = robot_pos_.z;

}

float VB_Planner::RawCast(Point direction) {
    // Input: PointCloud; and Principal direction vector
    // Output: update goal waypoint -- the travel distance to an obstacle
    int counter = 0;
    float center_x = robot_pos_.x;
    float center_y = robot_pos_.y;
    Point center_pos = this->CPoint(center_x, center_y);
    Point check_pos_principal = center_pos;
    // principal direction
    while(counter < obs_count_thred_ && this->Norm(check_pos_principal - center_pos) < max_sensor_range_) {
        check_pos_principal.x += direction.x / raw_cast_resolution_;
        check_pos_principal.y += direction.y / raw_cast_resolution_;

        if (this->HitObstacle(check_pos_principal)) {
            counter += 1;
        }
    }
    return this->Norm(check_pos_principal - center_pos);
}

bool VB_Planner::HitObstacle(Point p) {
    // Credit: Chao C.,
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    pcl::PointXYZI cloud_point;
    cloud_point.x = p.x;
    cloud_point.y = p.y;
    cloud_point.z = robot_pos_.z;
    kdtree_collision_cloud_->radiusSearch(cloud_point, collision_radius_, pointSearchInd, pointSearchSqDis);
    if (!pointSearchInd.empty()) {
        return true;
    }
    return false;
}

void VB_Planner::OdomHandler(const nav_msgs::Odometry odom_msg) {
    // Credit: CMU SUB_T dfs_behavior_planner, Chao C.,
    // https://bitbucket.org/cmusubt/dfs_behavior_planner/src/master/src/dfs_behavior_planner/dfs_behavior_planner.cpp
    int num_direction;
    Point heading_right;
    Point heading_left;
    odom_ = odom_msg;
    rviz_direction_.header = odom_.header;
    robot_pos_.x = odom_.pose.pose.position.x;
    robot_pos_.y = odom_.pose.pose.position.y;
    robot_pos_.z = odom_.pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geo_quat = odom_msg.pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);
    robot_heading_.x = cos(yaw);
    robot_heading_.y = sin(yaw);
    direct_stack_.push_back(robot_heading_);
    num_direction = int(std::ceil (M_PI / angle_resolution_));
    for (int i=1; i<num_direction%2; i++) {
        heading_right = this->CPoint(cos(yaw+i*angle_resolution_),sin(yaw+i*angle_resolution_));
        heading_left = this->CPoint(cos(yaw-i*angle_resolution_),sin(yaw-i*angle_resolution_));
        direct_stack_.push_back(heading_right);
        direct_stack_.push_back(heading_left);
    }
}

void VB_Planner::HandleWaypoint() {
    // publish rviz
    geometry_msgs::PoseStamped pose;
    pose.header = rviz_direction_.header;
    pose.pose.position.x = robot_pos_.x;
    pose.pose.position.y = robot_pos_.y;
    pose.pose.position.z = robot_pos_.z;
    rviz_direction_.poses.clear();
    rviz_direction_.poses.push_back(pose);
    pose.pose.position.x = robot_pos_.x + max_sensor_range_ * open_direction_.x;
    pose.pose.position.y = robot_pos_.y + max_sensor_range_ * open_direction_.y;
    // pose.pose.position.x = goal_waypoint_.point.x;
    // pose.pose.position.y = goal_waypoint_.point.y;
    pose.pose.position.z = robot_pos_.z;
    rviz_direction_.poses.push_back(pose);
    // publish waypoint;
    goal_waypoint_.header = odom_.header;
    goal_pub_.publish(goal_waypoint_);
    rviz_direct_pub_.publish(rviz_direction_);
    // std::cout << "Goal Publihsed ..." << std::endl;
}

void VB_Planner::CloudHandler(const sensor_msgs::PointCloud2ConstPtr laser_msg) {
    // Take laser cloud -> update Principal Direction
    laser_cloud_->clear();
    pcl::fromROSMsg(*laser_msg, *laser_cloud_);
    this->LaserCloudFilter();
    kdtree_collision_cloud_->setInputCloud(laser_cloud_filtered_);
    // old_principla_direction_ = this->PrincipalAnalysis(); // update 
    this->OpenDirectionAnalysis();
    this->ElasticRawCast(); // update waypoint 
    this->HandleWaypoint(); // Log frame id, etc. -> goal 

}

void VB_Planner::LaserCloudFilter() {
    // Source credit: http://pointclouds.org/documentation/tutorials/passthrough.php
    pcl::PassThrough<pcl::PointXYZI> cloud_filter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_temp->clear();
    std::size_t laser_cloud_size = laser_cloud_->points.size();
    pcl::PointXYZI point;
    for (std::size_t i=0; i<laser_cloud_size; i++) {
        point = laser_cloud_->points[i];
        misc_utils_ns::LeftRotatePoint(point);
        // point.z = robot_pos_.z;
        laser_cloud_temp->points.push_back(point);
    }
    laser_cloud_ = laser_cloud_temp;

    cloud_filter.setInputCloud (laser_cloud_);
    cloud_filter.setFilterFieldName ("z");
    cloud_filter.setFilterLimits (robot_pos_.z-2*collision_radius_, robot_pos_.z+2*collision_radius_);
    //pass.setFilterLimitsNegative (true);
    cloud_filter.filter(*laser_cloud_filtered_);
}

float VB_Planner::Norm(Point p) {
    return sqrt(p.x*p.x + p.y*p.y);
}

/* ---------------------------------------------------------------------------- */
