#include "vb_greedy_planner/vb_greedy_planner.h"

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
        obs_count_thred_ = 10;
    }
    if (!nh_.getParam("collision_radius",collision_radius_)) {
        collision_radius_ = 0.5;
    }
    if (!nh_.getParam("angle_resolution",angle_resolution_)) {
        angle_resolution_ = 60;
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
    old_open_direction_ = this->CPoint(0,0);
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

Point VB_Planner::OpenDirectionAnalysis() {
    int num_direct = direct_stack_.size();
    Point open_direct = robot_heading_;
    std::vector<float> temp_score_stack;
    float high_score = 0;
    direct_score_stack_.clear();
    direct_score_stack_.reserve(num_direct);
    temp_score_stack.reserve(num_direct);
    for (int i=0; i<num_direct; i++) {
        temp_score_stack[i] = this->RawCast(direct_stack_[i]);
    }
    // average filter
    int filter_coeff = num_direct / 10;
    for (int i=0; i<num_direct; i++) {
        float score = 0;
        int counter = 0;
        for (int j=0; j<filter_coeff; j++) {
            int s = 2;
            if (i==0) s = 1; 
            if (i+j*s < num_direct) {
                score += temp_score_stack[i+j*s];
                counter += 1;
            }
        }
        direct_score_stack_[i] = score / float(counter);
    }

    for (int i=0; i<num_direct; i++) {
        float score = direct_score_stack_[i];
        if (score > high_score && (old_open_direction_ == this->CPoint(0,0) || old_open_direction_ * direct_stack_[i] > 0.8)) {
            high_score = score;
            open_direct = direct_stack_[i];
        }
    }
    open_direction_ = open_direct;
    return open_direct;
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
    float angle_step;
    Point heading_right;
    Point heading_left;
    direct_stack_.clear();
    odom_ = odom_msg;
    rviz_direction_.header = odom_.header;
    robot_pos_.x = odom_.pose.pose.position.x;
    robot_pos_.y = odom_.pose.pose.position.y;
    robot_pos_.z = odom_.pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geo_quat = odom_msg.pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);
    // std::cout<<"Debug Yaw: "<< yaw << std::endl;
    robot_heading_.x = cos(yaw);
    robot_heading_.y = sin(yaw);
    // std::cout<<"Heading X: "<<  robot_heading_.x << "Heading Y: "<<  robot_heading_.y << std::endl;
    direct_stack_.push_back(robot_heading_);
    angle_step = M_PI * 1.2 / float(angle_resolution_); 

    for (int i=1; i<int(angle_resolution_/2); i++) {
        heading_right = this->CPoint(cos(yaw+i*angle_step),sin(yaw+i*angle_step));
        heading_left = this->CPoint(cos(yaw-i*angle_step),sin(yaw-i*angle_step));
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
    old_open_direction_ = this->OpenDirectionAnalysis();
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
