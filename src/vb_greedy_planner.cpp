#include "vb_greedy_planner/vb_greedy_planner.h"

/* ---------------------------------------------------------------------------- */

VB_Planner::VB_Planner() {
    // ROS Parameter Read 
    if (!nh_.getParam("ray_cast_resolution",ray_cast_resolution_)) {
        ray_cast_resolution_ = 100;
    }
    if (!nh_.getParam("max_sensor_range",max_sensor_range_)) {
        max_sensor_range_ = 15.0;
    }
    if (!nh_.getParam("planner_type",planner_type_)) {
        planner_type_ = 0;
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
    if (!nh_.getParam("dead_end_filter", dead_end_filter_)) {
        dead_end_filter_ = 50;
    }
    if (!nh_.getParam("dead_end_thred", dead_end_thred_)) {
        dead_end_thred_ = 0.2;
    }
    // get topic name parameter
    if (!nh_.getParam("vb_goal_topic", goal_topic_)) {
        goal_topic_ = "/uav1/custom_waypoint";
    }
    if (!nh_.getParam("vb_laser_topic",laser_topic_)) {
        laser_topic_ = "/velodyne_cloud_registered";
    }
    if (!nh_.getParam("vb_odom_topic",odom_topic_)) {
        odom_topic_ = "/integrated_to_map";
    }
    if (!nh_.getParam("vb_frontier_topic",frontier_topic_)) {
        frontier_topic_ = "/frontier_cloud";
    }
   
    // initial cloud
    laser_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_filtered_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    kdtree_collision_cloud_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

	frontier_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_frontier_filtered_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    kdtree_frontier_cloud_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    // initial old principal direciton
    old_open_direction_ = this->CPoint(0,0);
    std::cout<<"Initialize Successfully"<<std::endl;
    max_score_stack_.clear();
    dead_end_ = false;
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
    goal_pub_ = nh_.advertise<ROSWayPoint>(goal_topic_,1);
    point_cloud_sub_ = nh_.subscribe(laser_topic_,1,&VB_Planner::CloudHandler,this);
	frontier_cloud_sub_ = nh_.subscribe(frontier_topic_,1,&VB_Planner::FrontierCloudHandler,this);
    odom_sub_ = nh_.subscribe(odom_topic_,1,&VB_Planner::OdomHandler,this);

    ros::Rate rate(1);
    while(ros::ok())
    {
        ros::spinOnce(); // process all callback function
        //process
        if (!laser_cloud_filtered_->empty() || !laser_frontier_filtered_->empty()) {
            this->UpdaterayCastingStack();
            old_open_direction_ = this->OpenDirectionAnalysis();
            this->ElasticRayCast(); // update waypoint 
            this->HandleWaypoint(); // Log frame id, etc. -> goal 
        
        }
        rate.sleep();
    }
}

Point VB_Planner::OpenDirectionAnalysis() {
    int num_direct = direct_stack_.size();
    Point open_direct = robot_heading_;
    float high_score = 0;
    switch (planner_type_)
    {
    case 0:
        this->VisibilityScoreAssign(direct_score_stack_);
        break;
    case 1:
        this->FrontierScoreAssign(direct_score_stack_);
        break;
    default:
        this->VisibilityScoreAssign(direct_score_stack_);
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
void VB_Planner::VisibilityScoreAssign(std::vector<float>& score_array) {
	int num_direct = direct_stack_.size();
	score_array.clear();
	score_array.reserve(num_direct);
    std::vector<float> temp_score_stack;
	temp_score_stack.reserve(num_direct);
    for (int i=0; i<num_direct; i++) {
        temp_score_stack[i] = this->rayCast(direct_stack_[i]);
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
        score_array[i] = score / float(counter);
    }
}

void VB_Planner::FrontierScoreAssign(std::vector<float>& score_array) {
	int num_direct = direct_stack_.size();
	score_array.clear();
	score_array.reserve(num_direct);
    std::vector<float> temp_score_stack;
	temp_score_stack.reserve(num_direct);
    for (int i=0; i<num_direct; i++) {
        temp_score_stack[i] = float(this->PointCounter(direct_stack_[i]));
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
        score_array[i] = score / float(counter);
    }
}

void VB_Planner::DeadEndAnalysis(double dist) {
    // Wiil update dead_end_ flag
    int num_dist = max_score_stack_.size();
    double average = 0;
    if (num_dist < dead_end_filter_) {
        max_score_stack_.push_back(dist);
        num_dist += 1;
    }else {
        max_score_stack_.erase(max_score_stack_.begin());
        max_score_stack_.push_back(dist);
    }
    for (std::size_t i=0; i<num_dist; i++) {
        average += max_score_stack_[i];
    }
    average /= num_dist;
    if (average < dead_end_thred_) {
        dead_end_ = true;
        max_score_stack_.clear();
        return;
    }
    dead_end_ = false;
}

void VB_Planner::ElasticRayCast() {
    // TODO -> Right now is the simple version of ray casting
    int counter = 0;
    Point center_pos = this->CPoint(robot_pos_.x, robot_pos_.y);
    Point check_pos_principal = center_pos;
    // principal direction
    double dist = this->Norm(check_pos_principal - center_pos);
    while(counter < obs_count_thred_ && this->Norm(check_pos_principal - center_pos) < max_sensor_range_) {
        check_pos_principal.x += open_direction_.x / ray_cast_resolution_;
        check_pos_principal.y += open_direction_.y / ray_cast_resolution_;

        if (this->HitObstacle(check_pos_principal)) {
            counter += 1;
        }
        dist = this->Norm(check_pos_principal - center_pos);
    }
    this->DeadEndAnalysis(dist); // update dead_end with limit time instances
    goal_waypoint_.pose.position.x = check_pos_principal.x;
    goal_waypoint_.pose.position.y = check_pos_principal.y;
    goal_waypoint_.pose.position.z = robot_pos_.z;

}

float VB_Planner::rayCast(Point direction) {
    // Input: PointCloud; and Principal direction vector
    // Output: update goal waypoint -- the travel distance to an obstacle
    int counter = 0;
    float center_x = robot_pos_.x;
    float center_y = robot_pos_.y;
    Point center_pos = this->CPoint(center_x, center_y);
    Point check_pos_principal = center_pos;
    // principal direction
    while(counter < obs_count_thred_ && this->Norm(check_pos_principal - center_pos) < max_sensor_range_) {
        check_pos_principal.x += direction.x / ray_cast_resolution_;
        check_pos_principal.y += direction.y / ray_cast_resolution_;

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

int VB_Planner::PointCounter(Point direction) {
	// TODO! Counte frontier point in direction
    double yaw_mid = atan2(direction.y, direction.x);
    double yaw_upper = fmin(yaw_mid + direction_resolution_, M_PI_2);
    double yaw_low = fmax(yaw_mid + direction_resolution_, - M_PI_2);
    double yaw_cur;
    int counter = 0;
    for (std::size_t i=0; i<frontier_size_; i++) {
        yaw_cur = frontier_direction_stack_[i];
        if (yaw_cur>yaw_low && yaw_cur<yaw_upper) {
            counter += 1;
        }
    }
    return counter;
}
void VB_Planner::UpdateFrontierDirectionArray(std::vector<double>& direction_array) {
    direction_array.clear();
    frontier_size_ = laser_frontier_filtered_->points.size();
    direction_array.resize(frontier_size_);
    pcl::PointXYZI point;
    for (std::size_t i=0; i<frontier_size_; i++) {
        point = laser_frontier_filtered_->points[i];
        double yaw = atan2(point.y, point.x);
        direction_array[i] = yaw;
    }
}


void VB_Planner::OdomHandler(const nav_msgs::Odometry odom_msg) {
    // Credit: CMU SUB_T dfs_behavior_planner, Chao C.,
    // https://bitbucket.org/cmusubt/dfs_behavior_planner/src/master/src/dfs_behavior_planner/dfs_behavior_planner.cpp

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
    if (old_open_direction_ == this->CPoint(0,0)) {
        old_open_direction_ = robot_heading_;
    }
}

void VB_Planner::UpdaterayCastingStack() {
    Point heading_right, heading_left;
    double angle_step;
    if (dead_end_) {
        old_open_direction_ = this->CPoint(0,0) - old_open_direction_; // if dead end -> inverse driection
    }
    double yaw = atan2(old_open_direction_.y, old_open_direction_.x);
    direct_stack_.push_back(old_open_direction_);
    switch (planner_type_)
    {
    case 0:
        angle_step = M_PI * 1.2 / float(angle_resolution_);
        break;
    case 1:
        angle_step = 2 * M_PI * 1.2 / float(angle_resolution_); // 360 degree sensing
        break;
    default:
        angle_step = M_PI * 1.2 / float(angle_resolution_);
        break;
    }
    direction_resolution_ = angle_step;
    for (int i=1; i<int(angle_resolution_/2); i++) {
        heading_right = this->CPoint(cos(yaw+i*angle_step),sin(yaw+i*angle_step));
        heading_left = this->CPoint(cos(yaw-i*angle_step),sin(yaw-i*angle_step));
        direct_stack_.push_back(heading_right);
        direct_stack_.push_back(heading_left);
    }
}

void VB_Planner::LeftRotatePoint(pcl::PointXYZI &pnt) {
// Source:https://bitbucket.org/cmusubt/misc_utils/src/master/src/misc_utils.cpp
    float tmp_z = pnt.z;
    pnt.z = pnt.y;
    pnt.y = pnt.x;
    pnt.x = tmp_z;
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
    goal_waypoint_.header.frame_id = "world";
    goal_pub_.publish(goal_waypoint_);
    rviz_direct_pub_.publish(rviz_direction_);
    // std::cout << "Goal Publihsed ..." << std::endl;
}

void VB_Planner::CloudHandler(const sensor_msgs::PointCloud2ConstPtr laser_msg) {
    // Take laser cloud -> update Principal Direction
    laser_cloud_->clear();
    pcl::fromROSMsg(*laser_msg, *laser_cloud_);
    this->LaserCloudFilter(laser_cloud_filtered_);
    kdtree_collision_cloud_->setInputCloud(laser_cloud_filtered_);
}

void VB_Planner::FrontierCloudHandler(const sensor_msgs::PointCloud2ConstPtr frontier_msg) {
    // Take laser cloud -> update Principal Direction
    frontier_cloud_->clear();
    pcl::fromROSMsg(*frontier_msg, *frontier_cloud_);
    this->LaserCloudFilter(laser_frontier_filtered_);
    kdtree_frontier_cloud_->setInputCloud(laser_frontier_filtered_);
}

void VB_Planner::LaserCloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud) {
    // Source credit: http://pointclouds.org/documentation/tutorials/passthrough.php
    pcl::PassThrough<pcl::PointXYZI> cloud_filter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_temp->clear();
    std::size_t laser_cloud_size = laser_cloud_->points.size();
    pcl::PointXYZI point;
    for (std::size_t i=0; i<laser_cloud_size; i++) {
        point = laser_cloud_->points[i];
        this->LeftRotatePoint(point);
        // point.z = robot_pos_.z;
        laser_cloud_temp->points.push_back(point);
    }
    laser_cloud_ = laser_cloud_temp;

    cloud_filter.setInputCloud (laser_cloud_);
    cloud_filter.setFilterFieldName ("z");
    cloud_filter.setFilterLimits (robot_pos_.z-2*collision_radius_, robot_pos_.z+2*collision_radius_);
    //pass.setFilterLimitsNegative (true);
    cloud_filter.filter(*filtered_cloud);
}

float VB_Planner::Norm(Point p) {
    return sqrt(p.x*p.x + p.y*p.y);
}

/* ---------------------------------------------------------------------------- */
