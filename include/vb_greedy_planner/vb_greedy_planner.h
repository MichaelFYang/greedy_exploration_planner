#ifndef VB_GREEDY_PLANNER_H
#define VB_GREEDY_PLANNER_H

#define VISIBILITY 0
#define FRONTIER 1
#define ANG_RES_Y 2.0
#define BEANS 7

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <string>
#include <math.h>
#include <algorithm>

typedef geometry_msgs::PoseStamped ROSWayPoint; 


struct Point3D {
    float x;
    float y;
    float z;
    Point3D() {}
    Point3D(float _x, float _y, float _z): x(_x), y(_y), z(_z) {}
    bool operator ==(const Point3D& pt) const
    {
        return x == pt.x && y == pt.y && z == pt.z;
    }

    float operator *(const Point3D& pt) const
    {
        return x * pt.x + y * pt.y + z * pt.z;
    }

    Point3D operator *(const float factor) const
    {
        return Point3D(x*factor, y*factor, z*factor);
    }

    Point3D operator /(const float factor) const
    {
        return Point3D(x/factor, y/factor, z/factor);
    }

    Point3D operator +(const Point3D& pt) const
    {
        return Point3D(x+pt.x, y+pt.y, z+pt.z);
    }
    Point3D operator -(const Point3D& pt) const
    {
        return Point3D(x-pt.x, y-pt.y, z-pt.z);
    }
};


class VB_Planner
{
public:
    VB_Planner();
    void Loop();

private:
    // rostopic define
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
	ros::Subscriber frontier_cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher rviz_direct_pub_;
    // function define
    Point3D CPoint(float x, float y, float z);
    float Norm(Point3D p);
    // Point3D PrincipalAnalysis();
    Point3D OpenDirectionAnalysis();
    float rayCast(Point3D direction);
    int PointCounter(Point3D direction);
    void ElasticRayCast();
    void InitializeParam();
    void OdomHandler(const nav_msgs::Odometry odom_msg);
    void HandleWaypoint();
    void CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg);
	void FrontierCloudHandler(const sensor_msgs::PointCloud2ConstPtr frointer_msg);
    bool HitObstacle(Point3D p);
	int ObstacleCounter(Point3D direction);
    void LaserCloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud);
    void HandleWayPoint();
    void DeadEndAnalysis(double dist);
    void UpdateRayCastingStack();
	void VisibilityScoreAssign(std::vector<std::vector<float> >& score_array);
	void FrontierScoreAssign(std::vector<std::vector<float> >& score_array);
    void UpdateFrontierDirectionArray(std::vector<double>& direction_array);
    void LeftRotatePoint(pcl::PointXYZI &pnt);
    // valuable define
    nav_msgs::Odometry odom_;
    bool dead_end_;
    std::size_t frontier_size_;
    std::vector<double> max_score_stack_;
    Point3D robot_heading_;
    std::vector<std::vector<Point3D> > direct_stack_3D_;
    std::vector<std::vector<float> > direct_score_stack_;
    std::vector<double> frontier_direction_stack_;
    std::vector<Point3D> collision_point_stack_;
    Point3D robot_pos_;
    ROSWayPoint goal_waypoint_;
    // Point3D principal_direction_;
    Point3D open_direction_;
    Point3D old_open_direction_;
    Point3D second_direction_left_;
    Point3D second_direction_right_;
    nav_msgs::Path rviz_direction_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr frontier_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_filtered_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_frontier_filtered_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_collision_cloud_;
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_frontier_cloud_;
    // ros  parameter value
    int ray_cast_resolution_;
    int angle_resolution_;
    double direction_resolution_;
    float max_sensor_range_;
    int obs_count_thred_;
    float collision_radius_;
    int dead_end_filter_; // time step for waiting
    float dead_end_thred_;
    int planner_type_;
    
    std::string robot_frame_id_;
    std::string goal_topic_, laser_topic_, frontier_topic_, odom_topic_;


};

#endif
