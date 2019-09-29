#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PointStamped.msg>
#include <geometry_msgs/Point.msg>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <math>

typedef geometry_msgs::Point ROSPoint;
typedef geometry_msgs::PointStamped ROSWayPoint;

struct Point {
    float x;
    float y;
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
    ros::Subscriber odom_sub_;
    ros::Publishder goal_pub_;
    // function define
    void PrincipalAnalysis();
    void ElasticRawCast();
    void InitializeParam();
    void OdomHandler(const nav_msgs::Odometry odom_msg);
    ROSWayPoint HandleWaypoint();
    void CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg);
    // valuable define
    nav_msgs::Odometry odom_;
    Point robot_heading_;
    ROSPoint goal_waypoint_;
    ROSPoint principal_direction_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_(new pcl::PointCloud<pcl::PointXYZI>());



}