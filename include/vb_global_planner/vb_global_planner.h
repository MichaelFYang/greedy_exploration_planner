#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <misc_utils/misc_utils.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <math.h>

typedef geometry_msgs::Point ROSPoint;
typedef geometry_msgs::PointStamped ROSWayPoint; 

struct Point {
    float x;
    float y;
    float operator *(const Point& pt) const
    {
        return x * pt.x + y * pt.y;
    }
    Point operator +(const Point& pt) const
    {
        Point result;
        result.x += pt.x;
        result.y += pt.y;
        return result;
    }
    Point operator -(const Point& pt) const
    {
        Point result;
        result.x -= pt.x;
        result.y -= pt.y;
        return result;
    }
};

struct Point3D {
    float x;
    float y;
    float z;
    float operator *(const Point3D& pt) const
    {
        return x * pt.x + y * pt.y + z * pt.z;
    }
    Point3D operator +(const Point3D& pt) const
    {
        Point3D result;
        result.x += pt.x;
        result.y += pt.y;
        result.z += pt.z;
        return result;
    }
    Point3D operator -(const Point3D& pt) const
    {
        Point3D result;
        result.x -= pt.x;
        result.y -= pt.y;
        result.z -= pt.z;
        return result;
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
    ros::Subscriber odom_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher rviz_direct_pub_;
    // function define
    Point CPoint(float x, float y);
    float Norm(Point p);
    void PrincipalAnalysis();
    void ElasticRawCast();
    void InitializeParam();
    void OdomHandler(const nav_msgs::Odometry odom_msg);
    void HandleWaypoint();
    void CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg);
    bool HitObstacle(Point p);
    void LaserCloudFilter();
    void HandleWayPoint();
    // valuable define
    nav_msgs::Odometry odom_;
    Point robot_heading_;
    Point3D robot_pos_;
    ROSWayPoint goal_waypoint_;
    Point principal_direction_;
    nav_msgs::Path rviz_direction_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_filtered_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_collision_cloud_;
    // ros  parameter value
    int raw_cast_revolution_;
    float max_sensor_range_;
    int obs_count_thred_;
    float collision_radius_;
    std::string robot_frame_id_;
    std::string goal_topic_, laser_topic_, odom_topic_;


};