#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include "tf/transform_listener.h"
#include <math.h>

class PCDClipper{
private:
    ros::Subscriber sub;
    ros::Publisher clipped_pcd_pub, localize_pcd_pub;

    // for rosparam
    bool enableLocalize;
    float tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
    std::string input_topic;
    std::string clipped_point_output_topic;
    std::string localization_point_output_topic;
    std::string input_frame_id;
    std::string output_frame_id;
    float localization_center_angle;
    float localization_viewing_angle;
    float clipped_center_angle;
    float clipped_viewing_angle;

    tf::TransformListener   listener;
    tf::StampedTransform    transform;

    void points_cb(const sensor_msgs::PointCloud2& msg);
    bool isInside(float y, float x, float center_angle, float viewing_angle);
    // bool findTransform();
public:
    PCDClipper();
    void MainLoop();
};