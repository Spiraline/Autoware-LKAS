#include <ros/ros.h>
#include <ros_autorunner/ros_autorunner.h>

// Include subscribe message type
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Lane.h>
#include <visualization_msgs/MarkerArray.h>

#define TOTAL_STEP_NUM 9 // Need to change when total step number is changed
#define SLEEP_PERIOD 1

class CarMakerAutorunner : public AutorunnerBase{
private:    
    ros::NodeHandle     nh_;
    ROSAutorunner        ros_autorunner_;
private:
    virtual void register_subscribers();
private:
    void imu_raw_cb(const sensor_msgs::Imu& msg);
    void filtered_points_cb(const sensor_msgs::PointCloud2& msg);
    void current_pose_cb(const geometry_msgs::PoseStamped& msg);
    void detection_objects_cb(const autoware_msgs::DetectedObjectArray& msg);
    void detection_objects_from_tracker_cb(const autoware_msgs::DetectedObjectArray& msg);
    void lane_waypoints_array_cb(const autoware_msgs::LaneArray& msg);
    void local_traj_cost_cb(const autoware_msgs::Lane& msg);
    void behavior_state_cb(const visualization_msgs::MarkerArray& msg);

public:
    Sub_v               sub_v_;
public:
    CarMakerAutorunner() {}
    CarMakerAutorunner(ros::NodeHandle nh) : nh_(nh){}
    virtual void Run();
};
