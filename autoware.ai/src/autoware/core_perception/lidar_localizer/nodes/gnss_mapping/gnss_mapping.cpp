#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h> // Need for voxel grid filter

#include <autoware_config_msgs/ConfigNDTMapping.h>
#include <autoware_config_msgs/ConfigNDTMappingOutput.h>

#include <time.h>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

// global variables
static pose previous_gnss_pose, current_gnss_pose, localizer_pose, added_pose;
static int is_gnss_ready = 2;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;

static pcl::PointCloud<pcl::PointXYZI> map;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

// Leaf size of VoxelGrid filter.
static double _voxel_leaf_size = 2.0;
static double _output_filter_resolution = 0.0;
static std::string _output_path = "~/";

static double _min_scan_range = 5.0;
static double _max_scan_range = 200.0;
static double _min_add_scan_shift = 1.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool _incremental_voxel_update = false;

static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  current_gnss_pose.x = input->pose.position.x;
  current_gnss_pose.y = input->pose.position.y;
  current_gnss_pose.z = input->pose.position.z;

  double x_diff = current_gnss_pose.x - previous_gnss_pose.x;
  double y_diff = current_gnss_pose.y - previous_gnss_pose.y;
  double z_diff = current_gnss_pose.z - previous_gnss_pose.z;

  if(is_gnss_ready > 0)
  {
    if(is_gnss_ready == 1)
    {
      current_gnss_pose.roll = 0;
      current_gnss_pose.pitch = 0;
      current_gnss_pose.yaw = atan2(y_diff, x_diff);
    }
    previous_gnss_pose = current_gnss_pose;
    is_gnss_ready--;
    return;
  }

  current_gnss_pose.yaw += atan2(y_diff, x_diff);
  previous_gnss_pose = current_gnss_pose;
}

static void save_map()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  map_ptr->header.frame_id = "map";
  map_filtered->header.frame_id = "map";
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

  // Apply voxelgrid filter
  if (_output_filter_resolution == 0.0)
  {
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  }
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(_output_filter_resolution, _output_filter_resolution, _output_filter_resolution);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*map_filtered);
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    std::cout << "Filtered: " << map_filtered->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_filtered, *map_msg_ptr);
  }

  ndt_map_pub.publish(*map_msg_ptr);

  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm* pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
  std::string filename = _output_path + "/" + std::string(buffer) + ".pcd";

  // Writing Point Cloud data to PCD file
  if (_output_filter_resolution == 0.0)
  {
    pcl::io::savePCDFileASCII(filename, *map_ptr);
    std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
  }
  else
  {
    pcl::io::savePCDFileASCII(filename, *map_filtered);
    std::cout << "Saved " << map_filtered->points.size() << " data points to " << filename << "." << std::endl;
  }
}

static double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

static double wrapToPmPi(double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if(!is_gnss_ready) return;

  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (_min_scan_range < r && r < _max_scan_range)
    {
      scan.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(_voxel_leaf_size, _voxel_leaf_size, _voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

  Eigen::AngleAxisf gnss_rotation_x(current_gnss_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf gnss_rotation_y(current_gnss_pose.pitch, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf gnss_rotation_z(current_gnss_pose.yaw, Eigen::Vector3f::UnitX());

  Eigen::Translation3f gnss_translation(current_gnss_pose.x, current_gnss_pose.y, current_gnss_pose.z);

  t_localizer =
      (gnss_translation * gnss_rotation_z * gnss_rotation_y * gnss_rotation_x).matrix() * tf_btol;

  // Didn't modify below

  t_base_link = t_localizer * tf_ltob;

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b;

  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  transform.setOrigin(tf::Vector3(current_gnss_pose.x, current_gnss_pose.y, current_gnss_pose.z));
  q.setRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_gnss_pose.x - added_pose.x, 2.0) + pow(current_gnss_pose.y - added_pose.y, 2.0));
  if (shift >= _min_add_scan_shift)
  {
    map += *transformed_scan_ptr;
    added_pose.x = current_gnss_pose.x;
    added_pose.y = current_gnss_pose.y;
    added_pose.z = current_gnss_pose.z;
    added_pose.roll = current_gnss_pose.roll;
    added_pose.pitch = current_gnss_pose.pitch;
    added_pose.yaw = current_gnss_pose.yaw;
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  ndt_map_pub.publish(*map_msg_ptr);

  q.setRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_gnss_pose.x;
  current_pose_msg.pose.position.y = current_gnss_pose.y;
  current_pose_msg.pose.position.z = current_gnss_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();

  current_pose_pub.publish(current_pose_msg);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_gnss_pose.x << ", " << current_gnss_pose.y << ", " << current_gnss_pose.z << ", " << current_gnss_pose.roll
            << ", " << current_gnss_pose.pitch << ", " << current_gnss_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  previous_gnss_pose.x = 0.0;
  previous_gnss_pose.y = 0.0;
  previous_gnss_pose.z = 0.0;
  previous_gnss_pose.roll = 0.0;
  previous_gnss_pose.pitch = 0.0;
  previous_gnss_pose.yaw = 0.0;

  current_gnss_pose.x = 0.0;
  current_gnss_pose.y = 0.0;
  current_gnss_pose.z = 0.0;
  current_gnss_pose.roll = 0.0;
  current_gnss_pose.pitch = 0.0;
  current_gnss_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  ros::init(argc, argv, "gnss_mapping");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // setting parameters
  private_nh.getParam("voxel_leaf_size", _voxel_leaf_size);
  private_nh.getParam("min_scan_range", _min_scan_range);
  private_nh.getParam("max_scan_range", _max_scan_range);
  private_nh.getParam("min_add_scan_shift", _min_add_scan_shift);
  private_nh.getParam("incremental_voxel_update", _incremental_voxel_update);
  private_nh.getParam("output_filter_resolution", _output_filter_resolution);
  private_nh.getParam("output_path", _output_path);

  std::cout << "voxel_leaf_size: " << _voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << _min_scan_range << std::endl;
  std::cout << "max_scan_range: " << _max_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << _min_add_scan_shift << std::endl;
  std::cout << "incremental_voxel_update: " << _incremental_voxel_update << std::endl;
  std::cout << "output_filter_resolution: " << _output_filter_resolution << std::endl;
  std::cout << "output_path: " << _output_path << std::endl;

  if (nh.getParam("tf_x", _tf_x) == false)
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_y", _tf_y) == false)
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_z", _tf_z) == false)
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_roll", _tf_roll) == false)
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_pitch", _tf_pitch) == false)
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_yaw", _tf_yaw) == false)
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;


  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  tf_ltob = tf_btol.inverse();

  map.header.frame_id = "map";

  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

  ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);

  ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 100000, gnss_callback);

  ros::Rate loop_rate(10);

  bool _save_map = false;

  while(ros::ok())
  {
    nh.getParam("save_map", _save_map);
    if(_save_map){
      save_map();
      nh.setParam("save_map", false);
      _save_map = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
