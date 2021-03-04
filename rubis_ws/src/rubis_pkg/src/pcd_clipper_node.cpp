#include "pcd_clipper.h"

PCDClipper::PCDClipper(){
    ros::NodeHandle nh;
    ros::Rate rate(10);

    // for topic name
    std::string node_name = ros::this_node::getName();
    std::string enableLocalize_name = node_name + "/enableLocalize";

    std::string input_topic_name = node_name + "/input_topic";
    std::string clipped_pcd_topic_name = node_name + "/clipped_point_output_topic";
    std::string localization_pcd_topic_name = node_name + "/localization_point_output_topic";

    std::string input_frame_id_topic_name = node_name + "/input_frame_id";
    std::string output_frame_id_topic_name = node_name + "/output_frame_id";

    std::string localization_center_angle_topic_name = node_name + "/localization_center_angle";
    std::string localization_viewing_angle_topic_name = node_name + "/localization_viewing_angle";
    std::string clipped_center_angle_topic_name = node_name + "/clipped_center_angle";
    std::string clipped_viewing_angle_topic_name = node_name + "/clipped_viewing_angle";

    

    nh.param<bool>(enableLocalize_name, enableLocalize, true);
    nh.param<std::string>(input_topic_name, input_topic, "/points_raw_origin");
    nh.param<std::string>(clipped_pcd_topic_name, clipped_point_output_topic, "/clipped_points_raw");
    nh.param<std::string>(localization_pcd_topic_name, localization_point_output_topic, "/localization_points_raw");

    nh.param<std::string>(input_frame_id_topic_name, input_frame_id, "/velodyne_left");
    nh.param<std::string>(output_frame_id_topic_name, output_frame_id, "/velodyne");

    nh.param<float>(localization_center_angle_topic_name, localization_center_angle, 0);
    nh.param<float>(localization_viewing_angle_topic_name, localization_viewing_angle, 360);
    nh.param<float>(clipped_center_angle_topic_name, clipped_center_angle, 0);
    nh.param<float>(clipped_viewing_angle_topic_name, clipped_viewing_angle, 360);

    sub = nh.subscribe(input_topic, 1, &PCDClipper::points_cb, this);

    if(enableLocalize){
        localize_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>(localization_point_output_topic, 1);
    }

    clipped_pcd_pub = nh.advertise<sensor_msgs::PointCloud2>(clipped_point_output_topic, 1); 
}

bool PCDClipper::isInside(float y, float x, float center_angle, float viewing_angle){
    float angle = atan2(fabs(y), fabs(x)) * 57.295779; // deg = rad * 180 / PI
    float angle_from_left = center_angle + viewing_angle / 2;
    float angle_from_right = center_angle - viewing_angle / 2;

    if(x > 0 && y > 0){ // quadrant 1
        if(angle_from_right <= 0 && 90 + angle_from_right < angle) return true;
        if(angle_from_left - 270 > angle && angle_from_right - 270 < angle) return true;
        return false;
    }
    else if(x < 0 && y > 0){ // quadrant 2
        if(angle_from_left >= 360 && 450 - angle_from_left < angle) return true;
        if(90 - angle_from_right > angle && 90 - angle_from_left < angle) return true;
        return false;
    }
    else if(x < 0 && y < 0){ // quadrant 3
        if(angle_from_left >= 450 && angle_from_left - 450 > angle) return true;
        if(angle_from_right - 90 < angle && angle_from_left - 90 > angle) return true;
        return false;
    }
    else if(x > 0 && y < 0){ // quadrant 4
        if(angle_from_right <= -90 && -(90 + angle_from_right) > angle) return true;
        if(270 - angle_from_left < angle && 270 - angle_from_right > angle) return true;
        return false;
    }
    return false;
}


void PCDClipper::points_cb(const sensor_msgs::PointCloud2& msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr localization_points(new pcl::PointCloud<pcl::PointXYZI>);
    
    sensor_msgs::PointCloud2 localization_out;
    sensor_msgs::PointCloud2 clipped_out;

    sensor_msgs::PointCloud2 msg_with_intensity = msg;
    msg_with_intensity.fields.at(3).datatype = 7;

    pcl::fromROSMsg(msg_with_intensity, *pcl_points);

    for(int i=0; i<(*pcl_points).size(); i++){
        if(enableLocalize && isInside(pcl_points->points[i].x, pcl_points->points[i].y, localization_center_angle, localization_viewing_angle)){
            localization_points->points.push_back(pcl_points->points[i]);
        }
        if(isInside(pcl_points->points[i].x, pcl_points->points[i].y, clipped_center_angle, clipped_viewing_angle)){
            clipped_points->points.push_back(pcl_points->points[i]);
        }
    }

    // if(findTransform()==false){
    //     return;
    // }

    // try{
    //     pcl_ros::transformPointCloud(*clipped_points, *clipped_points, transform);
    //     if(enableLocalize)
    //         pcl_ros::transformPointCloud(*localization_points, *localization_points, transform);
    // }
    // catch(tf::TransformException& ex){
    //     ROS_ERROR("transformPointCloud");
    //     ROS_ERROR("%s", ex.what());
    //     return;
    // }

    if(enableLocalize){
        pcl::toROSMsg(*localization_points, localization_out);
        localization_out.header = msg.header;
        // localization_out.header.frame_id = output_frame_id;
        localize_pcd_pub.publish(localization_out);
    }

    pcl::toROSMsg(*clipped_points, clipped_out);
    clipped_out.header = msg.header;
    // clipped_out.header.frame_id = output_frame_id;
    clipped_pcd_pub.publish(clipped_out);
}

// bool PCDClipper::findTransform(){
//     try{
//         listener.waitForTransform(output_frame_id, input_frame_id, ros::Time(0), ros::Duration(0.001));
//         listener.lookupTransform(output_frame_id, input_frame_id, ros::Time(0), transform);
//         return true;
//     }
//     catch(tf::TransformException& ex){
//         ROS_ERROR("FindTransform");
//         ROS_ERROR("%s", ex.what());
//         return false;
//     }
// }

void PCDClipper::MainLoop(){
    while(ros::ok()){        
        ros::spin();
    }
    return;
}