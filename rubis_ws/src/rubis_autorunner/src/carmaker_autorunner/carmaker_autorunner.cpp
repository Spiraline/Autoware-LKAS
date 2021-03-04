#include <carmaker_autorunner/carmaker_autorunner.h>

void CarMakerAutorunner::Run(){
    register_subscribers();             // Register subscribers that shoud check can go next or not
    ros_autorunner_.init(nh_, sub_v_);   // Initialize the ROS-Autorunner
    ros::Rate rate(1);                  // Rate can be changed
    while(ros::ok()){               
        ros_autorunner_.Run();           // Run Autorunner
        ros::spinOnce();
        rate.sleep();
    }    
}

void CarMakerAutorunner::register_subscribers(){
    sub_v_.resize(TOTAL_STEP_NUM);          // Resizing the subscriber vectors. Its size must be same with number of steps

    // Set the check function(subscriber)
    sub_v_[STEP(1)] = nh_.subscribe("/imu_raw", 1, &CarMakerAutorunner::imu_raw_cb, this);
    sub_v_[STEP(2)] = nh_.subscribe("/filtered_points", 1, &CarMakerAutorunner::filtered_points_cb, this);
    sub_v_[STEP(3)] = nh_.subscribe("/current_pose", 1, &CarMakerAutorunner::current_pose_cb, this);
    sub_v_[STEP(4)] = nh_.subscribe("/detection/image_detector/objects", 1, &CarMakerAutorunner::detection_objects_cb, this);
    sub_v_[STEP(5)] = nh_.subscribe("/detection/object_tracker/objects_top", 1, &CarMakerAutorunner::detection_objects_from_tracker_cb, this);
    sub_v_[STEP(6)] = nh_.subscribe("/lane_waypoints_array", 1, &CarMakerAutorunner::lane_waypoints_array_cb, this);
    sub_v_[STEP(7)] = nh_.subscribe("/local_trajectory_cost", 1, &CarMakerAutorunner::local_traj_cost_cb, this);
    sub_v_[STEP(8)] = nh_.subscribe("/behavior_state", 1, &CarMakerAutorunner::behavior_state_cb, this);
    // sub_v_[STEP(9)];
}

void CarMakerAutorunner::imu_raw_cb(const sensor_msgs::Imu& msg){
    if(!ros_autorunner_.step_info_list_[STEP(2)].is_prepared){
        ROS_WARN("[STEP 1] Connected with Carmaker");
        ROS_WARN("[STEP 1] Imu Info Detected");
        // sleep(SLEEP_PERIOD);
        sleep(5);
        ros_autorunner_.step_info_list_[STEP(2)].is_prepared = true;
    }
}

void CarMakerAutorunner::filtered_points_cb(const sensor_msgs::PointCloud2& msg){
    if(!msg.fields.empty() && !ros_autorunner_.step_info_list_[STEP(3)].is_prepared){
        ROS_WARN("[STEP 2] Voxel Grid filter activated");
    	sleep(SLEEP_PERIOD);
        ros_autorunner_.step_info_list_[STEP(3)].is_prepared = true;
    }
}

void CarMakerAutorunner::current_pose_cb(const geometry_msgs::PoseStamped& msg){
    if(!ros_autorunner_.step_info_list_[STEP(4)].is_prepared){
        ROS_WARN("[STEP 3] Localization Success!");
        ROS_WARN("[STEP 3] NDT pose Detected");
        sleep(SLEEP_PERIOD);
        ros_autorunner_.step_info_list_[STEP(4)].is_prepared = true;
    }
}

void CarMakerAutorunner::detection_objects_cb(const autoware_msgs::DetectedObjectArray& msg){
    if(!ros_autorunner_.step_info_list_[STEP(5)].is_prepared){
        ROS_WARN("[STEP 4] Object Deteced from Vision");
        sleep(SLEEP_PERIOD);
        ros_autorunner_.step_info_list_[STEP(5)].is_prepared = true;
    }
}

void CarMakerAutorunner::detection_objects_from_tracker_cb(const autoware_msgs::DetectedObjectArray& msg){
    if(!ros_autorunner_.step_info_list_[STEP(6)].is_prepared){
        ROS_WARN("[STEP 5] Object tracking Complete");
        sleep(SLEEP_PERIOD);
        ros_autorunner_.step_info_list_[STEP(6)].is_prepared = true;
    }
}

void CarMakerAutorunner::lane_waypoints_array_cb(const autoware_msgs::LaneArray& msg){
    if(!msg.lanes.empty() && !ros_autorunner_.step_info_list_[STEP(7)].is_prepared){
        ROS_WARN("[STEP 6] Global path is created");
        ros_autorunner_.step_info_list_[STEP(7)].is_prepared = true;
    }
}

void CarMakerAutorunner::local_traj_cost_cb(const autoware_msgs::Lane& msg){
    if(!ros_autorunner_.step_info_list_[STEP(8)].is_prepared){
        ROS_WARN("[STEP 7] Trajectory Evaluate Complete");
        ros_autorunner_.step_info_list_[STEP(8)].is_prepared = true;
    }
}

void CarMakerAutorunner::behavior_state_cb(const visualization_msgs::MarkerArray& msg){
    std::string state = msg.markers.front().text;
    ROS_WARN("[STEP 8] Behavior State %s", state.c_str());
    if(!msg.markers.empty() && state.find(std::string("Forward"))!=std::string::npos){
        ROS_WARN("[STEP 8] Behvior state is set to forward");
        ros_autorunner_.step_info_list_[STEP(9)].is_prepared = true;
    }
}