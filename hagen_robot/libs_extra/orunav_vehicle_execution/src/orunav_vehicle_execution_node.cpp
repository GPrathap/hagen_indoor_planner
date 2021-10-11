#include <orunav_vehicle_execution/orunav_vehicle_execution_node.h>

KMOVehicleExecutionNode::KMOVehicleExecutionNode(ros::NodeHandle &paramHandle){
    bool use_arm;
    // Parameters
    paramHandle.param<int>("robot_id", robot_id_, 1);
    {
      std::vector<int> robot_ids;
      robot_ids.push_back(robot_id_);
      target_handler_.setRobotIDsToCompute(robot_ids);
    }

    paramHandle.param<std::string>("model", model_name_, std::string("cititruck"));
    paramHandle.param<bool>("use_forks", use_forks_, false);
    paramHandle.param<bool>("use_arm", use_arm, false);
    paramHandle.param<double>("time_to_meter_factor", time_to_meter_factor_, 0.02);
    paramHandle.param<double>("max_tracking_error", max_tracking_error_, 100.);

    paramHandle.param<bool>("use_vector_map_and_geofence", use_vector_map_and_geofence_, false);

    paramHandle.param<bool>("traj_debug", traj_params_.debug, true);
    paramHandle.param<double>("wheel_base_x", traj_params_.wheelBaseX, 1.190);
    paramHandle.param<double>("time_step", traj_params_.timeStep, 0.06);
    vehicle_state_.setTimeStep(traj_params_.timeStep);
    paramHandle.param<double>("max_vel", traj_params_.maxVel, 0.1);
    paramHandle.param<double>("max_vel_rev", traj_params_.maxVelRev, traj_params_.maxVel);
    paramHandle.param<double>("max_rotational_vel", traj_params_.maxRotationalVel, 0.1);
    paramHandle.param<double>("max_rotational_vel_rev", traj_params_.maxRotationalVelRev, traj_params_.maxRotationalVel);
    paramHandle.param<double>("max_acc", traj_params_.maxAcc, 0.1);
    paramHandle.param<double>("max_steering_angle_vel", traj_params_.maxSteeringAngleVel, 0.8); // Make sure this is high enough - if the trajectory generation are limited to much by this the velocity profile will vary a lot.
    paramHandle.param<double>("min_incr_path_dist", min_incr_path_dist_, 0.1);
    paramHandle.param<int>("min_nb_path_points_", min_nb_path_points_, 20);
    paramHandle.param<bool>("visualize", visualize_, true);
    traj_params_.useCoordTimeAccConstraints = true;
    traj_params_.useCoordTimeContraintPoints = true;
    traj_params_.debug = true;
    traj_params_.debugPrefix = std::string("ct_traj_gen/");

    paramHandle.param<bool>("overwrite_traj_params_with_velocity_constraints", overwrite_traj_params_with_velocity_constraints_, true);
    traj_params_original_ = traj_params_;
    traj_slowdown_params_ = traj_params_;

    paramHandle.param<double>("max_slowdown_vel", traj_slowdown_params_.maxVel, 0.1);

    paramHandle.param<double>("min_docking_distance", min_docking_distance_, 1.0);
    paramHandle.param<double>("max_docking_distance", max_docking_distance_, 1.3);
    paramHandle.param<double>("min_docking_driven_distance", min_docking_driven_distance_, 0.6);
    paramHandle.param<double>("max_target_angular_diff", max_target_angular_diff_, 0.2);
    paramHandle.param<double>("max_target_distance_diff", max_target_distance_diff_, 0.2);
    paramHandle.param<double>("max_target_distance_diff_fwd", max_target_distance_diff_fwd_, 0.2);
    paramHandle.param<double>("max_target_distance_diff_side", max_target_distance_diff_side_, 0.2);
    paramHandle.param<double>("max_steering_range_smoothed_path", max_steering_range_smoothed_path_, 0.5);
    paramHandle.param<double>("overshoot_distance", overshoot_distance_, 0.);
    paramHandle.param<int>("docking_max_nb_opt_points", docking_max_nb_opt_points_, 40);
    paramHandle.param<bool>("use_ct", use_ct_, false);
    paramHandle.param<bool>("provide_dts", provide_dts_, false);

    paramHandle.param<double>("ebrake_lookahead_time", ebrake_lookahead_time_, 2.);
    paramHandle.param<double>("slowdown_drivingslow_lookahead_time", slowdown_drivingslow_lookahead_time_, 20.);
    paramHandle.param<double>("slowdown_lookahead_time", slowdown_lookahead_time_, 5.);
    paramHandle.param<bool>("use_safetyregions", use_safetyregions_, false);
    std::string safety_laser_topic, safety_laser_topic2;
    paramHandle.param<std::string>("safety_laser_topic", safety_laser_topic, std::string("/laser_scan"));
    paramHandle.param<std::string>("safety_laser_topic2", safety_laser_topic2, std::string("/laser_forkdir_scan"));
    paramHandle.param<int>("chunk_idx_connect_offset", chunk_idx_connect_offset_, 3);
    paramHandle.param<bool>("draw_sweep_area", draw_sweep_area_, false);
    paramHandle.param<bool>("cts_clear_first_entry_in_pairs", cts_clear_first_entry_in_pairs_, true);
    paramHandle.param<bool>("use_ahead_brake", use_ahead_brake_, false);
    paramHandle.param<bool>("visualize_sweep_and_constraints", visualize_sweep_and_constraints_, false);
    paramHandle.param<bool>("use_update_task_service", use_update_task_service_, false);
    paramHandle.param<bool>("start_driving_after_recover", start_driving_after_recover_, true);
    paramHandle.param<bool>("real_cititruck", real_cititruck_, false);
    paramHandle.param<bool>("no_smoothing", no_smoothing_, false);
    paramHandle.param<bool>("resolve_motion_planning_error", resolve_motion_planning_error_, true);

    paramHandle.param<double>("max_linear_vel_pallet_picking", max_linear_vel_pallet_picking_, 0.1);
    paramHandle.param<double>("max_rotational_vel_pallet_picking", max_rotational_vel_pallet_picking_, 0.1);
    paramHandle.param<double>("max_linear_vel_rev_pallet_picking", max_linear_vel_rev_pallet_picking_, max_linear_vel_pallet_picking_);
    paramHandle.param<double>("max_rotational_vel_rev_pallet_picking", max_rotational_vel_rev_pallet_picking_, max_rotational_vel_pallet_picking_);
    
   
    trajectorychunk_pub_ = nh_.advertise<orunav_msgs::ControllerTrajectoryChunkVec>("control/controller/trajectories", 1000);
    command_pub_ = nh_.advertise<orunav_msgs::ControllerCommand>("control/controller/commands", 1000);
    
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    marker_pub1_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
    marker_pub2_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
    marker_pub3_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker3", 10);
    marker_pub4_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker4", 10);
    marker_pub5_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker5", 10);
    marker_pub6_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker6", 10);



    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/robot1/goal", 50, &KMOVehicleExecutionNode::goalTrajectoryCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 50, &KMOVehicleExecutionNode::odomCallback, this);
    smooth_path_pub_ = nh_.advertise<nav_msgs::Path>("/move_base/hagen/smoothpath", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("/move_base/hagen/init_path", 1);
    control_report_sub_ = nh_.subscribe<orunav_msgs::ControllerReport>("control/controller/reports", 10, &KMOVehicleExecutionNode::process_report, this);
    velocity_constraints_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>(orunav_generic::getRobotTopicName(robot_id_, "/velocity_constraints"), 1, &KMOVehicleExecutionNode::process_velocity_constraints, this);
    report_pub_ = nh_.advertise<orunav_msgs::RobotReport>("control/report", 1);

    heartbeat_slow_visualization_ = nh_.createTimer(ros::Duration(1.0), &KMOVehicleExecutionNode::publish_visualization_slow, this);
    heartbeat_fast_visualization_ = nh_.createTimer(ros::Duration(0.1), &KMOVehicleExecutionNode::publish_visualization_fast, this);
    heartbeat_report_pub_ = nh_.createTimer(ros::Duration(1.0), &KMOVehicleExecutionNode::publish_report, this);
   
   
    path_smoother.init(nh_);
    path_planner.init(nh_);

    if (use_arm)
    {
      model_ = &model2;
    }
    else
    {
      model_ = &model1;
    }
    if (model_name_ != std::string("cititruck"))
    {
      if (model_name_ == std::string("pitviper"))
      {
        model_ = &model_pitviper;
      }
      else if (model_name_ == std::string("bt"))
      {
        model_ = &model_bt;
      }
    }

    valid_map_ = false;
    b_shutdown_ = false;

    //call worker thread
    // client_thread_ = boost::thread(boost::bind(&KMOVehicleExecutionNode::run, this));
  }

void KMOVehicleExecutionNode::visualizeCurrentMission()
  {

    boost::mutex::scoped_lock lock(current_mutex_);
    if (current_path_.sizePath() > 0)
    {
      orunav_generic::Path path = orunav_generic::minIncrementalDistancePath(current_path_, 0.1);
      orunav_rviz::drawPathInterface(path, "current_path_subsampled", 0, traj_params_.wheelBaseX, marker_pub3_);
      orunav_rviz::drawPoint2dContainerAsConnectedLineIncZ_(orunav_generic::createPoint2dVecFromPose2dContainerInterface(path), "path_points", 1, 1, 0., 0., false, marker_pub3_);

      if (current_cts_.size() == current_path_.sizePath() && vehicle_state_.isDriving())
      {
        orunav_rviz::drawCoordinatedTimes(current_path_, current_cts_, current_start_time_, "cts", 2, time_to_meter_factor_, 0.12, marker_pub3_);
        orunav_rviz::drawPose2dTimesAsLines(current_path_, current_cts_, current_start_time_, "cts_lines", robot_id_, time_to_meter_factor_, 0.02, marker_pub3_);
      }
    }

    // The trajectory
    orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
    orunav_rviz::drawTrajectoryChunksWithControl(chunks, 2, "chunks", marker_pub1_);

    // Draw the trajetory as a path / cts, each pose2d is separated with 60 ms.
    orunav_rviz::drawTrajectoryChunksUsingTime(chunks, 2, "chunks_dt", current_start_time_, time_to_meter_factor_, traj_params_.timeStep, marker_pub1_);

    if (!visualize_sweep_and_constraints_)
      return;

    // The sweep area
    int current_path_idx = vehicle_state_.getCurrentPathIdx();

    // if (draw_sweep_area_ && current_path_idx > 0 && current_path_idx < current_path_.sizePath() - 1)
    // {
    //   orunav_geometry::Polygon current_sweep_area = constraint_extract::computeSweepArea(orunav_generic::selectPathIntervall(current_path_, current_path_idx, current_path_.sizePath()), *model_, vehicle_state_.getInternalState2d());
    //   orunav_rviz::drawPoint2dContainerAsConnectedLine(current_sweep_area, "sweep", 2, 2, marker_pub4_);
    // }

    // Constrains related
    // if (current_constraints_path_.sizePath() != current_constraints_.size())
    //   return;

    // Draw all active constraints
    // for (size_t i = 0; i < current_constraints_.size(); i++)
    // {
    //   const constraint_extract::PolygonConstraint constraint = current_constraints_[i];
    //   orunav_generic::Pose2d pose = current_constraints_path_.getPose2d(i);

    //   if (constraint.isActive(pose))
    //   {
    //     orunav_rviz::drawPoint2dContainerAsConnectedLine(constraint.getInnerConstraint(), "constraints_xy", i, 0, marker_pub_);
    //     orunav_rviz::drawPose2d(orunav_generic::Pose2d(pose(0), pose(1), constraint.getThBounds()(0)), i, 3, 0.3, "constraints_th_left", marker_pub_);
    //     orunav_rviz::drawPose2d(orunav_generic::Pose2d(pose(0), pose(1), constraint.getThBounds()(1)), i, 3, 0.3, "constraints_th_right", marker_pub_);

    //     // Draw the outer polygon as well if available.
    //     if (i < current_constraints_outer_.size())
    //     {
    //       orunav_rviz::drawPoint2dContainerAsConnectedLine(current_constraints_outer_[i], "constraints_outer_xy", i, 2, marker_pub_);
    //       orunav_rviz::drawPoint2dContainerAsConnectedLine(constraint.getOuterConstraint(), "constraints_outer2_xy", i, 2, marker_pub_);
    //     }
    //   }
    // }
  }

void KMOVehicleExecutionNode::publish_visualization_slow(const ros::TimerEvent &event){
    if (!visualize_)
      return;

    visualizeCurrentMission();

    // Draw driven trajectory.
    orunav_generic::Trajectory traj = vehicle_state_.getDrivenTrajectory();
    orunav_generic::CoordinatedTimes ct = vehicle_state_.getDrivenTrajectoryTimes();

    assert(traj.sizePath() == ct.size());
    // std::cout<< "=======================================wating to get starat==========="<<std::endl;
    if (!vehicle_state_.isWaiting())
    {
      // std::cout<< "=======================================wating ==========="<<std::endl;
      orunav_rviz::drawCoordinatedTimes(traj, ct, current_start_time_, "driven_ct", robot_id_, time_to_meter_factor_, marker_pub2_);
      orunav_rviz::drawTrajectoryWithControl(traj, 0, 0, "driven_traj", marker_pub2_);
    }
}

bool KMOVehicleExecutionNode::validTask(const orunav_msgs::Task &task)
  {
    // Check the critical point idx. Must be within the path lenght (or == -1).
    if (task.criticalPoint == -1)
      return true;

    int path_size = task.path.path.size();
    if (task.criticalPoint >= path_size)
    {
      ROS_WARN("[KMOVehicleExecutionNode] RID:%d - invalid critical point idx:%d (size of path:%d)", robot_id_, task.criticalPoint, path_size);
      return false;
    }
    return true;
  }

  bool KMOVehicleExecutionNode::validTarget(const orunav_msgs::RobotTarget &target)
  {
    // ACTIVATE_SUPPORT_LEGS not valid as a start operation
    if (target.start_op.operation == target.start_op.ACTIVATE_SUPPORT_LEGS)
    {
      ROS_WARN("start_op.operation == start_op.ACTIVATE_SUPPORT_LEGS");
      return false;
    }
    return true;
  }

  bool KMOVehicleExecutionNode::validTaskMsg(const orunav_msgs::Task &task) const {
    // Any path points?
    if (task.path.path.size() < 3) {
      ROS_ERROR_STREAM("Not a valid task: path to short, current length : " << task.path.path.size()); 
      return false;
    }
    if (use_ct_) {
      if (task.dts.dts.size() < 2) { // 2- the fastest and slowest
        ROS_ERROR_STREAM("Not a valid task: no dts vectors, current length: " << task.dts.dts.size());
        return false;
      }
      if (task.path.path.size() != task.dts.dts[0].dt.size()) {
        ROS_ERROR_STREAM("Not a valid task: dts[0] length different from path length: " << task.dts.dts[0].dt.size());
        return false;
      }
    }
    if (task.path.path.size() < task.constraints.constraints.size()) {
      ROS_ERROR_STREAM("Not a valid task: amount of constraints larger the path length: " << task.constraints.constraints.size());
      return false;
    }
    return true;
  }

  
void KMOVehicleExecutionNode::publish_visualization_fast(const ros::TimerEvent &event){

    orunav_rviz::drawText(vehicle_state_.getCurrentState2d().getPose2d(),
                          vehicle_state_.getDebugStringExtended(),
                          "state_str", 1, 1, 0.8, 0.3, marker_pub4_);

    orunav_rviz::drawText(vehicle_state_.getCurrentState2d().getPose2d(),
                          orunav_generic::toString(robot_id_),
                          "robot_id_str", 1, 3, 1.0, 2.0, marker_pub4_);

    orunav_rviz::drawPosition(vehicle_state_.getCurrentState2d().getPose2d()[0],
                              vehicle_state_.getCurrentState2d().getPose2d()[1],
                              (ros::Time::now().toSec() - current_start_time_) * time_to_meter_factor_,
                              1,
                              marker_pub4_);

    // Draw the safety zones.
    // if (use_safetyregions_)
    // {
    //   orunav_rviz::drawPoint2dContainerAsConnectedLine(current_global_ebrake_area_, "ebrake", 0, 0, marker_pub_);
    //   orunav_rviz::drawPoint2dContainerAsConnectedLine(current_global_slowdown_area_, "slowdown", 0, 1, marker_pub_);
    // }
    orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
    orunav_generic::Path path = vehicle_state_.getPath();


    // std::cout<< "----------------------------->>>> chunks.empty() "<<  chunks.empty() << std::endl;
    if (!chunks.empty())
    {
      {
        int current_path_idx = vehicle_state_.getCurrentPathIdx();
        //        ROS_INFO_STREAM("current_path_idx : " << current_path_idx);
        if (current_path_idx >= 0 && current_path_idx < path.sizePath())
        {
          orunav_generic::Pose2d p = path.getPose2d(current_path_idx);
          orunav_rviz::drawPose2d(p, 1, 1, 2., std::string("current_path_idx"), marker_pub3_);
        }
      }
      {
        int earliest_path_idx = vehicle_state_.getEarliestPathIdxToConnect();
        ROS_INFO_STREAM("earliest_path_idx : " << earliest_path_idx << "============="<< path.sizePath());
        if (earliest_path_idx >= 0 && earliest_path_idx < path.sizePath())
        {
          orunav_generic::Pose2d p = path.getPose2d(earliest_path_idx);
          orunav_rviz::drawPose2d(p, 1, 0, 2., std::string("earliest_connect_path_idx"), marker_pub6_);
        }
        int docking_path_idx = getDockingPathIdx(path, earliest_path_idx, min_docking_distance_, max_docking_distance_);
        //        ROS_INFO_STREAM("docking_path_idx : " << docking_path_idx);
        if (docking_path_idx >= 0 && docking_path_idx < path.sizePath())
        {
          orunav_generic::Pose2d p = path.getPose2d(docking_path_idx);
          orunav_rviz::drawPose2d(p, 1, 0, 2., std::string("docking_path_idx"), marker_pub3_);
        }
      }
    }
    // Critical point
    {
      int critical_point_idx = vehicle_state_.getCriticalPointIdx();
      if (path.sizePath() > 0)
      {
        if (critical_point_idx < 0)
          critical_point_idx = path.sizePath() - 1;
        if (critical_point_idx < path.sizePath())
        {
          orunav_rviz::drawSphere(path.getPose2d(critical_point_idx)[0],
                                  path.getPose2d(critical_point_idx)[1],
                                  0.2,
                                  1,
                                  std::string("critical_point"),
                                  marker_pub5_);
        }
      }
      else
      { // If the crit point is zero, no path is loaded to vehicle_state, draw the current state just for illustration.
        if (critical_point_idx == 0)
        {
          orunav_rviz::drawSphere(vehicle_state_.getCurrentState2d().getPose2d()[0],
                                  vehicle_state_.getCurrentState2d().getPose2d()[1],
                                  0.2,
                                  1,
                                  std::string("critical_point"),
                                  marker_pub5_);
        }
      }
    }
}


void KMOVehicleExecutionNode::publish_report(const ros::TimerEvent &event){
    orunav_msgs::RobotReport msg = vehicle_state_.getReport();
    msg.robot_id = robot_id_;
    report_pub_.publish(msg);
}

void KMOVehicleExecutionNode::process_velocity_constraints(const std_msgs::Float64MultiArrayConstPtr &msg){
    if (msg->data.size() != 4)
    {
      ROS_ERROR_STREAM("Wrong format on /velocity_constraints topic");
      return;
    }
    double max_linear_velocity_constraint = msg->data[0];
    double max_rotational_velocity_constraint = msg->data[1];
    double max_linear_velocity_constraint_rev = msg->data[2];
    double max_rotational_velocity_constraint_rev = msg->data[3];

    vehicle_state_.setNewVelocityConstraints(max_linear_velocity_constraint, max_rotational_velocity_constraint, max_linear_velocity_constraint_rev, max_rotational_velocity_constraint_rev);
    ROS_INFO_STREAM("New velocity constraint (fwd) [linear: " << msg->data[0] << " | rot: " << msg->data[1] << "] (rev) [linear: " << msg->data[2] << " | rot: " << msg->data[3] << "]");
    if (vehicle_state_.newVelocityConstraints())
    {
      cond_.notify_one();
    }
}

void KMOVehicleExecutionNode::process_report(const orunav_msgs::ControllerReportConstPtr &msg){

    orunav_generic::State2d state = createState2dFromControllerStateMsg(msg->state);

    inputs_mutex_.lock();

    //    controller_status_ = msg->status;
    bool completed_target;
    vehicle_state_.update(msg, completed_target, use_forks_);
    inputs_mutex_.unlock();

    // TODO check here ....
    // updateSafetyZones();

    // Separate thread / timer.
    if (vehicle_state_.getResendTrajectory())
    {
      ROS_ERROR("Resending the trajectory(!)");
      vehicle_state_.setResendTrajectory(false);
      cond_.notify_one();
    }

    if (completed_target)
    {
      // TODO: the vehicle coordinator is simplified and cannot queue up a list with tasks anymore
      // vehicle_state_.setDocking(false);
      ROS_INFO("[KMOVehicleExecutionNode] %d - target completed, time duration: %f", (int)robot_id_, ros::Time::now().toSec() - current_start_time_);
    }

    // Tracking performance check
    double tracking_error = 0.;
    if (msg->traj_values.size() > 0)
    {
      if (msg->traj_values[0].active && msg->traj_values[0].status == 1)
      {
        tracking_error = msg->traj_values[0].value;
      }
    }
    // TODO check here...
    // trackingErrorEBrake(tracking_error);
  }


void KMOVehicleExecutionNode::publishInitPath(const std::vector<orunav_msgs::PoseSteering>& path, double r, double g, double b, double a){
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();
    for(unsigned int i=0; i < path.size(); i++){
      geometry_msgs::PoseStamped next_pose;
      next_pose.pose.position.x = path[i].pose.position.x;
      next_pose.pose.position.y = path[i].pose.position.y;
      next_pose.pose.position.z = 0.0;
      gui_path.poses[i] = next_pose;
    }
    path_pub.publish(gui_path);
}


void KMOVehicleExecutionNode::publishSmoothPath(const orunav_generic::Path& path, double r, double g, double b, double a){
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.sizePath());
    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();
     for(int i=0; i< path.sizePath(); i++){
      geometry_msgs::PoseStamped stap;
      stap.pose.position.x = path.getPose2d(i)[0];
      stap.pose.position.y = path.getPose2d(i)[1];
      stap.pose.position.z = 0.0;
      gui_path.poses[i] = stap;
    }
    smooth_path_pub_.publish(gui_path);
}


void KMOVehicleExecutionNode::goalTrajectoryCallback(const geometry_msgs::PoseStampedConstPtr& msg1){
   
    orunav_generic::Path path;

    geometry_msgs::PoseStamped goal_;
    goal_.pose.position.x = msg1->pose.position.x;
    goal_.pose.position.y = msg1->pose.position.y;
    goal_.pose.position.z = 0.0;
    // TODO 
    goal_.pose.orientation.x = msg1->pose.orientation.x;
    goal_.pose.orientation.y = msg1->pose.orientation.y;
    goal_.pose.orientation.z = msg1->pose.orientation.z;
    goal_.pose.orientation.w = msg1->pose.orientation.w;

    orunav_generic::Path path_out;
    path_planner.getPathCB(current_pose, goal_, 0.0, 0.0, path_out);
    // publishInitPath(path_out, 1.0, 0.8, 0.3, 1);
    orunav_msgs::ComputeTaskStatus msg;
    if(path_out.sizePath() < 3){
      ROS_ERROR("[KMOVehicleExecutionNode] - Call to service get_path returns ERROR");
      // if (!resolve_motion_planning_error_)
      // {
        // res.result = 0;
        msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_SERVICE_FAILED;
        compute_status_pub_.publish(msg);
        // return false;
      // }
      // ROS_WARN("[KMOVehicleExecutionNode] RID:%d - no path found(!), will attempt to generate another path", robot_id_);
      //   // Are the start / goal very close?
      // if (getTargetStartGoalDistance(target) > sqrt(2) * map.info.resolution)
      // { // if the targets are reasonable apart
      //   // This indicates that there is some real problems finding the path...
      //   ROS_ERROR("[KMOVehicleExecutionNode] RID:%d - target and goal is to far appart, the motion planner should have found a path", robot_id_);
      //   // res.result = 0;
      //   msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_FAILED;
      //   compute_status_pub_.publish(msg);
      //   // return false;
      // }
      // // If they are, try to use the driven path (if any) to generate a repositioning path...
      // if (!getRepositioningPathMsgUsingDrivenPath(srv.response.path, target, vehicle_state_.getPathFromLastAssignedTask() /*getPath()*/,
      //                                             1.0))
      // {
      //   ROS_WARN("[KMOVehicleExecutionNode] RID:%d - failed to compute repositioning path", robot_id_);
      //   // res.result = 0;
      //   msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_REPOSITIONING_FAILED;
      //   compute_status_pub_.publish(msg);
      //   // return false;
      // }
      // ROS_INFO("[KMOVehicleExecutionNode] - computed repositioning path based on previous path");
    }
    if(path_out.sizePath()>3){
      ROS_INFO("[KMOVehicleExecutionNode] - get_path successful");
      msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_SERVICE_SUCCESS;
      compute_status_pub_.publish(msg);

      orunav_generic::makeValidPathForTrajectoryProcessing(path_out);
      path_out = orunav_generic::minIncrementalDistanceMinNbPointsPath(path_out, min_incr_path_dist_, min_nb_path_points_);
      ROS_INFO_STREAM("[KMOVehicleExecutionNode] - size of path : " << path_out.sizePath());


      // if (visualize_) {

        auto  start_pose2d = path_out.getPose2d(0);
        orunav_generic::Pose2d start_pose(start_pose2d[0], start_pose2d[1], start_pose2d[2]);
        auto  goal_pose2d = path_out.getPose2d(path_out.sizePath()-1);
        orunav_generic::Pose2d goal_pose(goal_pose2d[0], goal_pose2d[1], goal_pose2d[2]);
        orunav_rviz::drawPose2d(start_pose, 0, 0, 1., "start_pose2d", marker_pub_);
        orunav_rviz::drawPose2d(goal_pose, 0, 2, 1., "goal_pose2d", marker_pub_);
        orunav_rviz::drawPose2dContainer(orunav_generic::minIncrementalDistancePath(path_out, 0.2), "path_subsampled", 1, marker_pub_);
      // }

      path_smoother.smoothTraj(path_out, path, current_pose, goal_);
      orunav_rviz::drawPose2dContainer(orunav_generic::minIncrementalDistancePath(path, 0.2), "path_smoothed", 0, marker_pub_);      
      ROS_INFO("[KMOVehicleExecutionNode] - get_path successful");
      msg.status = orunav_msgs::ComputeTaskStatus::PATH_PLANNER_SERVICE_SUCCESS;
      compute_status_pub_.publish(msg);

      // // Remove duplicate points in the path.
      // orunav_generic::makeValidPathForTrajectoryProcessing(path);
      // // Make it less dense... important for the smoothing steps.
      // //    path = orunav_generic::minIncrementalDistancePath(path, min_incr_path_dist_);
      // path = orunav_generic::minIncrementalDistanceMinNbPointsPath(path, min_incr_path_dist_, min_nb_path_points_);
      // ROS_INFO_STREAM("[KMOVehicleExecutionNode] - size of path : " << path.sizePath());

      // Call the server...
      orunav_msgs::RobotTarget target;
      
      target.goal.pose = msg1->pose;
      target.goal.steering = 0.;
      target.goal_op.operation = target.goal_op.NO_OPERATION;
      target.start_op.operation = target.start_op.NO_OPERATION;

      ROS_INFO_STREAM("[PointNClickTargetClientNode] target.goal_op.operation : " << target.goal_op.operation); 
      ROS_INFO_STREAM("[PointNClickTargetClientNode] target.start_op.operation : " << target.start_op.operation); 
      
      target.robot_id = 1;
      target.task_id = 1;  // This will anyway be handled by the coordinator.
      target.goal_id = target.task_id;  // Obsolete?
      target.start_earliest = ros::Time::now();

      
      msg.task_id = target.task_id;
      msg.status = orunav_msgs::ComputeTaskStatus::COMPUTE_TASK_START;
      compute_status_pub_.publish(msg);

      ROS_INFO_STREAM("computeTaskCB taskRID: " << target.robot_id << " node RID:" << robot_id_);
      if (!validTarget(target))
      {
        ROS_WARN("[KMOVehicleExecutionNode] RID:%d - invalid target(!)", robot_id_);
        // res.result = 0;
        msg.status = orunav_msgs::ComputeTaskStatus::INVALID_TARGET;
        compute_status_pub_.publish(msg);
        // return false;
      }

      ROS_INFO("[KMOVehicleExecutionNode] RID:%d - received computeTask", robot_id_);
      // Do we have a map available? TODO - this should be possible to be sent from the coordination as well(!).
      if (!valid_map_)
      {
        ROS_WARN("[KMOVehicleExecutionNode] RID:%d - empty map(!), cannot computeTask", robot_id_);
        // res.result = 0;
        msg.status = orunav_msgs::ComputeTaskStatus::INVALID_MAP;
        compute_status_pub_.publish(msg);
        // return false;
      }

      bool start_from_current_state = true;
      if (start_from_current_state)
      {
        if (!vehicle_state_.validCurrentState2d())
        {
          ROS_WARN("[KMOVehicleExecutionNode] RID:%d - start from current state2d, current state2d unknown, cannot computeTask", robot_id_);
          // res.result = 0;
          msg.status = orunav_msgs::ComputeTaskStatus::INVALID_START;
          compute_status_pub_.publish(msg);
          // return false;
        }
        current_mutex_.lock();

        orunav_msgs::PoseSteering ps;
        auto  start_pose2d = path.getPose2d(0);
        ps.pose = createMsgFromPose2d(start_pose2d);
        ps.steering = path.getSteeringAngle(0);

        target.start = ps;
        target.start.steering = 0.;

        current_mutex_.unlock();
      }

      // orunav_msgs::RobotTarget target = target;
      map_mutex_.lock();
      // relevant if NOT use_vector_map_and_geofence_
      nav_msgs::OccupancyGrid map = current_map_;
      map_mutex_.unlock();



      

      orunav_msgs::ComputeTask::Response res;
      // Packet the message and return it.
      res.task.target = target; // Simply return the original target (not the one with modified goal / start poses).
      res.task.path = createPathMsgFromPathAndState2dInterface(path, createState2dFromPoseSteeringMsg(target.start)
                                                                          , createState2dFromPoseSteeringMsg(target.goal));
      // res.task.constraints = srv_constraints.response.constraints;
      // ROS_INFO_STREAM("KMO res.constraints.constraints_outer.size() : " << res.task.constraints.constraints_outer.size());
      // ROS_INFO_STREAM("KMO res.constraints.constraints.size() : " << res.task.constraints.constraints.size());

      // res.task.dts = dts;
      res.task.criticalPoint = -1;
      res.task.criticalRobotID = -1;
      res.result = 1;
      msg.status = orunav_msgs::ComputeTaskStatus::COMPUTE_TASK_SUCCESS;
      compute_status_pub_.publish(msg);

      std::cout<< "===================================n constrol==================================================" << std::endl;
      ROS_INFO("[CoordinatorFake] RID:%d - received setTask", (int)res.task.target.robot_id);

      // Verify the task
      if (!validTaskMsg(res.task)) {
        ROS_ERROR("[CoordinatorFake] invalid task.");
        // return false;
      }

      ROS_INFO("[CoordinatorFake] RID: 1");
      // Simply just call the execute directly using the same task.
      orunav_msgs::Task task_;
      task_ = res.task;


      // All items below needs to be set by the coordinator.
      task_.update = true;
      task_.target.task_id = 1;
      ROS_INFO("[CoordinatorFake] RID: 2");
    
      // if (use_ct_) {
      //   task_.cts.ts.push_back(computeCTSDirectlyWithoutDoingAnyCoordinationAtAll(res.task.dts));
      // }
      ROS_INFO("[CoordinatorFake] RID: 111");
      // This doesn't work...
      res.result = 1;
      ROS_INFO("[CoordinatorFake] RID: 1114");
      ROS_INFO("[CoordinatorFake] RID: 115");

      {
        // ros::ServiceClient client;
        // std::string service_name;
        // service_name = orunav_generic::getRobotTopicName(task_.target.robot_id, this->execute_task_name_);
        // ROS_INFO_STREAM("[CoordinatorFakeNode] - " << service_name << " call successful");
        // client = nh_.serviceClient<orunav_msgs::SetTask>(service_name);
        
        // orunav_msgs::SetTask srv;
        orunav_msgs::SetTask srv;
        srv.request.task = task_;

        // Check the current state of the vehicle
        ROS_INFO("[KMOVehicleExecutionNode] RID:%d [%d] - received executeTask (update:%d)", robot_id_, task_.target.robot_id, (int)task_.update);
        ROS_INFO("[KMOVehicleExecutionNode] RID:%d - next critical point (-1 == there is none) (:%d)", robot_id_, (int)task_.criticalPoint);
        //    ROS_ERROR_STREAM("task.cts : " << task_.cts);

        if (!vehicle_state_.isWaiting() && !task_.update)
        {
          ROS_WARN("[KMOVechileExecutionNode] : not in WAITING state(!) this TASK will be IGNORED");
          // return false;
        }
        
        if (task_.update && vehicle_state_.brakeSentUsingServiceCall()) {
            ROS_INFO("[KMOVehicleExecutionNode] - Update and execute task. Calling RECOVER.");
            // sendRecoverCommand(VehicleState::BrakeReason::SERVICE_CALL);
            vehicle_state_.setResendTrajectory(true);
        }

        // Any start operation?
        if (task_.target.start_op.operation != task_.target.start_op.NO_OPERATION)
        {
          ROS_WARN("start operation requested(!)");
        }
        ROS_INFO_STREAM( "executeTaskCB goal operation: " << task_.target.goal_op.operation );


        if (!validTask(task_))
        {
          ROS_WARN_STREAM("[KMOVehicleExecutionNode] not a valid task(!) - will be IGNORED");
          // return false;
        }

        inputs_mutex_.lock();
        vehicle_state_.update(task_);
        inputs_mutex_.unlock();
    }	    
    std::cout<< "======================================sdfsdfsdf" << std::endl; 

    ROS_INFO("[KMOVehicleExecutionNode] waking up, vehicle state : %s", vehicle_state_.getDebugString().c_str());
    // Need to be in WAIT state or ACTIVE state
    if (!vehicle_state_.canSendTrajectory())
    { // This handle the previous problem if many coordination times are sent at once (WAITING_TRAJECTORY_SENT)
      ROS_INFO("[KMOVehicleExecutionNode] - cannot send trajectory (wrong state)");
      usleep(100000);
      vehicle_state_.setResendTrajectory(true);
      // continue;
    }

    if (vehicle_state_.getCriticalPointIdx() >= 0 && vehicle_state_.getCriticalPointIdx() < 2)
    {
      ROS_WARN_STREAM("[KMOVehicleExecutionNode] critical point is too close to the starting pose... - will NOT start");
      // This is perfecly ok, AT_CRITICAL_POINT and WAITING_FOR_TASK is the same state but with different names. Make sure that the controller is waiting.
      if (vehicle_state_.isWaiting())
      {
        vehicle_state_.setAtCriticalPoint();
      }
      // continue;
    }
    if (vehicle_state_.getCurrentPathIdx() > 0)
    {
      if (vehicle_state_.getCriticalPointIdx() < vehicle_state_.getCurrentPathIdx() + 2)
      {
        ROS_WARN_STREAM("[KMOVehicleExecutionNode] critical point is too close to the starting pose... - will NOT start (path idx : " << vehicle_state_.getCurrentPathIdx() << ", crit point : " << vehicle_state_.getCriticalPointIdx() << ")");
        // continue;
      }
    }

    orunav_msgs::Task task = vehicle_state_.getTask();
    path = orunav_conversions::createPathFromPathMsg(task.path);

    orunav_generic::CoordinatedTimes cts;
    if (task.cts.ts.size() > 0)
      cts = orunav_conversions::getCoordinatedTimesFromCoordinatorTimeMsg(task.cts.ts[0]); // [0] -> this is still a vector (always lenght 1?) for old reasons
    if (cts_clear_first_entry_in_pairs_)
    {
      cts.clearFirstEntryInPairs();
      // for (size_t i = 0; i < cts.size(); i++) {
      //   std::cerr << "cts[" << i << "] : " << cts[i] << std::endl;
      // }
    }

    std::pair<unsigned int, orunav_generic::TrajectoryChunks> chunks_data;
    traj_params_.debugPrefix = std::string("coord_time/");

    ROS_INFO("--mutex lock--");
    inputs_mutex_.lock();
    current_constraints_path_ = path;

    // TODO - this will affect the order of the constraints, they are not currently used anyway.
    orunav_generic::makeValidPathForTrajectoryProcessing(path);
    if (!orunav_generic::validPathForTrajectoryProcessing(path))
    {
      ROS_ERROR("KMOVehicleExecution - INVALIDPATH quit(!) this SHOULD never happen");
      exit(-1);
    }
    else
    {
      ROS_INFO("VALIDPATH");
    }

    orunav_generic::savePathTextFile(path, "rid" + orunav_generic::toString(task.target.robot_id) + "_task" + orunav_generic::toString(task.target.task_id) + ".path");
    current_path_ = path;
    // current_constraints_ = orunav_conversions::createPolygonConstraintsVecFromRobotConstraintsMsg(task.constraints);
    // current_constraints_outer_ = orunav_conversions::createConvexOuterPolygonsFromRobotConstraintsMsg(task.constraints);
    current_target_ = task.target;
    current_cts_ = cts;
    inputs_mutex_.unlock();

    ///////////////////////////////////////////////////////////////////
    // CASE 1:
    ///////////////////////////////////////////////////////////////////
    if (vehicle_state_.vehicleStoppedAndTrajectoryNotCompleted() || (vehicle_state_.isWaitingTrajectoryEmpty() && vehicle_state_.hasActiveTaskCriticalPoint()))
    {
      // Waiting - need to ship a trajectory off from the current location with zero speed.
      ROS_INFO("[KMOVehicleExecutionNode] - CASE1");

      unsigned int path_idx;
      vehicle_state_.setPath(path);
      chunks_data = computeTrajectoryChunksCASE1(vehicle_state_, traj_params_, path_idx, use_ct_);
      if (chunks_data.second.empty())
      {
        ROS_WARN_STREAM("[KMOVehicleExecutionNode] CASE1 - couldn't compute trajectory, path size is to small... (path idx : " << path_idx << ", crit point : " << vehicle_state_.getCriticalPointIdx() << ")");
        // Move from "at critical point" to "waiting for task". Currently this is how the coordinator is aborting the current task.
        vehicle_state_.abortTask();
        // continue;
      }
      vehicle_state_.setCurrentPathIdx(path_idx);

      // Important the chunks will need to be re-indexed from 0.
      // It is currently only CASE2 which use on-the-fly modification of an active trajectory.
      vehicle_state_.clearTrajectoryChunkIdx();
    }

    ///////////////////////////////////////////////////////////////////
    // CASE 2:
    ///////////////////////////////////////////////////////////////////
    else if (vehicle_state_.isActive())
    {
      ROS_INFO("[KMOVehicleExecutionNode] - CASE2, need to send an updated trajectory for goalID %d", current_target_.goal_id);
      unsigned int path_idx;
      // idx when we can safely connect
      unsigned int chunk_idx = vehicle_state_.getCurrentTrajectoryChunkIdx() + chunk_idx_connect_offset_; // This is the earliest we can connect to.
      unsigned int chunk_idx_to_end_margin = 5;                                                           // After we connect we need some chunks to drive before the end.

      bool new_path_is_shorter = (path.sizePath() < vehicle_state_.getPath().sizePath());
      if (new_path_is_shorter)
      {
        chunk_idx_to_end_margin = 1;
        ROS_INFO_STREAM("[KMOVehicleExecution] - SIZE of paths: new:" << path.sizePath() << " old:" << vehicle_state_.getPath().sizePath() << " curretn IDX:" << vehicle_state_.getCurrentPathIdx());
      }
      if (new_path_is_shorter || vehicle_state_.isChunkIdxValid(chunk_idx + chunk_idx_to_end_margin))
      {
        // Two options
        // 1) the new path start is the same as the prev path start
        // 2) the new path start is the same as the prev path goal
        // In case of 1), we're all fine to continue...
        // In case of 2), we need to connect the provided path with the current path.
        ROS_INFO_STREAM("[KMOVehicleExecution] - (+3) chunk_idx: valid : " << chunk_idx);
        double path_chunk_distance;
        vehicle_state_.updatePath(path);
        if (use_ct_)
        {
          ROS_INFO_STREAM("Adding cts: " << cts.size());
          ROS_INFO_STREAM("Path size : " << path.sizePath());
          vehicle_state_.setCoordinatedTimes(cts);
        }
        if (vehicle_state_.isDrivingSlowdown())
        {
          ROS_INFO_STREAM("[KMOVehicleExecution] - will drive in slowdown mode - ignoring CTs");
          vehicle_state_.clearCoordinatedTimes();
          bool valid = false;
          chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_slowdown_params_, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
          if (!valid)
          {
            // continue;
          }
          // Special case - clear the CT's -> however, should check that the CTS are not slower than the slowdown...
        }
        else if (vehicle_state_.newVelocityConstraints())
        {
          ROS_INFO_STREAM("[KMOVehicleExecution] - got new velocity constraints");
          TrajectoryProcessor::Params traj_params = traj_params_original_;
    updateTrajParamsWithVelocityConstraints(traj_params, vehicle_state_);
          ROS_INFO_STREAM("new trajectory params: " << traj_params);

          // Overwrite the default velocity constratins
          if (overwrite_traj_params_with_velocity_constraints_)
          {
            traj_params_ = traj_params;
          }

          bool valid = false;
          chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_params, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
          if (!valid)
          {
            // continue;
          }
          vehicle_state_.resetNewVelocityConstraint();
        }
        else
        {
          bool valid;
          chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_params_, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
          if (!valid)
          {
            // continue;
          }
        }
        ROS_INFO_STREAM("[VehicleExecutionNode] - distance between the connected path state and the chunk_idx used : " << path_chunk_distance);
        // Check the distance > threshold (could also be to check what control input is required to bring it from state chunk to state path and to check if this is reasonable rather then a simple distance check...).
        vehicle_state_.setCurrentPathIdx(path_idx);
      }
      else
      {
        // The vehicle is simply to close to the final goal...
        // Instead of stopping, simply retry until we get into another state.
        ROS_INFO_STREAM("[KMOVehicleExecutionNode] - too close to the goal, will try again");
        usleep(1000000);
        vehicle_state_.setResendTrajectory(true);
        // continue;
      }
    }
    ///////////////////////////////////////////////////////////////////
    // CASE 3:
    ///////////////////////////////////////////////////////////////////
    else if (vehicle_state_.isWaitingTrajectoryEmpty())
    {
        ROS_INFO("[KMOVehicleExecutionNode] - CASE3, great - goalID is %d", current_target_.goal_id);
        // Is the start distance close enought with the current distance?
        double dist = orunav_generic::getDistBetween(vehicle_state_.getCurrentState2d().getPose2d(),
                                                      path.getPose2d(0));
        if (dist > 1.2)
        { // TODO param (this looks like a very magic number :-))
          ROS_WARN(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> [KMOVehicleExecutionNode] - really off start pose, will not try to connect to along a path in WAIT mode (%f)", dist);
          // continue;
        }

        vehicle_state_.setPath(path);
        if (use_ct_)
        {
          vehicle_state_.setCoordinatedTimes(cts);
        }
        ROS_INFO("Computing trajectory");
        if (vehicle_state_.newVelocityConstraints()) {
          ROS_INFO_STREAM("[KMOVehicleExecution] - got new velocity constraints");
          TrajectoryProcessor::Params traj_params = traj_params_original_;
          updateTrajParamsWithVelocityConstraints(traj_params, vehicle_state_);
          
          ROS_INFO_STREAM("new trajectory params: " << traj_params);
          
          // Overwrite the default velocity constratins
          if (overwrite_traj_params_with_velocity_constraints_) {
            traj_params_ = traj_params;
          }
          chunks_data = computeTrajectoryChunksCASE3(vehicle_state_, traj_params, use_ct_);
          vehicle_state_.resetNewVelocityConstraint();
        }
        else {
          chunks_data = computeTrajectoryChunksCASE3(vehicle_state_, traj_params_, use_ct_);
        }
        ROS_INFO("Computing trajectory - done");
        // Do we need to perform any operations at start? (note this is the only case when we should do any start operations...).
        if (vehicle_state_.performStartOperation())
        {
          usleep(100000);
          vehicle_state_.setResendTrajectory(true);
          // continue;
        }
    }

    ///////////////////////////////////////////////////////////////////
    // CASE 4:
    ///////////////////////////////////////////////////////////////////
    else if (vehicle_state_.brakeSentUsingServiceCall()) {
      ROS_INFO("[KMOVehicleExecutionNode] - CASE4, goalID is %d", current_target_.goal_id);
      ROS_ERROR("[KMOVehicleExecutionNode] - waiting the recovery will be triggered.");
      // continue;
    }
      
    ///////////////////////////////////////////////////////////////////
    // CASE 5:
    ///////////////////////////////////////////////////////////////////
    else
    {
      ROS_INFO("[KMOVehicleExecutionNode] - CASE5, goalID is %d", current_target_.goal_id);
      ROS_ERROR("[KMOVehicleExecutionNode] - in wrong STATE(!) - should never happen");
    }

    //-----------------------------------------------------------------
    vehicle_state_.activateTask();

    // Add a check whether to brake due to slow speeds.
    // If the speed after breaking gets high enough wake up / unbreak.

    ros::Time start_time;
    double start_time_d = vehicle_state_.getCoordinatedStartTime();
    if (start_time_d < 0)
    {
      start_time = ros::Time::now() + ros::Duration(0.5);
      if (use_ct_)
      {
        ROS_WARN("[KMOVehicleExecutionNode] - start time is not coordinated");
      }
    }else{
        start_time = ros::Time(start_time_d);
    }
    sendTrajectoryChunks(chunks_data);
    vehicle_state_.trajectorySent();
    if (!vehicle_state_.isActive()){
        sendActivateStartTimeCommand(start_time);
    }

      // current_target_.start = ps;
      // auto goal_pose2d = path.getPose2d(path.sizePath()-1);
      // ps.pose = createMsgFromPose2d(start_pose2d);
      // ps.steering = path.getSteeringAngle(path.sizePath()-1);
      // current_target_.goal = ps;
      // current_target_.start_op.operation = current_target_.start_op.NO_OPERATION;
      // current_target_.goal_op.operation = current_target_.goal_op.NO_OPERATION;
      // current_target_.start_earliest = ros::Time::now();
      // current_target_.goal_id = 1;
      // vehicle_state_.setPath(path);
      // vehicle_state_.
      // calculateCommands(path);
  }
}

void KMOVehicleExecutionNode::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom = *msg;
  current_pose.pose.position.x = odom.pose.pose.position.x;
  current_pose.pose.position.y = odom.pose.pose.position.y;
  current_pose.pose.position.z = 0.0;
  current_pose.pose.orientation.x = odom.pose.pose.orientation.x;
  current_pose.pose.orientation.y = odom.pose.pose.orientation.y;
  current_pose.pose.orientation.z = odom.pose.pose.orientation.z;
  current_pose.pose.orientation.w = odom.pose.pose.orientation.w;
}

void KMOVehicleExecutionNode::updateTrajParamsWithVelocityConstraints(TrajectoryProcessor::Params &traj_params, const VehicleState &vehicle_state)  {
  traj_params.maxVel = std::min(traj_params.maxVel, vehicle_state.getMaxLinearVelocityConstraint());
  traj_params.maxVelRev = std::min(traj_params.maxVelRev, vehicle_state.getMaxLinearVelocityConstraintRev());
  traj_params.maxRotationalVel = std::min(traj_params.maxRotationalVel, vehicle_state.getMaxRotationalVelocityConstraint());
  traj_params.maxRotationalVelRev = std::min(traj_params.maxRotationalVelRev, vehicle_state.getMaxRotationalVelocityConstraintRev());
  
  traj_params.maxVel = std::max(traj_params.maxVel, 0.01); // Always allow to drive faster than 1 cm /s.
  traj_params.maxVelRev = std::max(traj_params.maxVelRev, 0.01);
  traj_params.maxRotationalVel = std::max(traj_params.maxRotationalVel, 0.01); // Alway allow to rotate more than 0.01 rad / s.
  traj_params.maxRotationalVelRev = std::max(traj_params.maxRotationalVelRev, 0.01);
}
  
void KMOVehicleExecutionNode::drawPointCloud(const sensor_msgs::PointCloud &points, const std::string &name, int id, int color, double scale, ros::Publisher &pub)
{

  if (points.points.size() == 0)
    return;
  visualization_msgs::Marker m;
  // orunav_rviz::assignDefault(m);
  // orunav_rviz::assignColor(m, color);
  m.scale.x = scale;
  m.scale.y = scale;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.ns = name + orunav_generic::toString(id);
  m.id = id;

  geometry_msgs::Point p;
  p.z = 0.;

  for (unsigned int i = 0; i < points.points.size(); i++)
  {
    p.x = points.points[i].x;
    p.y = points.points[i].y;
    m.points.push_back(p);
  }
  pub.publish(m);
}


  KMOVehicleExecutionNode::KMOVehicleExecutionNode()
  {
    b_shutdown_ = true;
    cond_.notify_one();
    client_thread_.join();
  }

  // This takes care of computing/updating the reference trajectory followed by the controller node.
  void KMOVehicleExecutionNode::run()
  {
    while (!b_shutdown_)
    {
     
    } // while
    ROS_ERROR("[KMOVehicleExecutionNode] trajectory update thread - died(!) : %s", vehicle_state_.getDebugString().c_str());
  }


 
  void KMOVehicleExecutionNode::sendActivateStartTimeCommand(const ros::Time &startTime)
  {

    if (vehicle_state_.canSendActivate())
    {
      orunav_msgs::ControllerCommand command;
      command.robot_id = robot_id_;
      command.traj_id = 0;
      command.command = command.COMMAND_ACTIVATE;
      command_pub_.publish(command);

      usleep(5000);

      command.command = command.COMMAND_STARTTIME;
      command.start_time = startTime;

      if (command.start_time < ros::Time::now())
      {
        ROS_WARN("[KMOVehicleExecutionNode] - command start time < current time : %f secs.", (ros::Time::now() - command.start_time).toSec());
        ROS_WARN_STREAM("[KMOVehicleExecutionNode] - current time: " << ros::Time::now().toSec()
                                                                     << " old command start time: " << command.start_time.toSec());
        //exit(-1); // Kill it! -> should not happen.
        command.start_time = ros::Time::now();
        ROS_WARN_STREAM("[KMOVehicleExecutionNode] - new command start time: " << command.start_time.toSec());
      }
      vehicle_state_.setTrajectoryChunksStartTime((command.start_time).toSec());
      current_start_time_ = command.start_time.toSec();

      ROS_INFO("[KMOVehicleExecutionNode] - command start time : %f", command.start_time.toSec());
      command_pub_.publish(command);

      usleep(5000);
    }
    ROS_INFO("[KMOVehicleExecutionNode] done sending controller command");
  }

void KMOVehicleExecutionNode::sendTrajectoryChunks(const std::pair<unsigned int, orunav_generic::TrajectoryChunks> &chunks_data)
  {

    // Add the chunks
    inputs_mutex_.lock();
    vehicle_state_.appendTrajectoryChunks(chunks_data.first, chunks_data.second);
    vehicle_state_.saveCurrentTrajectoryChunks("current_chunks.txt");
    saveTrajectoryChunksTextFile(chunks_data.second, "chunks_data_to_be_added.txt");
    // Make sure that the ones that are in vehicle state are the ones that are sent to the controller....
    orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
    ROS_INFO_STREAM("[KMOVehicleExecutionNode] - appended chunks, index: " << chunks_data.first);
    ROS_INFO("[KMOVehicleExecutionNode] - appended chunks, size : %lu", chunks.size());
    inputs_mutex_.unlock();

    // -----------------------------------------------------------------
    // Mutex off - only local variables from now on in the communication
    // -----------------------------------------------------------------
    // waitForController();

    std::vector<double> A0, A1, b;

    orunav_msgs::ControllerTrajectoryChunkVec c_vec;
    // Send them off
    for (unsigned int i = chunks_data.first; i < chunks.size(); i++)
    {
      orunav_msgs::ControllerTrajectoryChunk c = createControllerTrajectoryChunkFromTrajectoryInterface(chunks[i]);

      c.robot_id = robot_id_;
      c.traj_id = 0;
      c.sequence_num = i;
      c.final = false;
      if (i == chunks.size() - 1)
        c.final = true;
      c_vec.chunks.push_back(c);
    }

    trajectorychunk_pub_.publish(c_vec);
    ROS_INFO("[KMOVehicleExecutionNode] - sending chunk size: %lu", c_vec.chunks.size());
  }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "orunav_vehicle_execution_node");
  ros::NodeHandle params("~");

  KMOVehicleExecutionNode ve(params);

  ros::spin();
}
