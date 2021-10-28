#include <orunav_vehicle_execution/hagen_vehicle_execution_node.h>

KMOVehicleExecutionNode::KMOVehicleExecutionNode(ros::NodeHandle &paramHandle){
    bool use_arm;
    // Parameters
    paramHandle.param<int>("robot_id", robot_id_, 1);
    {
      std::vector<int> robot_ids;
      robot_ids.push_back(robot_id_);
      // target_handler_.setRobotIDsToCompute(robot_ids);
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
   
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 50, &KMOVehicleExecutionNode::goalTrajectoryCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 50, &KMOVehicleExecutionNode::odomCallback, this);
    smooth_path_pub_ = nh_.advertise<nav_msgs::Path>("/move_base/hagen/smoothpath", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("/move_base/hagen/init_path", 1);
    control_report_sub_ = nh_.subscribe<orunav_msgs::ControllerReport>("control/controller/reports", 10, &KMOVehicleExecutionNode::process_report, this);
    velocity_constraints_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>(orunav_generic::getRobotTopicName(robot_id_, "/velocity_constraints"), 1, &KMOVehicleExecutionNode::process_velocity_constraints, this);
    report_pub_ = nh_.advertise<orunav_msgs::RobotReport>("control/report", 1);
    smooothed_pub_ = nh_.advertise<nav_msgs::Path>("/move_base/NavfnROS/plan", 1);
    update_goal_sub_ = nh_.subscribe<nav_msgs::Path>("/init_plan/plan", 1, &KMOVehicleExecutionNode::processPlan, this);
    heartbeat_report_pub_ = nh_.createTimer(ros::Duration(1.0), &KMOVehicleExecutionNode::publish_report, this);
   
    path_smoother.init(paramHandle);
    path_planner.init(paramHandle);

    solver_thread = startPathPlannerThread();

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
      orunav_rviz::drawPathInterface(path, "current_path_subsampled", 0, traj_params_.wheelBaseX, marker_pub_);
      orunav_rviz::drawPoint2dContainerAsConnectedLineIncZ_(orunav_generic::createPoint2dVecFromPose2dContainerInterface(path), "path_points", 1, 1, 0., 0., false, marker_pub_);

      if (current_cts_.size() == current_path_.sizePath() && vehicle_state_.isDriving())
      {
        orunav_rviz::drawCoordinatedTimes(current_path_, current_cts_, current_start_time_, "cts", 2, time_to_meter_factor_, 0.12, marker_pub_);
        orunav_rviz::drawPose2dTimesAsLines(current_path_, current_cts_, current_start_time_, "cts_lines", robot_id_, time_to_meter_factor_, 0.02, marker_pub_);
      }
    }

    // The trajectory
    orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
    orunav_rviz::drawTrajectoryChunksWithControl(chunks, 2, "chunks", marker_pub_);

    // Draw the trajetory as a path / cts, each pose2d is separated with 60 ms.
    orunav_rviz::drawTrajectoryChunksUsingTime(chunks, 2, "chunks_dt", current_start_time_, time_to_meter_factor_, traj_params_.timeStep, marker_pub_);

    if (!visualize_sweep_and_constraints_)
      return;

    // The sweep area
    int current_path_idx = vehicle_state_.getCurrentPathIdx();
  }

std::thread KMOVehicleExecutionNode::startPathPlannerThread() {
    return std::thread([&] { solverThread(); });
}


void KMOVehicleExecutionNode::processPlan(const nav_msgs::PathConstPtr& msg1){
    temp_path = *msg1;
    if(!is_goal_set){
      ROS_ERROR("[KMOVehicleExecutionNode] - Still no goal pose is set...");
      return;
    }
    if(!granted_execution){
      std::lock_guard<std::mutex> lk(lock_on_solver);
      granted_execution = true;
      condition_on_solver.notify_one();
    }else{
       ROS_ERROR("[KMOVehicleExecutionNode] - Still calculating older path");
    }
}


void KMOVehicleExecutionNode::solverThread() {
  while(true){
    std::cout<< "Waiting to start execution of solver..." << std::endl;
    std::unique_lock<std::mutex> lk(lock_on_solver);
    condition_on_solver.wait(lk, [&]{return granted_execution;});
    std::cout<< "Starting the solver thread..." << std::endl;

    orunav_generic::Path path;
    orunav_generic::Path path_out;
    path_planner.getPathCB(current_pose, goal_pose, 0.0, 0.0, path_out);
    if(path_out.sizePath() < 3){
      ROS_ERROR("[KMOVehicleExecutionNode] - Call to service get_path returns ERROR");
      have_trajector_ = false;
    } else {
      orunav_generic::makeValidPathForTrajectoryProcessing(path_out);
      path_out = orunav_generic::minIncrementalDistanceMinNbPointsPath(path_out, min_incr_path_dist_, min_nb_path_points_);
      ROS_INFO_STREAM("[KMOVehicleExecutionNode] - size of path : " << path_out.sizePath());

      auto  start_pose2d = path_out.getPose2d(0);
      std::cout<< "====================== path in: start: "<< start_pose2d.transpose() << std::endl;
      orunav_generic::Pose2d start_pose(start_pose2d[0], start_pose2d[1], start_pose2d[2]);
      auto  goal_pose2d = path_out.getPose2d(path_out.sizePath()-1);
      orunav_generic::Pose2d goal_pose_(goal_pose2d[0], goal_pose2d[1], goal_pose2d[2]);
      orunav_rviz::drawPose2d(start_pose, 0, 0, 1., "start_pose2d", marker_pub_);
      orunav_rviz::drawPose2d(goal_pose_, 0, 2, 1., "goal_pose2d", marker_pub_);
      orunav_rviz::drawPose2dContainer(orunav_generic::minIncrementalDistancePath(path_out, 0.2), "path_subsampled", 1, marker_pub_);

      path_smoother.smoothTraj(path_out, path, current_pose, goal_pose);
      orunav_rviz::drawPose2dContainer(orunav_generic::minIncrementalDistancePath(path, 0.2), "path_smoothed", 0, marker_pub_); 
      publishGlobalPath(path, 0.3, 0.6, 0.9, 1);    
      ROS_INFO("[KMOVehicleExecutionNode] - get_path successful");
      have_trajector_ = true;
    }
    std::cout<< "Solver is finished..." << std::endl;
    granted_execution = false;
    lk.unlock();
    condition_on_solver.notify_all();
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

void KMOVehicleExecutionNode::publishGlobalPath(const orunav_generic::Path& path, double r, double g, double b, double a){
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
    smooothed_pub_.publish(gui_path);
}


void KMOVehicleExecutionNode::goalTrajectoryCallback(const geometry_msgs::PoseStampedConstPtr& msg1){
    goal_pose.pose.position.x = msg1->pose.position.x;
    goal_pose.pose.position.y = msg1->pose.position.y;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = msg1->pose.orientation.x;
    goal_pose.pose.orientation.y = msg1->pose.orientation.y;
    goal_pose.pose.orientation.z = msg1->pose.orientation.z;
    goal_pose.pose.orientation.w = msg1->pose.orientation.w;
    is_goal_set = true; 
}

void KMOVehicleExecutionNode::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    odom = *msg;
    // current_pose.pose.position.x = odom.pose.pose.position.x;
    // current_pose.pose.position.y = odom.pose.pose.position.y;
    // current_pose.pose.position.z = 0.0;
    // current_pose.pose.orientation.x = odom.pose.pose.orientation.x;
    // current_pose.pose.orientation.y = odom.pose.pose.orientation.y;
    // current_pose.pose.orientation.z = odom.pose.pose.orientation.z;
    // current_pose.pose.orientation.w = odom.pose.pose.orientation.w;
    // std::cout<< "---------" << current_pose.pose.position.x << "," << current_pose.pose.position.y << std::endl;
    //  std::cout<< "=========================before=odom======================" << current_pose.pose.position.x  << "," << current_pose.pose.position.y << std::endl;
    try{
      
      tf2_ros::TransformListener tfListener(tfBuffer);
      transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
      current_pose.pose.position.x = transformStamped.transform.translation.x;
      current_pose.pose.position.y = transformStamped.transform.translation.y;
      current_pose.pose.position.z = 0.0;
      current_pose.pose.orientation.x = transformStamped.transform.rotation.x;
      current_pose.pose.orientation.y = transformStamped.transform.rotation.y;
      current_pose.pose.orientation.z = transformStamped.transform.rotation.z;
      current_pose.pose.orientation.w = transformStamped.transform.rotation.w;
      // std::cout<< "=========================before=odom======================" << current_pose.pose.position.x  << "," << current_pose.pose.position.y << std::endl;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
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

  ros::init(argc, argv, "hagen_vehicle_execution_node");
  ros::NodeHandle params("~");

  KMOVehicleExecutionNode ve(params);

  ros::spin();
}
