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
    paramHandle.param<bool>("visualize", visualize_, false);
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



    goal_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 50, &KMOVehicleExecutionNode::goalTrajectoryCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 50, &KMOVehicleExecutionNode::odomCallback, this);
    smooth_path_pub_ = nh_.advertise<nav_msgs::Path>("/move_base/hagen/smoothpath", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("/move_base/hagen/init_path", 1);
    
    path_planner.init(nh_);
    path_smoother.init(nh_);

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


void KMOVehicleExecutionNode::goalTrajectoryCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& msg){
   
    orunav_generic::Path current_smoothed_path;

    geometry_msgs::PoseStamped goal_;
    goal_.pose.position.x = msg->goal.target_pose.pose.position.x;
    goal_.pose.position.y = msg->goal.target_pose.pose.position.y;
    goal_.pose.position.z = 0.0;
    // TODO 
    goal_.pose.orientation.x = msg->goal.target_pose.pose.orientation.x;
    goal_.pose.orientation.y = msg->goal.target_pose.pose.orientation.y;
    goal_.pose.orientation.z = msg->goal.target_pose.pose.orientation.z;
    goal_.pose.orientation.w = msg->goal.target_pose.pose.orientation.w;

    std::vector<orunav_msgs::PoseSteering> path_out;
    path_planner.getPathCB(current_pose, goal_, 0.0, 0.0, path_out);
    publishInitPath(path_out, 1.0, 0.8, 0.3, 1);
    if(path_out.size()>3){
      path_smoother.smoothTraj(path_out, current_smoothed_path, current_pose, goal_);
      publishSmoothPath(current_smoothed_path, 0.3, 0.5, 0.8, 1);
      vehicle_state_.setPath(current_smoothed_path);
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


  void KMOVehicleExecutionNode::calculateCommands(orunav_generic::Path& path){
   
    // unsigned int path_idx;
    // unsigned int chunk_idx = vehicle_state_.getCurrentTrajectoryChunkIdx() + chunk_idx_connect_offset_; // This is the earliest we can connect to.
    // unsigned int chunk_idx_to_end_margin = 5;                                                           // After we connect we need some chunks to drive before the end.
    // bool new_path_is_shorter = (path.sizePath() < vehicle_state_.getPath().sizePath());
    // if (new_path_is_shorter)
    // {
    //   chunk_idx_to_end_margin = 1;
    //   ROS_INFO_STREAM("[KMOVehicleExecution] - SIZE of paths: new:" << path.sizePath() << " old:" << vehicle_state_.getPath().sizePath() << " curretn IDX:" << vehicle_state_.getCurrentPathIdx());
    // }
    // chunks_data = computeTrajectoryChunksCASE1(vehicle_state_, traj_params_, path_idx, use_ct_);
    // vehicle_state_.setCurrentPathIdx(path_idx);
    while(true){
      if (vehicle_state_.getCurrentPathIdx() > 0)
      {
        if (vehicle_state_.getCriticalPointIdx() < vehicle_state_.getCurrentPathIdx() + 2)
        {
          ROS_WARN_STREAM("[KMOVehicleExecutionNode] critical point is too close to the starting pose... - will NOT start (path idx : " << vehicle_state_.getCurrentPathIdx() << ", crit point : " << vehicle_state_.getCriticalPointIdx() << ")");
          continue;
        }
      }

      orunav_msgs::Task task = vehicle_state_.getTask();
      orunav_generic::CoordinatedTimes cts;
      if (task.cts.ts.size() > 0)
        cts = getCoordinatedTimesFromCoordinatorTimeMsg(task.cts.ts[0]); // [0] -> this is still a vector (always lenght 1?) for old reasons
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
      makeValidPathForTrajectoryProcessing(path);

      if (!orunav_generic::validPathForTrajectoryProcessing(path))
      {
        ROS_ERROR("KMOVehicleExecution - INVALIDPATH quit(!) this SHOULD never happen");
        exit(-1);
      }
      else
      {
        ROS_INFO("VALIDPATH");
      }

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
          continue;
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
              continue;
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
              continue;
            }
            vehicle_state_.resetNewVelocityConstraint();
          }
          else
          {
            bool valid;
            chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_params_, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
            if (!valid)
            {
              continue;
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
          continue;
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
          continue;
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
              continue;
            }
      }
      ///////////////////////////////////////////////////////////////////
      // CASE 4:
      ///////////////////////////////////////////////////////////////////
      else if (vehicle_state_.brakeSentUsingServiceCall()) {
          ROS_INFO("[KMOVehicleExecutionNode] - CASE4, goalID is %d", current_target_.goal_id);
          ROS_ERROR("[KMOVehicleExecutionNode] - waiting the recovery will be triggered.");
          continue;
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
      }
      else
      {
        start_time = ros::Time(start_time_d);
      }
      sendTrajectoryChunks(chunks_data);
      vehicle_state_.trajectorySent();
      if (!vehicle_state_.isActive())
      {
        sendActivateStartTimeCommand(start_time);
      }
    }
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
