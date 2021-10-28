#include <orunav_vehicle_execution/path_planner.h>


  void PathPlannerService::init(ros::NodeHandle param_nh) 
  {
      // read parameters
      param_nh.param<std::string>("motion_primitives_directory", motion_prim_dir_, "./Primitives/");
      param_nh.param<std::string>("lookup_tables_directory", lookup_tables_dir_, "./LookupTables/");
      param_nh.param<std::string>("maps_directory", maps_dir_, "./");
      std::string model;
      param_nh.param<std::string>("model", model, "");
      param_nh.param<double>("min_incr_path_dist", min_incr_path_dist_, 0.001);
      param_nh.param<bool>("save_paths", save_paths_, false);

      WP::setPrimitivesDir(motion_prim_dir_);
      WP::setTablesDir(lookup_tables_dir_);
      WP::setMapsDir(maps_dir_);
      car_model_ = new CarModel(model);

      map_sub = param_nh.subscribe<nav_msgs::OccupancyGrid>("/map",10, &PathPlannerService::process_map, this);
      
      ROS_INFO_STREAM("[GetPathService] - Using model : " << model << "\n");
      param_nh.param<bool>("visualize",visualize_, false);
  }

  void PathPlannerService::process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    current_map = *msg;
    got_map = true;
    std::cout << "... got a map!" << std::endl;
  }


  PathPlannerService::~PathPlannerService(){
      delete car_model_;
      ROS_INFO_STREAM("[GetPathService] - shutting down\n");
  }

void PathPlannerService::convertNavMsgsOccupancyGridToWorldOccupancyMapRef(const nav_msgs::OccupancyGrid& msg, WorldOccupancyMap &map){
  double granularity = msg.info.resolution;

  int xcells = msg.info.width;
  int ycells = msg.info.height;
  
  std::vector<std::vector<double> > occupancyMap;
  occupancyMap.resize(msg.info.height);
  
  unsigned int k = 0;
  for (unsigned int i = 0; i < msg.info.height; i++){
      occupancyMap[i].resize(msg.info.width);
      for (unsigned int j = 0; j < msg.info.width; j++){
        occupancyMap[i][j] = msg.data[k]*0.01;
        k++;
      }
  }
  map.initialize(xcells, ycells, granularity, occupancyMap);
}

orunav_generic::Pose2d PathPlannerService::createPose2dFromMsg(const geometry_msgs::Pose& pose){
      orunav_generic::Pose2d p;
      p(0) = pose.position.x;
      p(1) = pose.position.y;
      p(2) = tf::getYaw(pose.orientation);
      return p;
}

orunav_generic::Pose2d PathPlannerService::getNavMsgsOccupancyGridOffsetRef(const nav_msgs::OccupancyGrid &msg) {
  return createPose2dFromMsg(msg.info.origin);
}

bool PathPlannerService::getPathCB(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, double start_steering, double goal_steering, orunav_generic::Path& path_out){

    ROS_INFO("[GetPathService] - start server : [%f,%f,%f](%f)", start.pose.position.x, start.pose.position.y,tf::getYaw(start.pose.orientation), start_steering);
    ROS_INFO("[GetPathService] - goal :  [%f,%f,%f](%f)", goal.pose.position.x, goal.pose.position.y,tf::getYaw(goal.pose.orientation), goal_steering);
    
    WorldOccupancyMap planner_map;
    convertNavMsgsOccupancyGridToWorldOccupancyMapRef(current_map, planner_map);

    orunav_generic::Pose2d map_pose_offset = getNavMsgsOccupancyGridOffsetRef(current_map);
    double map_offset_x = map_pose_offset(0);
    double map_offset_y = map_pose_offset(1);
    double start_orientation = tf::getYaw(start.pose.orientation);
    double goal_orientation = tf::getYaw(goal.pose.orientation);
 
    assert(fabs(map_pose_offset(2) < 0.001)); // The orientation should be the same... implement this whenever needed. 
    
    if (planner_map.getMap().empty()) {
      ROS_ERROR("[GetPathService] - error in the provided map / conversion");
      return false;
    }

    PathFinder* pf;
    if (planner_map.getMap().empty()){
      pf = new PathFinder(20, 20);
    }
    else{
      std::cout<< "Getting from the real map -------------->" << std::endl;
      pf = new PathFinder(planner_map);
    }
    
    max_planning_time = 100.0;
    if (max_planning_time > 0.) 
      pf->setTimeBound(max_planning_time);
    
    std::cout<< "=========================before=======================" << start.pose.position.x << "," << map_offset_x << "," << start.pose.position.y << "," << map_offset_y << std::endl;
    VehicleMission vm(car_model_, start.pose.position.x-map_offset_x, start.pose.position.y-map_offset_y, start_orientation, start_steering, goal.pose.position.x-map_offset_x
                                                                    , goal.pose.position.y-map_offset_y, goal_orientation, goal_steering);
    
    pf->addMission(&vm);
    if (max_planning_time > 0)
      pf->setTimeBound(max_planning_time);
    
    ROS_INFO("[GetPathService] - Starting to solve the path planning problem ... ");
    ros::Time start_time = ros::Time::now();
    std::vector<std::vector<Configuration*> > solution = pf->solve(false);
    ros::Time stop_time = ros::Time::now();
    ROS_INFO("[GetPathService] - Starting to solve the path planning problem - done");
    ROS_INFO("[GetPathService] - PATHPLANNER_PROCESSING_TIME: %f", (stop_time-start_time).toSec());
    
    assert(!solution.empty());
    bool solution_found = (solution[0].size() != 0);
    ROS_INFO_STREAM("[GetPathService] - solution_found : " << solution_found);
    ROS_INFO_STREAM("[GetPathService] - solution[0].size() : " << solution[0].size());
    
    orunav_generic::Path path;

    for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++)
    {
      for (std::vector<Configuration*>::iterator confit = (*it).begin(); confit != (*it).end(); confit++) {
        std::vector<vehicleSimplePoint> path_local = (*confit)->getTrajectory();
        for (std::vector<vehicleSimplePoint>::iterator it2 = path_local.begin(); it2 != path_local.end(); it2++) {
          double orientation = it2->orient;  
          
          orunav_generic::State2d state(orunav_generic::Pose2d(it2->x+map_offset_x,
                                                               it2->y+map_offset_y,
                                                               orientation), it2->steering);
          path.addState2dInterface(state);
        }
      }
    }

    ROS_INFO_STREAM("[GetPathService] - Nb of path points : " << path.sizePath());

    // Cleanup
    delete pf;
    for (std::vector<std::vector<Configuration*> >::iterator it = solution.begin(); it != solution.end(); it++) {
      std::vector<Configuration*> confs = (*it);
      for (std::vector<Configuration*>::iterator confit = confs.begin(); confit != confs.end(); confit++) {
        delete *confit;
      }
      confs.clear();
    }
    solution.clear();
    // Cleanup - end
    
    if (path.sizePath() == 0)
      solution_found = false;

    if (solution_found) {
      // First requirement (that the points are separated by a minimum distance).
      // orunav_generic::Path path_min_dist = orunav_generic::minIncrementalDistancePath(path, min_incr_path_dist_);
      // Second requirment (path states are not allowed to change direction of motion without any intermediate points).
      // orunav_generic::Path path_dir_change = orunav_generic::minIntermediateDirPathPoints(path_min_dist);
      // createPathMsgFromPathInterface(path_dir_change, path_out);
      // if (visualize_) {
      //   orunav_generic::Pose2d start_pose(start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation));
      //   orunav_generic::Pose2d goal_pose(goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));
      //   orunav_rviz::drawPose2d(start_pose, 0, 0, 1., "start_pose2d", marker_pub_);
      //   orunav_rviz::drawPose2d(goal_pose, 0, 2, 1., "goal_pose2d", marker_pub_);
      //   orunav_rviz::drawPose2dContainer(orunav_generic::minIncrementalDistancePath(path_dir_change, 0.2), "path_subsampled", 1, marker_pub_);
      // }
      // if (save_paths_) {
      //   orunav_generic::Path path = orunav_conversions::createPathFromPathMsg(res.path);
      //   std::stringstream st;
      //   st << "path_" << res.path.robot_id << "-" << res.path.goal_id << ".path";
      //   std::string fn = st.str();
        
      //   orunav_generic::savePathTextFile(path, fn);
      // }
      path_out = path;
      
      return true;
    }
   
    return false; 
  }

  void PathPlannerService::createPathMsgFromPathInterface(const orunav_generic::PathInterface& path_in, std::vector<orunav_msgs::PoseSteering>& path_out){
    for (size_t i = 0; i < path_in.sizePath(); i++){
        orunav_msgs::PoseSteering ps;
        // geometry_msgs::PoseStamped ps;
        ps.pose.orientation = tf::createQuaternionMsgFromYaw(path_in.getPose2d(i)(2));
        ps.pose.position.x = path_in.getPose2d(i)(0);
        ps.pose.position.y = path_in.getPose2d(i)(1);
        ps.pose.position.z = 0.;
        ps.steering = path_in.getSteeringAngle(i);
        path_out.push_back(ps);
    }
  }


  