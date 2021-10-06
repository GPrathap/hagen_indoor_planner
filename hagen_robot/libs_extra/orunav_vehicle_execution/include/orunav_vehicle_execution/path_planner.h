#ifndef PATH_PLANNER_APA
#define PATH_PLANNER_APA

#include <ros/ros.h>
#include <tf/tf.h>
#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>

#include <orunav_msgs/GetPath.h>
#include <orunav_motion_planner/PathFinder.h>
#include <orunav_motion_planner/VehicleMission.h>
#include <iostream>
#include <string>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

class PathPlannerService
{
  private:
    std::string motion_prim_dir_;
    std::string lookup_tables_dir_;
    std::string maps_dir_;
    CarModel* car_model_;
    
    bool visualize_;
    
    ros::Publisher marker_pub_, path_pub;
    ros::Subscriber map_sub;
    double min_incr_path_dist_;
    bool save_paths_;

    ros::NodeHandle nh_;
    ros::ServiceServer service_;

    nav_msgs::OccupancyGrid current_map;
    bool got_map = false;  
    double max_planning_time = 20.; 

 public:

    PathPlannerService()= default;
    void init(ros::NodeHandle param_nh);
    void process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    ~PathPlannerService();
    bool getPathCB(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, double start_steering, double goal_steering, std::vector<orunav_msgs::PoseSteering>& path_out);
    void convertNavMsgsOccupancyGridToWorldOccupancyMapRef(const nav_msgs::OccupancyGrid& msg, WorldOccupancyMap &map);
    orunav_generic::Pose2d getNavMsgsOccupancyGridOffsetRef(const nav_msgs::OccupancyGrid &msg);
    orunav_generic::Pose2d createPose2dFromMsg(const geometry_msgs::Pose& pose);
    void createPathMsgFromPathInterface(const orunav_generic::PathInterface& path_in, std::vector<orunav_msgs::PoseSteering>& path_out);
};

#endif 