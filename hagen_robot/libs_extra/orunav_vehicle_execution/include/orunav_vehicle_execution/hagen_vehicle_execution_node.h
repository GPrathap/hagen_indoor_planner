#ifndef TRAJECTORY_EXECUTION_NODE
#define TRAJECTORY_EXECUTION_NODE

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
// #include <orunav_rviz/orunav_rviz.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <orunav_geometry/geometry.h>
#include <orunav_geometry/robot_model_2d.h>

#include <orunav_msgs/ComputeTask.h>
#include <orunav_msgs/UpdateTask.h>
#include <orunav_msgs/ExecuteTask.h>

#include <orunav_msgs/GetPath.h>
#include <orunav_msgs/GetGeoFence.h>
#include <orunav_msgs/GetVectorMap.h>
#include <orunav_msgs/GetPolygonConstraints.h>
#include <orunav_msgs/GetSmoothedPath.h>
#include <orunav_msgs/GetSmoothedStraightPath.h>
#include <orunav_msgs/GetDeltaTVec.h>
#include <orunav_msgs/ObjectPoseEstimation.h>
#include <orunav_msgs/ComputeTaskStatus.h>
#include <orunav_msgs/SetTask.h>

#include <orunav_msgs/ControllerTrajectoryChunkVec.h>
#include <orunav_msgs/ControllerTrajectoryChunk.h>
#include <orunav_msgs/ControllerCommand.h>
#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/RobotTarget.h>
#include <orunav_msgs/ForkReport.h>
#include <orunav_msgs/ForkCommand.h>
#include <orunav_msgs/RobotReport.h>
#include <orunav_msgs/ObjectPose.h>
#include <orunav_msgs/VectorMap.h>
#include <orunav_msgs/GeoFence.h>
#include <orunav_msgs/EBrake.h>
#include <orunav_msgs/BrakeTask.h>

// #include <orunav_constraint_extract/polygon_constraint.h> // Only used for visualization.
// #include <orunav_constraint_extract/conversions.h>
// #include <orunav_constraint_extract/grid_map.h>
// #include <orunav_constraint_extract/utils.h>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp> // for std::pair
#include <iostream>
#include <fstream>

// #include <orunav_vehicle_execution/pallet_handling_utils.h>
#include <orunav_vehicle_execution/vehicle_state.h>
#include <orunav_vehicle_execution/trajectory_generation.h>
#include <orunav_vehicle_execution/trajectory_processor_naive_ct.h>
#include <orunav_vehicle_execution/trajectory_processor_naive.h>
#include <orunav_vehicle_execution/path_smoother_dynamic.h>

// #include <orunav_node_utils/robot_target_handler.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <orunav_rviz/orunav_rviz.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

class KMOVehicleExecutionNode
{

    private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_compute_;
    ros::Publisher compute_status_pub_;
    ros::ServiceServer service_execute_;
    ros::ServiceServer service_brake_;

    ros::Publisher trajectorychunk_pub_, smooth_path_pub_ , path_pub;
    ros::Publisher command_pub_;
    ros::Publisher forkcommand_pub_;
    ros::Publisher marker_pub_, marker_pub1_, marker_pub2_, marker_pub3_, marker_pub4_, marker_pub5_, marker_pub6_;
    ros::Publisher report_pub_;
    ros::Publisher planningmap_pub_, smooothed_pub_;

    ros::Subscriber laserscan_sub_, goal_sub_;
    ros::Subscriber laserscan2_sub_;

    ros::Subscriber control_report_sub_, odom_sub_, update_goal_sub_;
    ros::Subscriber fork_report_sub_;
    ros::Subscriber enc_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber pallet_poses_sub_;

    ros::Subscriber velocity_constraints_sub_;
    ros::Subscriber ebrake_sub_;

    boost::mutex map_mutex_, inputs_mutex_, current_mutex_, run_mutex_;
    boost::thread client_thread_;
    boost::condition_variable cond_;

    VehicleState vehicle_state_;
    // orunav_node_utils::RobotTargetHandler target_handler_;
    long current_id_;

    nav_msgs::OccupancyGrid current_map_;
    bool valid_map_;

    orunav_msgs::VectorMap vector_map_;
    orunav_msgs::GeoFence geofence_;

    bool got_first_state_;
    bool valid_control_;

    orunav_generic::Path current_path_;             // This is used to communicate the actual path to be driven...
    orunav_generic::Path current_constraints_path_; // Only used for visualization...

    orunav_msgs::RobotTarget current_target_;
    std::string current_state_str_;
    orunav_generic::CoordinatedTimes current_cts_;

    // Path generation params
    double min_incr_path_dist_;
    int min_nb_path_points_;
    TrajectoryProcessor::Params traj_params_;
    TrajectoryProcessor::Params traj_params_original_;
    TrajectoryProcessor::Params traj_slowdown_params_;
    bool overwrite_traj_params_with_velocity_constraints_;

    // Forklift settings
    int robot_id_;
    bool use_forks_;
    std::string model_name_;

    // Internal flags
    bool b_shutdown_;

    // Driving params
    double max_tracking_error_;

    ros::Timer heartbeat_report_pub_;

    // Visualization params
    bool visualize_;
    ros::Timer heartbeat_slow_visualization_;
    ros::Timer heartbeat_fast_visualization_;
    ros::Timer heartbeat_perception_;
    double current_start_time_;
    double time_to_meter_factor_;

    bool draw_sweep_area_;

    orunav_geometry::RobotModel2dCiTiTruck model1;
    orunav_geometry::RobotModel2dCiTiTruckWithArm model2;
    orunav_geometry::RobotModel2dPitViper model_pitviper; // TODO - this should be removed and placed in ac.
    orunav_geometry::RobotModel2dBtTruck model_bt;
    orunav_geometry::RobotModel2dInterface *model_;

    // Docking params
    double min_docking_distance_;
    double max_docking_distance_;
    double min_docking_driven_distance_;
    double max_target_angular_diff_;
    double max_target_distance_diff_;
    double max_target_distance_diff_fwd_;
    double max_target_distance_diff_side_;
    double max_steering_range_smoothed_path_;
    double overshoot_distance_;
    int docking_max_nb_opt_points_;

    // Coordination parameters
    bool use_ct_; // This is probably obsolete.
    bool provide_dts_; // If the use_ct is not set, compute the dts anyway to provide this information to the coordinator.

    // Motion planner parameters
    bool use_vector_map_and_geofence_;

    // Used for the laser safety functionallities - important, the boost polygon performs really bad when used in debug mode (factor of 100s).
    tf::TransformListener tf_listener_;
    laser_geometry::LaserProjection laser_projection_;
    double ebrake_lookahead_time_;
    double slowdown_drivingslow_lookahead_time_;
    double slowdown_lookahead_time_;
    bool use_safetyregions_;

    int chunk_idx_connect_offset_;

    ros::Time last_process_fork_report_time_;
    bool cts_clear_first_entry_in_pairs_;
    bool use_ahead_brake_;
    bool visualize_sweep_and_constraints_;

    bool use_update_task_service_;
    bool start_driving_after_recover_;

    bool real_cititruck_;
    bool no_smoothing_;
    bool resolve_motion_planning_error_;

    std::set<int> ebrake_id_set_;

    double max_linear_vel_pallet_picking_;
    double max_rotational_vel_pallet_picking_;
    double max_linear_vel_rev_pallet_picking_;
    double max_rotational_vel_rev_pallet_picking_;

    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped goal_pose;
    nav_msgs::Odometry odom;

    PathSmootherDynamic path_smoother;
    PathPlannerService path_planner;
    bool is_goal_set = false;
    std::mutex lock_on_solver;
    std::condition_variable condition_on_solver;
    bool granted_execution = false;
    std::thread solver_thread;
    bool have_trajector_ = false;

    nav_msgs::Path temp_path;
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::Buffer tfBuffer;
    tf::TransformListener listener;

    public:
        KMOVehicleExecutionNode(ros::NodeHandle &paramHandle);
        KMOVehicleExecutionNode();
        
        void drawPointCloud(const sensor_msgs::PointCloud &points, const std::string &name, int id, int color, double scale, ros::Publisher &pub);
        void updateTrajParamsWithVelocityConstraints(TrajectoryProcessor::Params &traj_params, const VehicleState &vehicle_state);
        void goalTrajectoryCallback(const geometry_msgs::PoseStampedConstPtr& msg1);
        void odomCallback(const nav_msgs::OdometryConstPtr& msg);
        void processPlan(const nav_msgs::PathConstPtr& msg1);
        void publishInitPath(const std::vector<orunav_msgs::PoseSteering>& path, double r, double g, double b, double a);
        void publishSmoothPath(const orunav_generic::Path& path, double r, double g, double b, double a);
        void run();
        void calculateCommands(orunav_generic::Path& path);
        void sendTrajectoryChunks(const std::pair<unsigned int, orunav_generic::TrajectoryChunks> &chunks_data);
        void sendActivateStartTimeCommand(const ros::Time &startTime);
        void process_report(const orunav_msgs::ControllerReportConstPtr &msg);
        void process_velocity_constraints(const std_msgs::Float64MultiArrayConstPtr &msg);
        void publish_report(const ros::TimerEvent &event);
        void visualizeCurrentMission();
        std::thread startPathPlannerThread();
        void solverThread();
        void publishGlobalPath(const orunav_generic::Path& path, double r, double g, double b, double a);

};


#endif 
