#ifndef PATH_SMOOTHER_B
#define PATH_SMOOTHER_B


// #include <Eigen/Eigen>
// #include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <acado_toolkit.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>
#include <orunav_vehicle_execution/acado_tools.h>

#include <tf/tf.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <nav_msgs/GetPlan.h>

#include <orunav_vehicle_execution/path_planner.h>
#include <orunav_vehicle_execution/trajectory_processor_naive.h>
#include <orunav_vehicle_execution/trajectory_processor.h>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/pose2d.h>
// #include <map_building_2d/edtoctomap.h>
// #include <map_building_2d/edt_map_environment.h>
// #include <decom_rviz_plugins/data_ros_utils.h>
// #include <decom_rviz_plugins/ellipsoid_decomp.h>
// #include <decom_rviz_plugins/multi_detector.h>

class SplitIndex
{
 public:
  class Params
  {
  public:
    Params()
      {
        max_nb_points = 10;
        nb_points_discard = 5;
      }
      int max_nb_points;
      int nb_points_discard;
      bool valid() {
        if (max_nb_points > nb_points_discard)
          return true;
        return false;
      }
    friend std::ostream& operator<<(std::ostream &os, const SplitIndex::Params &obj)
      {
        os << "\nmax_nb_points     : " << obj.max_nb_points;
        os << "\nnb_points_discard : " << obj.nb_points_discard;
	      return os;
      }
  };
  
 SplitIndex(Params &params, const orunav_generic::Trajectory &traj) : _traj(traj) {

    assert(params.valid());
    _params = params;
    bool done = false;
    int start = 0;
    int size = traj.sizeTrajectory();
    while (!done) {
      int stop = start + _params.max_nb_points;
      if (stop >= size) {
        stop = size;
        done = true;
      }
      _indexes.push_back(this->getIncrementVec(start, stop));
      start += _params.max_nb_points - _params.nb_points_discard - 1;
    }
  }
  
  std::vector<int> getIncrementVec(int start, int stop) const {
    std::vector<int> ret;
    for (int i = start; i < stop; i++) {
      ret.push_back(i);
    }
    return ret;
  }

  std::vector<orunav_generic::Trajectory> getTrajectories(const orunav_generic::Trajectory &traj) const {
    std::vector<orunav_generic::Trajectory> ret;
    for (int i = 0; i < _indexes.size(); i++) {
      ret.push_back(orunav_generic::selectTrajectoryIndexes(traj, _indexes[i]));
    }
    return ret;
  }


  void setTrajectory(int idx, const orunav_generic::Trajectory &traj) {
    bool last_entry = (idx == _indexes.size()-1);
    int stop = _indexes[idx].size();
    if (!last_entry) {
      stop = _indexes[idx].size()  - _params.nb_points_discard;
    }

    int offset = _indexes[idx][0];
    for (int i = 0; i < stop; i++) { 
      
      //      _traj.addTrajectoryPoint(traj.getPose2d(i), traj.getSteeringAngle(i), traj.getDriveVel(i), traj.getSteeringVel(i));
      _traj.setPose2d(traj.getPose2d(i), offset+i);
      _traj.setSteeringAngle(traj.getSteeringAngle(i), offset+i);
      _traj.setDriveVel(traj.getDriveVel(i), offset+i);
      _traj.setSteeringVel(traj.getSteeringVel(i), offset+i);
    }
  }

  orunav_generic::Trajectory getTrajectory(int idx) const {
    return orunav_generic::selectTrajectoryIndexes(this->_traj, _indexes[idx]);
  }

  orunav_generic::Trajectory getTrajectory() const {
    return _traj;
  }

  void printDebug() const {
    for (int i = 0; i < _indexes.size(); i++) {
      std::cout << "\n[vec:" << i << "]" << std::endl;
      for (int j = 0; j < _indexes[i].size(); j++) {
	std::cout << "(" << _indexes[i][j] << ")" << std::flush;
      }
    }
    std::cout << std::endl;
  }

  int size() const {
    return _indexes.size();
  }

 private:
  std::vector<std::vector<int> > _indexes;
  orunav_generic::Trajectory _traj;
  Params _params;
};


class PathSmootherDynamic
{
 public:
  class Params
  {
  public:
    Params(){
        v_min = -1;
        v_max = 1;
        w_min = -1;
        w_max = 1;
        phi_min = -1.1;
        phi_max = 1.1;
        get_speed = true; //true;
        init_states = true;
        init_controls = true;
        max_nb_opt_points = 12;
        even_point_dist = true;	
        min_dist = -1.;
        use_v_constraints = true;//true;
        use_w_constraints = true;//true;
        use_total_time = true;
        nb_iter_steps = 100;
        visualize = true; //false;
        wheel_base = 0.68; // snowwhite
        update_v_w_bounds = true;
        keep_w_bounds = false;
        w_zero = false;
        minimize_phi_and_dist = false;
        use_th_constraints = true; // Note that this is only used if there is any constraints provided.
        use_xy_constraints = true; // Note that this is only used if there is any constraints provided.
        use_constraints_modulus = 1;
        kkt_tolerance = 0.001;
        integrator_tolerance = 0.0001;
        weight_steering_control = 1.;
        use_multiple_shooting = true;
        use_condensing = true;
        use_incremental = true;
        incr_max_nb_points = 20;
        incr_nb_points_discard = 5;
    }
      double v_min;
      double v_max;
      double w_min;
      double w_max;
      double phi_min;
      double phi_max;
      bool get_speed;
      bool init_states;
      bool init_controls;
      int max_nb_opt_points;
      bool even_point_dist;
      double min_dist;
      bool use_v_constraints;
      bool use_w_constraints;
      bool use_total_time;
      int nb_iter_steps;
      bool visualize;
      double wheel_base;
      bool update_v_w_bounds;
      bool keep_w_bounds;
      bool w_zero;
      bool minimize_phi_and_dist;
      bool use_th_constraints;
      bool use_xy_constraints;
      int use_constraints_modulus;
      double kkt_tolerance;
      double integrator_tolerance;
      double weight_steering_control;
      bool use_multiple_shooting;
      bool use_condensing;
      bool use_incremental;
      int incr_max_nb_points;
      int incr_nb_points_discard;
      
      friend std::ostream& operator<<(std::ostream &os, const PathSmootherDynamic::Params &obj){
        os << "\nv_min             : " << obj.v_min;
        os << "\nv_max             : " << obj.v_max;
        os << "\nw_min             : " << obj.w_min;
        os << "\nw_max             : " << obj.w_max;
        os << "\nphi_min           : " << obj.phi_min;
        os << "\nphi_max           : " << obj.phi_max;
        os << "\nget_speed         : " << obj.get_speed;
        os << "\ninit_states       : " << obj.init_states;
        os << "\ninit_controls     : " << obj.init_controls;
        os << "\nmax_nb_opt_points : " << obj.max_nb_opt_points;
        os << "\neven_point_dist   : " << obj.even_point_dist;
        os << "\nmin_dist          : " << obj.min_dist;
        os << "\nuse_v_constraints : " << obj.use_v_constraints;
        os << "\nuse_w_constraints : " << obj.use_w_constraints;
        os << "\nuse_total_time    : " << obj.use_total_time;
        os << "\nnb_iter_steps     : " << obj.nb_iter_steps;
        os << "\nvisualize         : " << obj.visualize;
        os << "\nwheel_base        : " << obj.wheel_base;
        os << "\nupdate_v_w_bounds : " << obj.update_v_w_bounds;
        os << "\nkeep_w_bounds     : " << obj.keep_w_bounds;
        os << "\nw_zero            : " << obj.w_zero;
        os << "\nminimize_phi_dist : " << obj.minimize_phi_and_dist;
        os << "\nuse_th_constraints: " << obj.use_th_constraints;
        os << "\nuse_xy_constraints: " << obj.use_xy_constraints;
        os << "\nuse_constr_modulus: " << obj.use_constraints_modulus;
        os << "\nkkt_tolerance     : " << obj.kkt_tolerance;
        os << "\nintegr_tolerance  : " << obj.integrator_tolerance;
        os << "\nweight_steer_ctrl : " << obj.weight_steering_control;
        os << "\nuse_multiple_s... : " << obj.use_multiple_shooting;
        os << "\nuse_condensing    : " << obj.use_condensing;
        os << "\nuse_incremental   : " << obj.use_incremental;
        os << "\nincr_max_nb_points: " << obj.incr_max_nb_points;
        os << "\nincr_nb_points_dis: " << obj.incr_nb_points_discard;
        return os;
      }
  };


  PathSmootherDynamic();
  PathSmootherDynamic::Params params;
 
  // MultiDetector detector1;55
  bool visualize;
  bool do_nothing;
  bool reassign_constraints;
  int reassign_iters;
  double reassign_min_distance_;
  ros::Publisher marker_pub, smooth_path_pub_, path_pub;
  ros::Subscriber odom_sub_, global_path_sub_;
  geometry_msgs::PoseStamped current_pose;
  std::string global_frame_;

  nav_msgs::Odometry odom;
  void init(ros::NodeHandle& node_);
  double get_yaw(geometry_msgs::PoseStamped msg);
  // void computeThBounds(const double &lb_orig, const double &ub_orig, const double &th, double &lb_new, double &ub_new);
  orunav_generic::Trajectory smooth_(const orunav_generic::Trajectory &traj, double dt, double start_time
                                                                                , double stop_time, bool use_pose_constraints);
  void smoothTraj(const orunav_generic::Path& path_original
                                , orunav_generic::Path& smoothed_path
                                , const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
  
  void publishSmoothPath(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);
  void publishInitPath(const std::vector<orunav_msgs::PoseSteering>& path, double r, double g, double b, double a);
};

#endif 