#include <navfn/path_smoother_dynamic.h>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

PathSmootherDynamic::PathSmootherDynamic(){
  // detector1.init();
}

void PathSmootherDynamic::init(ros::NodeHandle& node_){
  ros::NodeHandle nh = ros::NodeHandle("~");
  nh.param<bool>("visualize",visualize,false);
  nh.param<bool>("visualize_deep", params.visualize, false);
  nh.param<int>("nb_iter_steps", params.nb_iter_steps, 100);
  nh.param<bool>("minimize_phi_and_dist", params.minimize_phi_and_dist, true);
  nh.param<double>("wheel_base",params.wheel_base, 0.68);
  nh.param<bool>("init_controls", params.init_controls, false);
  nh.param<bool>("use_th_constraints", params.use_th_constraints, true);
  nh.param<bool>("use_xy_constraints", params.use_xy_constraints, true);
  nh.param<double>("kkt_tolerance", params.kkt_tolerance, 0.01);
  nh.param<double>("integrator_tolerance", params.integrator_tolerance, 0.045601 );
  nh.param<double>("weight_steering_control", params.weight_steering_control, 1.);
  nh.param<double>("phi_min", params.phi_min, -1.1);
  nh.param<double>("phi_max", params.phi_max, 1.1);
  nh.param<double>("v_min", params.v_min, -0.368201);
  nh.param<double>("v_max", params.v_max, 0.657771);
  nh.param<double>("w_min", params.w_min, -0.683589);
  nh.param<double>("w_max", params.w_max, 0.455311);
  nh.param<bool>("do_nothing", do_nothing, false);
  // np.param<std::string>("global_frame", global_frame_, "map");
  global_frame_ = "map";
  // nh.param<bool>("reassign_constraints", reassign_constraints, false);
  // nh.param<int>("reassign_iters", reassign_iters, 1);
  // nh.param<double>("reassign_min_distance", reassign_min_distance_, -1.);
  nh.param<bool>("use_incremental", params.use_incremental, false);
  nh.param<int>("incr_max_nb_points", params.incr_max_nb_points, 20);
  nh.param<int>("incr_nb_points_discard", params.incr_nb_points_discard, 5);
  odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 50, &PathSmootherDynamic::odomCallback, this);
  global_path_sub_ = node_.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 10, &PathSmootherDynamic::goalTrajectoryCallback, this);
  smooth_path_pub_ = node_.advertise<nav_msgs::Path>("/move_base/hagen/smoothpath", 1);
}

void PathSmootherDynamic::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom = *msg;
  current_pose.pose.position.x = odom.pose.pose.position.x;
  current_pose.pose.position.y = odom.pose.pose.position.y;
  current_pose.pose.position.z = 0.0;
}

void PathSmootherDynamic::goalTrajectoryCallback(const nav_msgs::PathConstPtr& msg){
    std::vector<geometry_msgs::PoseStamped> current_path;
    std::vector<geometry_msgs::PoseStamped> current_smoothed_path;
    for (int i=1; i< msg->poses.size(); i++)
    {
      current_path.push_back(msg->poses[i]);
    }
    geometry_msgs::PoseStamped goal_ = msg->poses[msg->poses.size()-1];
    smoothTraj(current_path, current_smoothed_path, current_pose, goal_);
    publishSmoothPath(current_smoothed_path, 0.3, 0.5, 0.8, 1);
}

void PathSmootherDynamic::publishSmoothPath(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = global_frame_;
    gui_path.header.stamp = ros::Time::now();
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }
    smooth_path_pub_.publish(gui_path);
  }

orunav_generic::Trajectory PathSmootherDynamic::smooth_(const orunav_generic::Trajectory &traj, double dt, double start_time
                                                                                , double stop_time, bool use_pose_constraints){

      // auto converetd_obs = DecompROS::cloud_to_vec(latest_cloud_);
      // Always always...
      ACADO_clearStaticCounters();

      std::cout << "Setting up -- start" << std::endl;

      ACADO::VariablesGrid q_init = convertPathToACADOStateVariableGrid(traj, 0.0, dt);
      
      ACADO::VariablesGrid u_init = convertTrajectoryToACADOControlVariablesGrid(traj, 0.0, dt);
      if (params.w_zero) {
        //	u_init = convertTrajectoryToACADOControlVariablesGrid(orunav_generic::setFixedControlValuesW(traj, 0.), 0.0, dt);
        setFixedACADOControlVariablesGrid(u_init, 0., 0.);
      }

      ACADO::DifferentialEquation f(start_time, stop_time);
      ACADO::DifferentialState x,y,th,phi; // the differential states
      ACADO::Control v, w; // the control input u

      f << dot(x) == cos(th)*v;
      f << dot(y) == sin(th)*v;
      f << dot(th) == tan(phi)*v/params.wheel_base; // 0.68 = L
      f << dot(phi) == w;
      
      ACADO::OCP ocp(q_init);
      if (params.minimize_phi_and_dist) {
	      ocp.minimizeLagrangeTerm(v*v + params.weight_steering_control*w*w);
      }
      else {
	      ocp.minimizeMayerTerm(1.);
      }
      ocp.subjectTo(f);
      // Enforce the the start / end pose.
      std::cout<< "======================traj init=====================" << params.integrator_tolerance <<std::endl;
      ocp.subjectTo(ACADO::AT_START, x == traj.getPose2d(0)(0));
      ocp.subjectTo(ACADO::AT_START, y == traj.getPose2d(0)(1));
      ocp.subjectTo(ACADO::AT_START, th == traj.getPose2d(0)(2));
      ocp.subjectTo(ACADO::AT_START, phi == traj.getSteeringAngle(0));
      std::cout<< traj.getPose2d(0)(0) << "," << traj.getPose2d(0)(1) << traj.getPose2d(0)(2) << "," << traj.getSteeringAngle(0) << std::endl;
      //ocp.subjectTo(ACADO::AT_START, v == 0);
      //ocp.subjectTo(ACADO::AT_START, w == 0);
      
      std::cout<< "======================traj end=====================" <<std::endl;
      ocp.subjectTo(ACADO::AT_END, x == traj.getPose2d(traj.sizePath()-1)(0));
      ocp.subjectTo(ACADO::AT_END, y == traj.getPose2d(traj.sizePath()-1)(1));
      ocp.subjectTo(ACADO::AT_END, th == traj.getPose2d(traj.sizePath()-1)(2));
      ocp.subjectTo(ACADO::AT_END, phi == traj.getSteeringAngle(traj.sizePath()-1));
      std::cout<< traj.getPose2d(traj.sizePath()-1)(0) << "," << traj.getPose2d(traj.sizePath()-1)(1) << traj.getPose2d(traj.sizePath()-1)(2) << "," << traj.getSteeringAngle(traj.sizePath()-1) << std::endl;
      //ocp.subjectTo(ACADO::AT_END, v == 0);
      //ocp.subjectTo(ACADO::AT_END, w == 0);
            
     
      ocp.subjectTo( params.phi_min <= phi <= params.phi_max);
      
      std::cout << "optimization -- start" << std::endl;
      ACADO::OptimizationAlgorithm algorithm(ocp);
      // ACADO params -- 

      std::cout<< "=====>params.use_multiple_shooting " << params.use_multiple_shooting << std::endl;
      if (!params.use_multiple_shooting)
	      algorithm.set( ACADO::DISCRETIZATION_TYPE, ACADO::SINGLE_SHOOTING ); // For the non-objective -> there is not any difference.
      else
	      algorithm.set( ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING );
      {
        int ret;
        algorithm.get( ACADO::DISCRETIZATION_TYPE, ret);
        if (ret == ACADO::SINGLE_SHOOTING) {
          std::cout << "SINGLE_SHOOTING will be used" << std::endl;
        }
        if (ret == ACADO::MULTIPLE_SHOOTING) {
          std::cout << "MULTIPLE_SHOOTING will be used" << std::endl;
        }
      }

      algorithm.set( ACADO::MAX_NUM_INTEGRATOR_STEPS, 100 ); // For the integrator.
      algorithm.set( ACADO::MAX_NUM_ITERATIONS, params.nb_iter_steps ); 
      algorithm.set( ACADO::PRINTLEVEL, ACADO::HIGH );
      // algorithm.set( ACADO::MEX_VERBOSE, 1);
      algorithm.set( ACADO::PRINT_SCP_METHOD_PROFILE, BT_TRUE );
      //      algorithm.set( ACADO::USE_REFERENCE_PREDICTION, ACADO::BT_FALSE );
      // if (params.use_condensing)
      //   algorithm.set( ACADO::USE_CONDENSING, ACADO::BT_TRUE ); // Important!
      // else 
      //   algorithm.set( ACADO::USE_CONDENSING, ACADO::BT_FALSE );
      // {
      //   int ret;
      //   algorithm.get( ACADO::USE_CONDENSING, ret);
      //   if (ret == ACADO::BT_TRUE) {
      //     std::cout << "CONDENSING will be used" << std::endl;
      //   }
      //   if (ret == ACADO::BT_FALSE) {
      //     std::cout << "CONDENSING will NOT be used" << std::endl;
      //   }
      // }

      algorithm.set( ACADO::USE_REALTIME_ITERATIONS, BT_FALSE ); // Important!
      algorithm.set( ACADO::KKT_TOLERANCE, params.kkt_tolerance );
      algorithm.set( ACADO::INTEGRATOR_TOLERANCE, params.integrator_tolerance );
            
      std::cout << "Initialize variables" << std::endl;
      if (params.init_states)
	      algorithm.initializeDifferentialStates( q_init );
      if (params.init_controls)
	      algorithm.initializeControls( u_init );
     
      std::cout << "Setting up states / control variables" << std::endl;
      ACADO::VariablesGrid states, controls;
      std::cout << "-1" << std::endl;
      algorithm.getDifferentialStates(states);
      std::cout << "-2" << std::endl;
      algorithm.getControls(controls);
      std::cout << "-3" << std::endl;
      
      algorithm.getDifferentialStates("/home/geesara/catkin_workspace/src/car_node/data_algo/states_init.txt");
      std::cout << "-4" << std::endl;
      algorithm.getControls("/home/geesara/catkin_workspace/src/car_node/data_algo/controls_init.txt");
      std::cout << "-5" << std::endl;
      

      if (params.visualize) {
        ACADO::GnuplotWindow window4(ACADO::PLOT_AT_START);
        window4.addSubplot( x, "x - init" );
        window4.addSubplot( y, "y - init" );
        window4.addSubplot( th, "th - init" );
        window4.addSubplot( phi, "phi - init" );
        window4.addSubplot( v, "v - init" );
        window4.addSubplot( w, "w - init" );
        
        ACADO::GnuplotWindow window2(ACADO::PLOT_AT_EACH_ITERATION);
        window2.addSubplot( x, "x - iter..." );
        window2.addSubplot( y, "y - iter..." );
        window2.addSubplot( th, "th - iter..." );
        window2.addSubplot( phi, "phi - iter..." );
        window2.addSubplot( v, "v - iter..." );
        window2.addSubplot( w, "w - iter..." );

        algorithm << window2;
        algorithm << window4;
      }

      std::cout << "Solve - running..." << std::endl;
      algorithm.solve();
      std::cout << "Optimization -- end" << std::endl;
      
      algorithm.getDifferentialStates(states);
      algorithm.getControls(controls);
      algorithm.getDifferentialStates("/home/geesara/catkin_workspace/src/car_node/data_algo/states_final.txt");
      algorithm.getControls("/home/geesara/catkin_workspace/src/car_node/data_algo/controls_final.txt");

      if (params.visualize) {
        ACADO::GnuplotWindow window;
        window.addSubplot( q_init(0), "x - provided" );
        window.addSubplot( q_init(1), "y - provided" );
        window.addSubplot( q_init(2), "th - provided" );
        window.addSubplot( q_init(3), "phi - povided" );
        window.addSubplot( u_init(0), "v - provided" );
        window.addSubplot( u_init(1), "w - provided" );
        window.plot();
        
              
        ACADO::GnuplotWindow window3;
        window3.addSubplot( states(0), "x - final" );
        window3.addSubplot( states(1), "y - final" );
        window3.addSubplot( states(2), "th - final" );
        window3.addSubplot( states(3), "phi - final" );
        window3.addSubplot( controls(0), "v - final" );
        window3.addSubplot( controls(1), "w - final" );
        window3.plot();
      }

      //  return convertACADOStateVariableGridToPath(states);
      return convertACADOStateControlVariableGridToTrajectory(states, controls);
  }

  
  double PathSmootherDynamic::get_yaw(geometry_msgs::PoseStamped msg){
      tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
                             msg.pose.orientation.z, msg.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
  }

  void PathSmootherDynamic::smoothTraj(const std::vector<geometry_msgs::PoseStamped>& path_original
                                , std::vector<geometry_msgs::PoseStamped>& smoothed_path
                                , const geometry_msgs::PoseStamped& start_p, const geometry_msgs::PoseStamped& goal_p){
        
            // orunav_generic::State2d start(start_p.pose.position.x, start_p.pose.position.y, get_yaw(start_p), 0.0);
            // orunav_generic::State2d goal(goal_p.pose.position.x, goal_p.pose.position.y, get_yaw(goal_p), 0.0);

             orunav_generic::State2d start(start_p.pose.position.x, start_p.pose.position.y, 0.0, 0.0);
            orunav_generic::State2d goal(goal_p.pose.position.x, goal_p.pose.position.y, 0.0, 0.0);

            ACADO_clearStaticCounters();  
            bool use_pose_constraints;
            use_pose_constraints = false;
            // orunav_generic::Pose2dContainerInterface path_orig;
            orunav_generic::Path path_orig;
            for (int i = 0; i < path_original.size(); i++)
            {
              orunav_generic::Pose2d p;
              p[0] = path_original[i].pose.position.x;
              p[1] = path_original[i].pose.position.y;
              p[2] = 0.0;
              path_orig.addPathPoint(p, 0.0);
            }
            
            std::cout << "PATH SMOOTHER : path_orig.size() : " << path_original.size() << std::endl;
            if (use_pose_constraints) {
              std::cout << "----- will use spatial constraints -----" << std::endl;
            }
            double T = 0;
            orunav_generic::Trajectory traj_gen;
            {
              TrajectoryProcessorNaive gen;
              TrajectoryProcessor::Params p;
              p.maxVel = 0.5;
              p.maxAcc = 0.2;
              p.wheelBaseX = params.wheel_base;
              gen.setParams(p);
              gen.addPathInterface(path_orig);
              traj_gen = gen.getTrajectory();
              T = orunav_generic::getTotalTime(gen);
            }

            std::cout << "--------- Estimated total time T : " << T << " -----------" << std::endl;
            unsigned int orig_size = path_orig.sizePath();
            int skip_points = orig_size / params.max_nb_opt_points - 1;
            if (skip_points < 0)
              skip_points = 0;
            double dt = 0.06 * (1 + skip_points);
            orunav_generic::Path path;
            if (params.even_point_dist) {
              double min_dist = orunav_generic::getTotalDistance(path_orig) / params.max_nb_opt_points;
              if (min_dist < params.min_dist)
                min_dist = params.min_dist;
              path = orunav_generic::minIncrDistancePath(path_orig, min_dist);
            }
            else {
              path = orunav_generic::subSamplePath(path_orig, skip_points);
            }
            if (params.use_total_time) {
              dt = T / path.sizePath(); 
            }

            if (use_pose_constraints) {
              path = path_orig;
            }

            std::cout << "Used dt : " << dt << std::endl;

            //orunav_generic::removeThNormalization(path);
            orunav_generic::Trajectory traj = orunav_generic::convertPathToTrajectoryWithoutModel(path, dt);
            assert(orunav_generic::validPath(traj, M_PI));
            if (orunav_generic::validPath(traj, M_PI))
              std::cerr << "Non-normalized path(!) - should never happen" << std::endl;
            orunav_generic::removeThNormalization(traj);
            assert(orunav_generic::validPath(traj, M_PI));
      
            double start_time = 0.0;
            unsigned int size = traj.sizeTrajectory();
            double stop_time = (size - 1)*dt;
      
            if (params.update_v_w_bounds) {
              PathSmootherDynamic::Params params_orig = params;
              if (params.init_controls || params.get_speed) {
                orunav_generic::getMinMaxVelocities(traj, params.v_min, params.v_max, params.w_min, params.w_max);
              }
              else {
                // Using the generated trajectory (from the traj processsor)
                orunav_generic::getMinMaxVelocities(traj_gen, params.v_min, params.v_max, params.w_min, params.w_max);
              }
              // This is used to smooth a straight path which otherwise will have a w_min = w_max = 0.
              if (params.keep_w_bounds) {
                params.w_min = params_orig.w_min;
                params.w_max = params_orig.w_max;
              }
              std::cout << "------ Updated v/w params : -------- " << params << std::endl;
            }

            // Make sure to normalize the start and end pose - this is done by updating the start and end trajectory state, this should be done after the velocities are computed.
            traj.setPose2d(start.getPose2d(), 0);
            traj.setSteeringAngle(start.getSteeringAngle(), 0);
            traj.setPose2d(goal.getPose2d(), traj.sizePath()-1);
            traj.setSteeringAngle(goal.getSteeringAngle(), traj.sizePath()-1);

            std::cout << "Updating the start and end pose : " << std::endl;
            assert(orunav_generic::validPath(traj, M_PI));
            orunav_generic::removeThNormalization(traj); // Force the start point and end point to not be normalized.
            assert(orunav_generic::validPath(traj, M_PI));
             orunav_generic::Trajectory smoothed_trajectory;
            if (!params.use_incremental)
            {
              
              smoothed_trajectory = smooth_(traj, dt, start_time, stop_time, use_pose_constraints); 
              orunav_generic::Path final_smoothed_path(smoothed_trajectory);
              std::vector<geometry_msgs::PoseStamped> smoothed_;
              for(int i=0; i< final_smoothed_path.sizePath(); i++){
                geometry_msgs::PoseStamped stap;
                stap.pose.position.x = final_smoothed_path.getPose2d(i)[0];
                stap.pose.position.y = final_smoothed_path.getPose2d(i)[1];
                stap.pose.position.z = 0.0;
                smoothed_.push_back(stap);
              }
              smoothed_path = smoothed_;
              return;
            }
            //     // Run the iterative approach
            // Keep the start and goal fixed (as previous) but divide the section to chunks and optimize over the cunks. The start of the next chunk is located in the previous chunk.
            SplitIndex::Params si_params;
            si_params.max_nb_points =  params.incr_max_nb_points;
            si_params.nb_points_discard = params.incr_nb_points_discard;
            int nb_points_discard;
            SplitIndex si(si_params, traj);
            for (int i = 0; i < si.size(); i++) { 
              orunav_generic::Trajectory t = si.getTrajectory(i);
              stop_time = dt * t.sizeTrajectory()-1;
              std::cout << "stop_time : " << stop_time << std::endl;
              std::cout << "================================================" << std::endl;
              std::cout << "t.sizeTrajectory() : " << t.sizeTrajectory() << std::endl;
              orunav_generic::Trajectory ts = smooth_(t, dt, start_time, stop_time, use_pose_constraints); 
              std::cout << "ts.sizeTrajectory() : " << ts.sizeTrajectory() << std::endl;
              si.setTrajectory(i, ts);
            }
            
            smoothed_trajectory = si.getTrajectory();
            orunav_generic::Path final_smoothed_path(smoothed_trajectory);
            std::vector<geometry_msgs::PoseStamped> smoothed_;
            for(int i=0; i< final_smoothed_path.sizePath(); i++){
              geometry_msgs::PoseStamped stap;
              stap.pose.position.x = final_smoothed_path.getPose2d(i)[0];
              stap.pose.position.y = final_smoothed_path.getPose2d(i)[1];
              stap.pose.position.z = 0.0;
              smoothed_.push_back(stap);
            }
            smoothed_path = smoothed_;
            return;
  }


  