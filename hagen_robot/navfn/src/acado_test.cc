#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>

#include <acado_toolkit.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

#include <navfn/path_smoother_dynamic.h>
#include <geometry_msgs/PoseStamped.h>


#include <thread>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_acord");
    PathSmootherDynamic path_smoother;
    ros::AsyncSpinner spinner(1);
    ros::NodeHandle nh("~");

    path_smoother.init(nh); 
    spinner.start();

    ros::waitForShutdown(); 
    // std::vector<geometry_msgs::PoseStamped> smoothed_path;
    // std::vector<geometry_msgs::PoseStamped> fake_path;
    // geometry_msgs::PoseStamped start;
    // geometry_msgs::PoseStamped goal;
    // geometry_msgs::PoseStamped inter_m;
    // start.pose.position.x = 3.45;
    // start.pose.position.y = 12;
    // start.pose.position.z = 0;
    // goal.pose.position.x =4.45;
    // goal.pose.position.y = 1.4;
    // goal.pose.position.z = 0;
    // inter_m.pose.position.x = 39.45;
    // inter_m.pose.position.y = 16;
    // inter_m.pose.position.z= 0;
    // fake_path.push_back(start);
    // fake_path.push_back(inter_m);
    // fake_path.push_back(goal);
    // path_smoother.smoothTraj(fake_path, smoothed_path, start, goal);
 

    return 0;
}
