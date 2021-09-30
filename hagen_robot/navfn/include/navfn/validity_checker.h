#ifndef VALIDITY_CHECKER_H
#define VALIDITY_CHECKER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <map_building_2d/edtoctomap.h>
#include <map_building_2d/edt_map_environment.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
namespace reeds_shepp
{

  class ValidityChecker : public ompl::base::StateValidityChecker
  {
  public:
      ValidityChecker(const ompl::base::SpaceInformationPtr& si);
      bool isValid(const ompl::base::State* state);
      double clearance(const ompl::base::State* state) const;
  };
}

#endif 
