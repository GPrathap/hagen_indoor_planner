/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kouros
*********************************************************************/

#include "navfn/reeds_shepp_ros.h"
#include <tf/tf.h>
#include <boost/foreach.hpp>

namespace reeds_shepp
{
  RSPathsROS::RSPathsROS()
  {
    initialized_ = false;
  }


  RSPathsROS::~RSPathsROS()
  {
    // delete costmapModel_;
  }


  void RSPathsROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    ros::NodeHandle pnh("~/" + name);
    pnh.param("min_turning_radius", minTurningRadius_, 1.0);
    pnh.param("max_planning_duration", maxPlanningDuration_, 0.2);
    pnh.param<int>("valid_state_max_cost", validStateMaxCost_, 252);
    pnh.param<int>("interpolation_num_poses", interpolationNumPoses_, 20);
    pnh.param<bool>("allow_unknown", allowUnknown_, false);
    pnh.param<int>("skip_poses", skipPoses_, 0);
    pnh.param<bool>("display_planner_output", displayPlannerOutput_, false);
    pnh.param<double>("obs_min_distance", obs_min_distance_, 0.5);

    reedsSheppStateSpace_ = std::make_shared<ompl::base::ReedsSheppStateSpace>();
    simpleSetup_ = std::make_shared<ompl::geometric::SimpleSetup>(reedsSheppStateSpace_);
    // bx_(10), by_(10), bounds_(2)

    robotFrame_ = costmap_ros->getBaseFrameID();
    globalFrame_ = costmap_ros->getGlobalFrameID();
    costmap_ = costmap_ros->getCostmap();
    costmapModel_ = std::make_shared<base_local_planner::CostmapModel>(*costmap_);
    // costmapROS_ = costmap_ros;
    // initialize the state space boundary
    std::cout<< " =========================" << costmap_->getSizeInMetersX() << "," << costmap_->getSizeInMetersY() << std::endl;
    std::cout<< " =========================" << costmap_->getOriginX() << "," << costmap_->getOriginY() << std::endl;
    footprint_ =  costmap_ros->getRobotFootprint();
    setBoundaries(costmap_->getOriginX(), costmap_->getOriginY(), costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());

    

    initialized_ = true;
  }

  void RSPathsROS::updateCostMap(costmap_2d::Costmap2D* costmap){
    costmap_ = costmap;
    costmapModel_ = std::make_shared<base_local_planner::CostmapModel>(*costmap_);
    return;
  }

  void RSPathsROS::setBoundaries(const double origin_x, const double origin_y, const double bx, const double by)
  {
    bx_ = bx;
    by_ = by;
    ompl::base::RealVectorBounds bounds_(2);
    bounds_.low[0] = origin_x;
    bounds_.low[1] = origin_y;
    bounds_.high[0] = origin_x + bx;
    bounds_.high[1] = origin_y + by;
    reedsSheppStateSpace_->as<ompl::base::SE2StateSpace>()->setBounds(bounds_);
    // create bounds for the x axis
    coordXBound.reset(new ompl::base::RealVectorBounds(2-1));
    coordXBound->setLow(origin_x);
    coordXBound->setHigh(origin_x + bx);

    // create bounds for the y axis
    coordYBound.reset(new ompl::base::RealVectorBounds(2-1));
    coordYBound->setLow(origin_y);
    coordYBound->setHigh(origin_y+by);
  }

  // void RSPathsROS::transform(const geometry_msgs::PoseStamped& poseIn,
  //   geometry_msgs::PoseStamped& poseOut, std::string targetFrameID)
  // {
  //   tf::Stamped<tf::Pose> tfIn, tfOut;
  //   tf::poseStampedMsgToTF(poseIn, tfIn);
  //   transform(tfIn, tfOut, targetFrameID);
  //   tf::poseStampedTFToMsg(tfOut, poseOut);
  // }

  // void RSPathsROS::transform(const tf::Stamped<tf::Pose>& tfIn,
  //   tf::Stamped<tf::Pose>& tfOut, std::string targetFrameID)
  // {
  //   if (tfListener_)
  //     tfListener_->transformPose(targetFrameID, stamp_, tfIn,
  //       tfIn.frame_id_, tfOut);
  // }


  void RSPathsROS::state2pose(
    const ompl::base::State* state, geometry_msgs::PoseStamped& pose)
  {
    const ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();
    pose.pose.position.x = s->getX();
    pose.pose.position.y = s->getY();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
    pose.header.frame_id = robotFrame_;
    pose.header.stamp = stamp_;
  }


  void RSPathsROS::pose2state(
    const geometry_msgs::PoseStamped& pose, ompl::base::State* state)
  {
    ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();
    s->setX(pose.pose.position.x);
    s->setY(pose.pose.position.y);
    s->setYaw(tf::getYaw(pose.pose.orientation));
  }

  void RSPathsROS::setEnvironment(const hagen_planner::EDTEnvironment::Ptr &env)
  {
    edt_env_ = env;
  }

  ompl::base::OptimizationObjectivePtr RSPathsROS::getPathLengthObjective(const ompl::base::SpaceInformationPtr& si){
    auto fg = new ompl::base::PathLengthOptimizationObjective(si);
    return ompl::base::OptimizationObjectivePtr(fg);
  }


  bool RSPathsROS::isStateValid(
    const ompl::base::SpaceInformation* si, const ompl::base::State *state)
  {
    // std::cout<< "=================111111" << std::endl;
    // check if state is inside boundary
    if (!si->satisfiesBounds(state))
      return false;
    // std::cout<< "=================111112" << std::endl;
    const ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();
    // std::cout<< "=================111113" << std::endl;
    Eigen::Vector3d pose(s->getX(), s->getY(), 0);
    // std::cout<< "=================111114" << std::endl;
    double dis = edt_env_->get_free_distance(pose);
    // std::cout<< "=================111115" << std::endl;
    // std::cout<< "robot pose: " << pose.transpose() << "," << dis << std::endl;

    // uint8_t cost = costmap_->footprintCost(s->getX(), s->getY(), s->getYaw(), footprint_);
    // std::cout<< "cost: "<< cost << std::endl;  
    geometry_msgs::PoseStamped statePose;
    state2pose(s, statePose);
    int cost = costmapModel_->footprintCost(statePose.pose.position.x, statePose.pose.position.y,tf::getYaw(statePose.pose.orientation), footprint_);
    
    // std::cout<< "robot close to obstcles before: " << dis << "  cost:"<< cost << std::endl;
    if( dis < obs_min_distance_){
      // std::cout<< "robot close to obstcles " << dis << "  cost:"<< cost << std::endl;
      return false;
    }

    if (fabs(s->getX()) < 5e-2 && fabs(s->getY()) < 5e-2)
      return true;

    // transform(statePose, statePose, globalFrame_);

    // check if state is in collision
    if (cost > validStateMaxCost_ && cost < 256 - (allowUnknown_?1:0)){
      std::cout<< "=========cost ===" << cost << std::endl;
      return false;
    }

    return true;
  }


  bool RSPathsROS::planPath(
    const geometry_msgs::PoseStamped& startPose,
    const geometry_msgs::PoseStamped& goalPose,
    std::vector<geometry_msgs::PoseStamped>& pathPoses)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner not initialized!");
      return false;
    }

    // create start and goal states
    ompl::base::ScopedState<> start(reedsSheppStateSpace_);
    ompl::base::ScopedState<> goal(reedsSheppStateSpace_);

    start[0] = startPose.pose.position.x;
    start[1] = startPose.pose.position.y;
    start[2] = .5*boost::math::constants::pi<double>();
   
    goal[2] = .5*boost::math::constants::pi<double>();
    goal[0] = goalPose.pose.position.x;
    goal[1] = goalPose.pose.position.y;
    
    // auto si(std::make_shared<ompl::base::SpaceInformation>(reedsSheppStateSpace_));
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(reedsSheppStateSpace_));
    si->setStateValidityChecker(boost::bind(&RSPathsROS::isStateValid, this, si.get(), _1));
    si->setStateValidityCheckingResolution(0.2);
  
    // auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
 
    pdef->setStartAndGoalStates(start, goal);
    // ompl::base::ProblemDefinitionPtr pdef1(new ompl::base::ProblemDefinition(si));

    pdef->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(si)));


    auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
    planner->setRange(0.1);// max step length
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(1.0);
    nav_msgs::Path plannedPath;
       std::cout<< "==============solved: ------> " <<  solved << std::endl;
    if (solved) {
        ompl::geometric::PathGeometric* path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        // path->interpolate(interpolationNumPoses_);
        
        // if (path->getStateCount() > interpolationNumPoses_)
        //   return false;
        pathPoses.resize(path->getStateCount());
        for (unsigned int i = 0; i < path->getStateCount(); i++){
          const ompl::base::State* state = path->getState(i);
          state2pose(state, pathPoses[i]);
          pathPoses[i].header.frame_id = robotFrame_;
          pathPoses[i].header.stamp = ros::Time::now();
        }
        std::cout<< "==============2: " <<  path->getStateCount() << std::endl;
    }
   
    std::cout.clear();
    std::cerr.clear();

    return true;
  }


  bool RSPathsROS::planPath(
    const std::vector<geometry_msgs::PoseStamped>& path,
    std::vector<geometry_msgs::PoseStamped>& newPath)
  {
    newPath.clear();

    unsigned int skipPoses = std::min<unsigned int>(skipPoses_, path.size() - 2);

    for (unsigned int i = 0; i < path.size() - 1; i =
      std::min<unsigned int>(i+skipPoses+1, path.size() - skipPoses - 2))
    {
      std::vector<geometry_msgs::PoseStamped> tmpPath;

      if (!planPath(path[i], path[i+skipPoses+1], tmpPath))
      {
        // ROS_ERROR("Failed to plan subplan %d out of %d", i,
          // path.size()/(skipPoses_+1));
        skipPoses++;
        i--;
        continue;
      }

      newPath.insert(newPath.end(), tmpPath.begin(), tmpPath.end());

      if (i >= path.size() - skipPoses - 2)
        break;
    }

    return true;
  }

  bool RSPathsROS::planRecedingPath(
    const std::vector<geometry_msgs::PoseStamped>& path,
    std::vector<geometry_msgs::PoseStamped>& newPath)
  {
    newPath.clear();

    bool success;
    for (unsigned int i = 0; i < floor(sqrt(path.size())); i++)
    {
      success = planPath(
        path.front(), path[(path.size()-1) / (i+1)], newPath);

      if (success)
      {
        for (unsigned j = (path.size()-1)/(i+1); j < path.size(); j++)
        {
          // geometry_msgs::PoseStamped pose;
          // transform(path[j], pose, robotFrame_);
          newPath.push_back(path[j]);
        }
      }
    }

    if (!success)
    {
      // ROS_ERROR("Failed to find valid plan even after reducing path");
      return false;
    }
  }

}  // namespace reeds_shepp
