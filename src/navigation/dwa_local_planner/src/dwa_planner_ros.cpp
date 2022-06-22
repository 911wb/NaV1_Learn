/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // 更新局部规划器的通用配置参数
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // 更新dwa的特定配置
      dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false) {

  }
  // 初始化局部规划器，初始化虚函数
  void DWAPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) 
  {
    if (! isInitialized()) 
    {
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // 更新局部规划器的代价地图
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      // 创建局部路径规划器，参数服务器会自动的配置默认参数， dp_是指向DWAPlanner类的shared_ptr
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      initialized_ = true;

      // 这些参数在以后的版本可能会被移除
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  // 接口函数，没实现具体功能，保存全局规划的路径
  bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) 
  {
    if (! isInitialized()) 
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // 更新了新的全局路径后，清除latch
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  // 接口函数，没实现具体功能
  bool DWAPlannerROS::isGoalReached() 
  {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS::~DWAPlannerROS(){
    // 之前new的,在析构函数中要被delele
    //make sure to clean things up
    delete dsrv_;
  }



  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) 
  {
    // 用动态窗口采样得到下发速度
    if(! isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // OdometryHelperRos的对象，getRobotVel从里程计得到当前速度
    geometry_msgs::PoseStamped robot_vel;
    // 获取机器人当前速度
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    // 驱动指令
    geometry_msgs::PoseStamped drive_cmds;
    // 基座的坐标系
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
    /*
    局部规划器以当前速度为参考，产生一个合理且可达的速度采样范围，确定下一步的速度。那么如
    何筛选呢？它用采样速度生成相应的「仿真路径」，借助costmap，从障碍物、与目标的距离、与全局规
    划路径的距离几个方面对路径成本进行评估，选择最优成本的路径，将它对应的采样速度发布给机器
    人，控制其运动。若在循环生成前向路径的过程中，前方遇障，无法得到前向的有效路径，那么进入
    逃逸模式，不断后退、旋转，离开一段距离后再进行前向规划，向前运动。在原地自转时，注意震荡
    控制，防止机器人左右频繁来回旋转。
    */
    // notice: 输入机器人位姿和速度，输出：下发给基座的速度命令drive_cmds、局部轨迹path
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    // ?传递运动速度
    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    //  如果没有计算出有效路径，打出log
    // 上一个周期计算出的局部路径规划或轨迹
    std::vector<geometry_msgs::PoseStamped> local_plan;

    /*
    接下来对生成路径path的代价进行判断，若为负，说明是无效路径，返回false；
    若为正，说明找到有效路径，将其进行格式转换后通过话题发布，便于对局部规划结果可视化。
    如果路径代价<0，说明没找到合法的路径，对于所有模拟路径，机器人的足迹都在振荡
    */
    if(path.cost_ < 0)
    {
      ROS_DEBUG_NAMED("dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // 找到有效的局部路径 遍历每一个有效的局部路径点
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) 
    {
      // 填充本地路径local_plan，把path中的点存放到PoseStamped的实例p中，
      // 用pose存储，一个个添加到local_plan中
      double p_x, p_y, p_th;

      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      // 转换成欧拉角
      tf2::convert(q, p.pose.orientation);

      // 将转换后的局部路径点存入局部路径
      local_plan.push_back(p);
    }

    //publish information to the visualizer
    // 发布局部路径用于可视化
    publishLocalPlan(local_plan);
    return true;
  }



  // 局部规划器的核心函数,该函数计算本次循环的下发速度
  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
  {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    // 获取机器人全局坐标系下的位姿
    if ( ! costmap_ros_->getRobotPose(current_pose_)) 
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    // 储存转换之后的路径
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    /*
    notice: 将全局路径规划从地图坐标系(pixel) 转换到 global系(m)，得到transformed plan
    并且修剪掉方框之外的路径
    */
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) 
    {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    // 如果全局路径是空，不做任何操作
    if(transformed_plan.empty()) 
    {
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // 在dwa_planner中更新全局路径(即使是停止或者旋转的动作)，可以检测轨迹
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());
    
    // 判断是否到达终点位置
    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) 
    {
      // 到了目标位置后，发布出空的路径规划
      std::vector<geometry_msgs::PoseStamped> local_plan;

      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      // 发布全局路径
      publishGlobalPlan(transformed_plan);
      // 发布局部路径
      publishLocalPlan(local_plan);
      // 速度限制
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      // 如果到达目标点附近，旋转机器人朝向对齐目标点朝向
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,// 最优轨迹对应的速度空间中的采样速度，恒速模型时
          limits.getAccLimits(),// 最大加速度
          dp_->getSimPeriod(),// 获得局部规划器预期所需要的计算时间
          &planner_util_,// 规划器辅助对象
          odom_helper_,// 用来辅助获取odom信息
          current_pose_,// 机器人当前位姿
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
    }
    else
    {
      // 没有到达目标点附近，则不断的调用该函数，计算速度指令(最优轨迹对应的速度空间中的采样速度，恒速模型)
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      
      if (isOk)
      {
        publishGlobalPlan(transformed_plan);
      } 
      else 
      {
        ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }


};
