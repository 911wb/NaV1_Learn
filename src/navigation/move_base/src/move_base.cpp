/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {
  // 构造函数初始化列表，入参是坐标变换的数据
  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), // 加载了baseGlobalPlanner的类库
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), // 加载了baseLocalPlanner的类库
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"), // 加载了recoveryBehaviour的类库
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) 
  {
    /* 
    step1:
    action客户端的目标点move_base_goal传进来，action服务端调用回调函数：executeCb接收该目标点并产生连续反馈、返回处理后的响应结果
    创建action服务对象： 
    参数：节点句柄、话题名称、回调函数作用：解析传入的目标值、产生连续反馈、返回结果
    _1为客户端请求占位，this：当前类对象
    是否自动启动，如果是手动启动，则调用as_.star()来启动action服务端
    ----------------
    第一次接收到goal时会进入该函数，但如果没有完成任务，尚未退出时，
    再有接收到goal并不会再新，建线程进入一次而是通过抢断信号的形式通知该函数，
    所以在处理goal的时候需要经常查看isPreemptRequested函数的返回，看是否有抢占。
    */

    // notice: as_指向action服务器，当执行as_->start()时调用MoveBase::executeCb函数，服务端启动后才能执行回调函数
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    // 私有名字，使用“～”私有命名空间，private_nh设置的参数是私有参数
    // 参考命名空间与节点名称:/命名空间/节点名称/私有参数名称
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    // nh.param("param3", parameter3, 33333);//parameter3 = nh.param("param3", 33333); 如果配置文件中没有把值给键param3则使用默认值33333
    // 设置move_base_node中的全局参数：如果在yaml文件中没有设置对应的参数，则默认使用param函数中最后的参数
    // 获取参数：param(键, ,默认值) 存在，返回对应结果，否则返回默认值
    std::string global_planner, local_planner;
    // step2:从参数服务器加载用户配置文件的参数，包括两个规划器名称、代价地图坐标系、规划频率、控制周期等
    // 全局规划器，默认navfn/NavfnROS
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    // base_link --> base_footprint
    // lua文件用的是base_footprint 
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_footprint"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // parameters of make_plan service
    // make_plan 服务的参数
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    // 创建匿名对象指针，“缓冲池”数组(因为路径是连续的)用于储存全局路径、最新路径、局部路径
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    // step3 新建全局规划线程，线程入口函数为MoveBase::planThread notice:
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    // 发布速度命令给基座：创建发布对象，要发布的消息类型为线速度和角速度，发布的话题为cmd_vel，队列中最大保存的消息数1
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // 发布当前目标点，要发布的消息类型为位姿(四元数表示)，发布的话题为current_goal
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
    // 相对命名空间，相对(话题参考的是节点的命名空间，与节点名称平级)，全局/相对
    // 实例化 ROS 句柄，该类封装了 ROS 中的一些常用功能
    ros::NodeHandle action_nh("move_base");

    // 发布MoveBaseActionGoal消息到/move_base/goal话题上
    // 创建发布对象，要发布的消息类型为position和orientation，发布的话题为goal，队列中最大保存的消息数1
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    // 创建发布对象，要发布的消息类型为触发恢复时机器人的位姿，发布的话题为recovery_status，队列中最大保存的消息数1
    recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

    // 框架中的move_base_simple/goal
    ros::NodeHandle simple_nh("move_base_simple");
    // 发布PoseStamped消息到/move_base_simple/goal话题上，回调函数goalCB会处理在/move_base_simple/goal话题上接收到的消息,供nav_view和rviz等仿真工具使用
    // step4 notice: ??  接收rviz发布的目标点位姿消息，传入goalCB函数，订阅的话题为goal，订阅的消息类型pose 
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    // 加载代价地图的参数（内切、外接、清理半径等），假设机器人的半径和costmap规定的一致
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    // notice: 初始化框架中的 global_costmap
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    // 暂停先不更新使用
    planner_costmap_ros_->pause();

    /*
    初始化全局规划器和局部规划器的指针和各自的costmap，这里两个规划器用到的地图实质上是
    Costmap2DROS类的实例，这个类是ROS对costmap的封装，类函数start()会调用各层地图的
    active()函数，开始订阅传感器话题，对地图进行更新，这部分在代价地图部分详述。
    */
    // step5 初始化全局规划器
    try 
    {
      // notice: 加载插件：全局路径规划器算法 A* 或者 Dijstra
      planner_ = bgp_loader_.createInstance(global_planner);
      // notice: 初始化框架中的 global_planner
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } 
    catch (const pluginlib::PluginlibException& ex) 
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    // notice: 初始化框架中的 local_costmap
    // notice: 父类指针指向子类对象，就可以管理操作多个子类对象
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    // step6 初始化局部规划器
    try 
    {
      // 加载插件：局部规划算法DWA 或者 TEB
      // notice: 父类指针指向子类对象，就可以管理操作多个子类对象
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      // notice: 初始化框中的 local_planner
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } 
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // step7 根据传感器数据动态更新全局和本地的代价地图
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    // step8 发布创建路径的服务，参数1为服务名称，参数2为处理请求的回调函数，处理客户端请求，并返回给客户端响应
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    // 发布清理代价图的服务
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh))
    {
      // notice: 加载默认恢复行为 初始化框中的recovery_behaviors 这里包括了该死的找不到路自转360°
      loadDefaultRecoveryBehaviors();
    }

    
    // 正在规划路径的状态，全局规划的状态
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    // step9 启动action服务端，服务端启动后才能执行回调函数
    // notice: as_指向action服务器，当执行as_->start()时调用MoveBase::executeCb函数 
    as_->start();
    // 动态修改参数
    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    // 把接收到的目标点位姿发布出去可视化 ?? 还是发布给executeCb
    action_goal_pub_.publish(action_goal);  
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  // 清理代价地图的回调函数
  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

  // TODO: 创建路径的回调函数，处理客户端请求(goal)，并返回响应(全局路径)给客户端响应 
  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req.start;
    }

    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }
    // 拷贝全局路径并当做响应，发送出去
    // copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;
    // 中断子线程
    planner_thread_->interrupt();
    // 主线程与子线程汇合
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }
  
  // 输入目标点以及以及储存计算出的路径的容器，计算出的路径保存在plan
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    // 该函数先进行一些预备工作，如检查全局代价地图、起始位姿，然后将起始位姿的数据格式做转换

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    // 初始化空plan
    plan.clear();

    // 如果没有全局代价地图，返回false，因为全局规划必须基于全局代价地图
    if(planner_costmap_ros_ == NULL) 
    {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    // 输入代价地图，获取机器人的起始位姿，存入global_pose，如果得不到机器人的起始位姿，返回false
    if(!getRobotPose(global_pose, planner_costmap_ros_)) 
    {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }
    // 起始点
    const geometry_msgs::PoseStamped& start = global_pose;

    // notice: 如果全局规划器失败了或者返回了一个零长度的路径，规划失败，如果成功计算出来的路径会储存在plan中
    // 关于全局规划的具体算法实现，在NavFn部分具体学习理解。
    if(!planner_->makePlan(start, goal, plan) || plan.empty())
    {
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q)
  {
    // 先检查四元数的值有没有非法值(nan)或者无穷大的值(inf)
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    // 接下来检查四元数的长度是不是趋于零
    if(tf_q.length2() < 1e-6)
    {
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    // 归一化四元数
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }
  // 将goal坐标系下的目标点转换到globa坐标系下
  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try
    {
      // T_goal * T_goal2global = T_global
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  // 唤醒全局规划器线程
  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  // 全局规划子线程时刻等待被executeCB函数唤醒
  void MoveBase::planThread()
  {
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;

    // 不同的线程对同一个变量进行操作时，需要加锁
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok())
    {
      // notice: 当executeCB函数中唤醒planThread并将标志位runPlanner_设置为真，跳出内部的循环，继续进行下面部分
      while(wait_for_wake || !runPlanner_)
      {
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        // 暂时关闭路径规划线程
        // 等待条件满足，如果条件不满足，则释放锁，将线程置为waiting状态，继续等待；如果条件满足，则重新获取锁，结束wait，继续向下执行
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      // start_time设为当前时间
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      // 该开始规划了，复制路径规划器的目标点(注意这里在上次循环中加锁了)，然后在这次解锁，因为临时变量是局部可见的
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      // //清除原来规划出的路径向量
      planner_plan_->clear();
      // 获取机器人的位姿作为起点并计算出全局路径，结果储存在容器planner_plan_中
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
      /*
      ① 若全局规划成功，则交换planner_plan_和latest_plan的值，即令latest_plan中存储的是本次全局规
      划的结果（最新），planner_plan_中存储的是上次全局规划的结果（次新）。设置标志位
      new_global_plan_ = true，表示得到了新的全局规划路线，并设置Movebase状态标志位state_为
      CONTROLLING，即全局规划完成，开始进行局部控制。
      ② 如果全局规划失败，MoveBase还在planning状态，即机器人没有移动，则进入自转模式(clearing)。
      */
      // 如果规划成功
      if(gotPlan)
      {

        ROS_INFO("global plan success send to local !!! ");
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)

        // 如果规划出路径则更新相应路径，并将state_设成CONTROLLING状态
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        // 交换指针指向的内容
        // 将最新的全局路径放到latest_plan_中，其在MoveBase::executeCycle中被传递到controller_plan_中，利用锁来进行同步
        planner_plan_ = latest_plan_;
        // 其在MoveBase::executeCycle中被传递到controller_plan_中，用锁来进行同步
        latest_plan_ = temp_plan;
        // 最近一次有效全局规划的时间设为当前时间
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        // 用于判断能否执行局部路径规划
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        // notice: 确保只有在我们还没到达目标时才启动controller以局部规划
        // 如果没有到达目标点，则进入CONTROLLING状态(为了之后的局部规划)
        if(runPlanner_)
          state_ = CONTROLLING;
        // notice: 如果planner_frequency设置为0，当一个目标点过来，全局规划器就只执行一次，否则循环执行
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      // 如果没有计算出路径，同时state_为PLANNING状态(notice:说明机器人没有移动)
      else if(state_==PLANNING)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        // 最迟制定出本次全局规划的时间 = 上次成功规划的时间 + 容忍时间
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        
        lock.lock();
        // 对同一目标进行全局规划的失败的次数记录+1
        planning_retries_++;

        /*
        notice: 全局路径规划失败后机器人的状态：clearing

        2.判断是否超过最大规划周期 或 者规划次数 (notice:机器人卡在一个地方没有动)
          如果是则进入自转模式 CLEARING
          否则应该会等待MoveBase::executeCycle局部规划，再次唤醒全局规划
        */

        // 检查时间和失败次数是否超过限制(对于同一个目标点)，若其中一项不满足限制，停止全局规划，进入恢复行为模式
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_)))
        {
          // 进入障碍物清理模式
          state_ = CLEARING;
          //全局规划标志位置为假
          runPlanner_ = false;  
          // 让机器人不要动
          publishZeroVelocity();
          // 全局规划异常导致回复行为，进入自转模式
          recovery_trigger_ = PLANNING_R;
        }
          
        lock.unlock();
      }

      // take the mutex for the next iteration
      // 加锁，下次循环中解锁
      lock.lock();

      // 如果还没到规划周期则定时器睡眠，在定时器中断中 通过planner_cond_唤醒，这里规划周期为0
      // 如果全局规划的频率大于0
      if(planner_frequency_ > 0)
      {
        // notice: 如果程序走得快但是没有到下一个周期，就等待，planner_frequency_越大，等待的时间就越短，执行全局规划的次数就越多，频率越高
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        // 如果等待的时间大于0
        if (sleep_time > ros::Duration(0.0))
        {
          // 告诉线程等待被唤醒
          wait_for_wake = true;
          // 以固定的时间调用回调函数，回调函数中开启全局路径规划子线程
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }

    }
  }

  /*

  该函数流程是：
    1.第一次进入后接收goal，判断有效性等，
    2.然后开启规划线程得到路径。
    3.全局规划开始后，如果节点正常循环执行以下:
      期间会不断检测是否有新的goal抢占，或者目标点的坐标系是不是和全局坐标系一致，
      如果有则在while循环中重复步骤1、2，但如果有被 空抢占（如cancel等）则清除退出
      调用executeCycle局部规划，如果完成则退出
    4.如果节点不正常，则重新启动全局路径规划器
  */
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    // step1 如果目标点朝向的四元数不合法，退出该函数
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation))
    {
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    // 判断goal的有效性，将目标位置转换到全局坐标系T_global
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
    // 发布零速度
    publishZeroVelocity();
    // 现在我们有了目标点，开始路径规划
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    // 全局规划标志位设为真
    runPlanner_ = true;
    // step2 由于全局规划器线程绑定的函数plannerThread()里有planner_cond_对象的wait函数，在这里调用notify启动全局规划器线程，进行全局路径规划
    planner_cond_.notify_one();// 唤醒全局路径规划线程
    lock.unlock();
    // 发布目标到current_goal话题上
    current_goal_pub_.publish(goal);
    // 设置局部规划频率
    ros::Rate r(controller_frequency_);
    // 如果代价地图被关闭，则打开
    if(shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    // 上一次的局部规划时间更新为当前时间
    last_valid_control_ = ros::Time::now();
    // 上一次的全局规划时间更新为当前时间
    last_valid_plan_ = ros::Time::now();
    // 上一次的震荡重置时间更新为当前时间
    last_oscillation_reset_ = ros::Time::now();
    // 对同一目标的全局规划次数记录归为0
    planning_retries_ = 0;

    ros::NodeHandle n;

    // step3 全局规划开始后，循环执行以下:
    while(n.ok())
    {
      if(c_freq_change_)
      {
        // 更改控制频率
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }
      /*
      这里需要进行判断：
        ① 如果action的服务器被抢占，可能是“局部规划进行过程中收到新的目标”，也可能是“收到取消行动的命令”。
          case 1：如果是收到新目标，那么放弃当前目标，重复上面对目标进行的操作，使用新目标。并重新全局规划；
          case 2：如果是收到取消行动命令，直接结束返回。
        ② 如果服务器未被抢占，或被抢占的if结构已执行完毕，接下来开始局部规划
      */
      // 如果action的服务器被抢占
      if(as_->isPreemptRequested())
      {
        // 如果新目标有效
        if(as_->isNewGoalAvailable())
        {
          // 如果获得了新目标，接收并存储新目标，并将上述过程重新进行一遍
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
          // step1 判断目标点有效性
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          // T_global
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          // 为下个执行循环重置状态
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          // 现在有新的目标点，唤醒全局路径规划的线
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          // 发布该目标点给rviz
          current_goal_pub_.publish(goal);
          // 相关的计时变量也要更新
          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else// 否则，服务器的抢占是由于收到了取消行动的命令
        {
          //if we've been preempted explicitly we need to shut things down
          // 上述的case 2：重置导航各个部分的状态 重置服务器状态
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          // 设置当前的 goal为被强占状态
          as_->setPreempted();

          // 取消命令后，返回
          return;
        }
      }
      // notice: 服务器接收到目标后，没有被新目标或取消命令抢占，执行以下

      // step 3.2 检查目标是否被转换到全局坐标系（/map）下
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
      {
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        // 进入planning状态，准备下个执行周期
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        // 有了新的目标点，所以需要确认路径规划器是唤醒的
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      // 记录开始局部规划的时刻为当前时间
      // 开始为executeCycle函数计时，walltime计算的是客观的真实时间，而不是仿真里的时间
      ros::WallTime start = ros::WallTime::now();

      // step 3.3 notice: 
      // 利用局部路径规划器直接输出轮子速度，来控制机器人按照路径走到目标点，成功返回真，否则返回假
      bool done = executeCycle(goal);

      // 任务完成，则退出，
      if(done)
        return;

      // 记录从局部规划开始到这时的时间差
      ros::WallDuration t_diff = ros::WallTime::now() - start;
      // 打印用了多长时间完成操作
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());
      // 用局部规划频率进行休眠
      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }
    // step4 如果节点不正常，则重新启动全局路径规划器：
    lock.lock();
    runPlanner_ = true;
    //  唤醒全局路径规划的线程方便退出
    planner_cond_.notify_one();
    lock.unlock();

    // 如果节点被关闭了，那么Action服务器也关闭并返回
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }
  // executeCycle函数的作用是进行局部规划
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal)
  {
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    // 刚创建时是空的，用于发布速度命令
    geometry_msgs::Twist cmd_vel;

    // 获取机器人的当前位姿(全局坐标系下)
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;

    
    // 产生连续反馈：feedback指的是从服务端周期反馈回客户端的信息，把当前位姿反馈给客户端
    move_base_msgs::MoveBaseFeedback feedback;
    // 更新反馈
    feedback.base_position = current_position;

    // 服务端处理部分数据完后，发送响应反馈给客户端
    as_->publishFeedback(feedback);

    // 如果长时间内移动距离没有超过震荡距离，那么认为机器人在震荡（长时间被困在一片小区域），进入恢复行为
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      // 把最新的振荡重置设置为当前时间
      last_oscillation_reset_ = ros::Time::now();
      // 振荡位姿设为当前姿态
      oscillation_pose_ = current_position;

      // 如果我们上一次的恢复行为是由振荡引起，我们就重新设置恢复行为的索引
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    // 若检查局部规划的地图是否不是当前的(不够新)，则让机器人停止运动，退出函数，如果数据不够新，机器人会乱撞
    if(!controller_costmap_ros_->isCurrent())
    {
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    // 如果有了新的全局路径，把它传给局部规划器
    if(new_global_plan_)
    {
      // 新全局路径标志位置为false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      // 全局路径的指针交换，controller_plan_和latest_plan_(全局路径)
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      // 全局路径赋给局部路径
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");
      // 把全局规划的结果传递给局部规划器，如果传递失败，退出并返回。
      if(!tc_->setPlan(*controller_plan_))
      {
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        // 关闭全局规划线程
        lock.lock();
        runPlanner_ = false;
        lock.unlock();
        // 停止Action服务器，打印“将全局规划传递至局部规划器控制失败”
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      // 如果我们找到有效的 规划路线，且恢复行为是“全局规划失败”，重置恢复行为索引
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    // notice: 如果全局规划失败(state==clearing)、全局规划成功(state==controlling)，都执行以下

    /*
    一般默认状态或者接收到一个有效goal时是PLANNING，在规划出全局路径后state_会由
    PLANNING->CONTROLLING，如果规划失败则由PLANNING->CLEARING
    分别对这三种状态做处理：
      ① PLANNING：全局规划还没完成，还没得到一个全局路线，那么唤醒一个全局规划线程去制定全局路线
      ② CONTROLLING：全局规划成功，得到全局路线，这里进行真正的局部规划：
        如果已经位于终点，结束局部规划；
        如果没到终点，检查机器人是否被困住(处于振荡状态)，如果是，则进入恢复行为；
        如果没到终点，且状态正常：
          进行局部路径规划，如果成功得到速度则直接发布到cmd_vel，
          如果失败则判断是否控制超时，不超时的话让全局再规划一个路径。超时则进入恢复行为
      3.如果出现了问题需要CLEARING（仅有全局规划失败、局部规划失败、长时间困在一片小区域三种原因）
        则每次尝试一种recovery方法，直到所有尝试完
    */
    switch(state_)
    {
      //if we are in a planning state, then we'll attempt to make a plan
      // step1 如果是全局路径规划状态，计算路径
      case PLANNING:
        {
          // 加锁，唤醒全局路径规划器线程
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      // step2 如果是控制状态，尝试计算出有效的下发速度命令
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        // 循环判断，局部规划器，是否到了目标点
        if(tc_->isGoalReached())
        {
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          // 关闭全局路径规划线程
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();
          // ROS_INFO("YES Goal reached !!!");
          // 终端打印
          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }
        /*
        检查是否满足震荡条件，如果满足条件则震荡
        last_oscillation_reset_在一下几种情况都会被更新，
        1. 获得新目标
        2. 距离超过震荡距离（默认0.5）
        3. 进行recovery后
        4. 执行executeCb时，全局路径和局部路径有效时
        较长时间没有发生以上情况会触发震荡，防止长时间卡在一个地方。
        oscillation_timeout_默认为0。
        */
        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          // 如果振荡状态超时了，发布0速度
          publishZeroVelocity();
          // 恢复行为
          state_ = CLEARING;
          // 恢复行为触发器置为，长时间困在一片小区域
          recovery_trigger_ = OSCILLATION_R;
        }
        
        // 如果一切正常没有震荡，开始局部轨迹规划
        {
          boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
          // notice: 如果计算速度指令成功，局部规划器(DWA、TEB)的核心函数,该函数计算本次循环的下发速度
          // notice: 给定一个目标点后正常情况下，全局路径规划只执行一次，局部路径规划循环执行多次
          if(tc_->computeVelocityCommands(cmd_vel))
          {
            ROS_INFO("local plan success send velocity !!!");

            ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                            cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
            // 若成功计算速度，上一次有效局部控制的时间设为当前
            last_valid_control_ = ros::Time::now();
            // notice: 向底盘发送速度控制消息，一个循环只发一次速度命令，不断发布速度指令
            vel_pub_.publish(cmd_vel);
            // 如果恢复行为触发器值是局部规划失败，把索引置0
            if(recovery_trigger_ == CONTROLLING_R)
              recovery_index_ = 0;
          }
          else // 如果局部轨迹规划失败
          {
            ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
            ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

            //check if we've tried to find a valid control for longer than our time limit
            // 判断控制命令的计算局部规划时间有没有超时
            if(ros::Time::now() > attempt_end)
            {
              // 进入障碍物清除模式(一种恢复行为)
              publishZeroVelocity();
              state_ = CLEARING;
              recovery_trigger_ = CONTROLLING_R;
            }
            else// 没超时则重新全局规划，再进行下一次局部规划。
            {
              // 如果没有超时，但是没有找到有效全局路径，返回重新启动全局路径规划器线程重新规划
              last_valid_plan_ = ros::Time::now();
              planning_retries_ = 0;
              state_ = PLANNING;
              publishZeroVelocity();

              // enable the planner thread in case it isn't running on a clock
              // 开启全局规划的线程
              boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
              runPlanner_ = true;
              planner_cond_.notify_one();
              lock.unlock();
            }
          }
        }

        break;

      // step3 用提供的恢复行为来清理空间，主要有三个恢复行为
      // notice: 如果全局规划失败，进入了恢复行为状态，我们尝试去用用户提供的恢复行为去清除空间
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        // 不管什么恢复行为，只要使能了，就唤醒执行
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

          move_base_msgs::RecoveryStatus msg;
          msg.pose_stamped = current_position;
          msg.current_recovery_number = recovery_index_;
          msg.total_number_of_recoveries = recovery_behaviors_.size();
          msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

          recovery_status_pub_.publish(msg);
          // 恢复
          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
           // 更新震荡的计时时间
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          // 检查恢复行为是否有效
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          // 返回全局规划状态
          state_ = PLANNING;

          // 更新到下个恢复行为的索引，尝试每一个恢复行为
          recovery_index_++;
        }
        else// 如果恢复失败
        {
          // 打印“所有的恢复行为都失败了，关闭全局规划器”
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          // 关闭全局规划节点
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");
          // 反馈失败的具体信息
          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        // disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        // 关闭全局规划线程
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    // 如果没有到达终点，每循环一次，就经过这里返回一次false，
    // 如果到达终点了就不会经过这里返回false了
    //ROS_INFO(" local plan break !!!");
    return false;
    /*
    总结：
    在Movebase主体中，各层地图的更新被启动，Action的回调函数触发全局规划线程，若成功，则将全
    局规划结果传入局部规划器，进行局部规划，得到速度指令，控制机器人前进，直到到达目标。
    其间，需要判断机器人是否到达终点（若是则规划停止）、机器人是否状态异常如发生震荡行为（若
    是则进入恢复行为）、机器人是否超时（若是则停止规划发布零速，否则重新规划）等等。
    这个主体是一个大的调用框架，保证了运动规划的正常运行，具体算法在各子过程中分别实现。
    */
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  // 加载默认的恢复行为
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      // step1 加载清除代价地图的恢复行为
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      // step2 加载原地旋转的恢复行为
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_)
      {
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      // step3 加载比较主动积极的代价地图重置行为
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(ags_clear);

      // step4 再来一次原地旋转
      if(clearing_rotation_allowed_)
      {
        recovery_behaviors_.push_back(rotate);
        recovery_behavior_names_.push_back("rotate_recovery");
      }
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  // 重置状态
  void MoveBase::resetState()
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    // 关闭全局规划线程
    runPlanner_ = false;
    lock.unlock();

    // 重置状态机并停止机器人运动
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    // 关闭代价地图
    if(shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  // 获取机器人的位姿，获取的过程不复杂，但是注意这里会检测各种可能异常，并抛出异常。是一个很重要的导航模块是否正常的检测过程
  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // 从代价地图中获得机器人位姿
    try
    {
      // 将robot_pose转换到全局坐标系下，并赋值给global_pose??
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
