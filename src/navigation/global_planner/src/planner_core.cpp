/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

/*
notice:
是makeplan服务被调用后的回调函数，该函数是导航过程中的主要运行函数，里面包括路径搜索，
路径回溯，添加方向等，发布扩展列表话题“potential”，发布可视化路径话题“plan”。

1.planner_core：初始化对象和参数
2.调用make_plan 调用Dij或者A*计算路径
3.二次逼近计算、势场计算(返回当前点四周最小代价值以及累加前面累积的代价值)，生成搜索路径
4.搜索到路径之后使用回溯法，
    grid_path(从终点开始找上下左右四个中最小的栅格值直到起点)
    gradient_path(从八个栅格中找到梯度最大的点)
5.方向滤波
*/
// 注册插件，基类：nav_core::BaseGlobalPlanner，子类： global_planner::GlobalPlanner
// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

// 将地图外轮廓设为障碍
void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) 
{
    unsigned char* pc = costarr;
    // 第一行
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    // 最后一行
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    // 第一列
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    // 最后一列
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}
// 构造函数
GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true),
        p_calc_(NULL), planner_(NULL), path_maker_(NULL), orientation_filter_(NULL),
        potential_array_(NULL) {
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        GlobalPlanner() {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner() 
{
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}
// 初始化函数1
void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
{
    // 调用初始化函数2
    // 将 Costmap2DROS 转化位 costmap 提取其 frameid
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) 
{
    //判断是否已经初始化完毕
    if (!initialized_)
    {
        //获取参数 节点句柄
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        // 获得代价地图的长和宽
        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
        
        // ture使用原来功能包navfn的规划方式
        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        // true使用二次近似函数，false使用更简单的计算方式节省硬件资源（否的话会扩展周边4个点，是扩展周边八个）
        private_nh.param("use_quadratic", use_quadratic, true);
        // 根据选择new出对应的p_calc_实例. 计算“一个点”的可行性
        if (use_quadratic)
            // 使用二次差值近似的potential，搜索八个节点效果较好 notice: 找终点的两种方法
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            // 搜索周围四个节点来找终点
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        // false则使用astar算法
        private_nh.param("use_dijkstra", use_dijkstra, true);
        // 根据选择new出对应的planner实例. 计算“所有”的可行点
        if (use_dijkstra)
        {
            // dijkstra算法
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            // 是否使用旧版本的计算方式
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            // notice: dijkstra算法：父类指针指向子类对象
            planner_ = de;
        }
        else
            // notice: A*算法：父类指针指向子类对象
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        // 是否沿边界创建路径，true则实例化GridPath，false则实例化GradientPath使用梯度下降算法。
        private_nh.param("use_grid_path", use_grid_path, false);
        // 路径方法，new出path_maker_实例。从可行点中提取路径
        if (use_grid_path)
            // 回溯：栅格路径，从终点开始找八个邻居节点中最小的G直节点，直到找到起点 
            path_maker_ = new GridPath(p_calc_);
        else
            // 回溯：梯度路径，从周围八个栅格中找到下降梯度最大的点，效果较好 notice: 找起点的两种方法
            path_maker_ = new GradientPath(p_calc_);

        // 给路径加方向，全局规划器规划出的只是路径点(x,y)，需要加上Θ角
        orientation_filter_ = new OrientationFilter();
        // 发布路径
        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        // 发布势场可视化，较慢
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        //  true允许规划未知地图（灰色）中的路径，false不允许
        private_nh.param("allow_unknown", allow_unknown_, true);
        // 允许规划 未知（unknow）中的全局路径 
        planner_->setHasUnknown(allow_unknown_);
        // 指定可选窗口的x,y大小以限定规划器的工作空间(程序中未用到)
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        // 允许到达目标点的误差范围
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        // 设定发布可行性点的最大值（astar/Dijkstra搜索算法扩展的栅格点）
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);

        // notice: 而每当我们需要规划路径时调用makeplan服务，GlobalPlanner会调用makePlan()方法也是我们路径规划的核心步骤。
        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
                &GlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}
// 以下使用rqt中的动态参数调节：
void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) 
{
    // int 致命代价
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    // int 中立代价
    planner_->setNeutralCost(config.neutral_cost);
    // uint 将代价图中每个代价乘以的因子（范围3.0-5.0）
    planner_->setFactor(config.cost_factor);
    // true发布可行性点话题"potential",false不发布
    publish_potential_ = config.publish_potential;
    // 枚举 路径反向模式，默认为最简单的Forward
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

// 将某个costmap设置为free
void GlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) 
{
    if (!initialized_) 
    {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

// makeplan 回调函数
bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    // 调用makeplan函数
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}
// 将map上的坐标系转化为world上的坐标系
void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

// 接口
bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

// notice: 两个步骤完成路径的生成：可以修改优化这两个函数
//  1. 计算可行点矩阵potential_array (planner_->calculatePotentials)
//  2. 从可行点矩阵中提取路径plan (path_maker_->getPath)）
bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) 
{
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) 
    {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    // 清除上一次的路径结果
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    // step1 目标点的坐标系应该和全局坐标系一致
    if (goal.header.frame_id != global_frame)
    {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }
    // step2 起始点的坐标系应该和全局坐标系一致
    if (start.header.frame_id != global_frame) 
    {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }
    // 获取起始点的世界坐标系坐标
    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;
    // step3 判断起始点和目标点是否超出了全局代价地图的范围
    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
    {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    // 如果使用旧版本的Dij
    if(old_navfn_behavior_)
    {
        start_x = start_x_i;
        start_y = start_y_i;
    }
    else// 如果使用新版本的Dij
    {
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) 
    {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    // step4 清除起始单元格，它不可能是障碍物
    clearRobotCell(start, start_x_i, start_y_i);
    // 获取代价地图的长宽
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    // step5 确保global_planner用的数组大小和地图一致
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    // 路径回溯
    path_maker_->setSize(nx, ny);

    // 二维矩阵，扩展的可行点
    // 创建数组，存放每个访问过的邻居节点的 G 值
    potential_array_ = new float[nx * ny];

    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);
    // 参数：代价值为0~255的master代价地图、起始点、终点、地图尺寸*2、可行的扩展点数组
    // step6 notice: 核心步骤，不断计算出可行的扩展点数组直到找到目标点，存放在potential_array_，调用Dijkstra或者A*的虚函数
    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    // 是否可视化G值数组
    if(publish_potential_)
        publishPotential(potential_array_);

    // 如果搜索算法是否找到了目标点
    if (found_legal)
    {
        // step7 notice: 通过父节点的G值，从终点到起点「回溯」获得一全局条路径，结果保存在plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) 
        {
            // 更新目标点的时间戳，和路径的其他点时间戳一致
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            // 将终点存到路径点数组容器
            plan.push_back(goal_copy);
        } 
        else
        {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }
    else
    {
        ROS_ERROR("Failed to get a plan.");
    }

    // step8 给路径添加方向，输入：起点、前面生成的一条路径点
    orientation_filter_->processPath(start, plan);

    // 发布路径和可视化
    publishPlan(plan);

    // 内存不能泄露
    delete[] potential_array_;

    return !plan.empty();
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

// 回溯提取路径
bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) 
{
    if (!initialized_)
    {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    // 清空上一次使用完的路径
    plan.clear();
    // 用于存放路径点
    std::vector<std::pair<float, float> > path;

    // 这个path的点只有在map中的位置信息(x,y)
    // notice: 生成路径点数组，path_maker_有两种方案：GradientPath、GridPath
    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) 
    {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();

    // 将path中每个点(局部坐标系)转换到world坐标系下，方向信息还没加入，
    // 这里统一设为零，然后依次存储到plan中
    for (int i = path.size() -1; i>=0; i--)
    {
        // 取出路径中的一个点，第一次取出的是起点
        std::pair<float, float> point = path[i];

        double world_x, world_y;

        // 把map的全局路径转换(局部坐标系)到world坐标系下
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        // 路径点
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        // 路径点的方向初始化为0
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        // 将转换到世界坐标系下的每个点存到plan
        plan.push_back(pose);
    }
    if(old_navfn_behavior_)
    {
        plan.push_back(goal);
    }
    return !plan.empty();
}

void GlobalPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else {
            if (fabs(max) < DBL_EPSILON) {
                grid.data[i] = -1;
            } else {
                grid.data[i] = potential_array_[i] * publish_scale_ / max;
            }
        }
    }
    potential_pub_.publish(grid);
}

} //end namespace global_planner