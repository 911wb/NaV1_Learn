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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_trajectory_generator.h>

#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {
// 产生采样速度
void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    std::vector<Eigen::Vector3f> additional_samples,
    bool discretize_by_time) {
  initialise(pos, vel, goal, limits, vsamples, discretize_by_time);
  // add static samples if any
  sample_params_.insert(sample_params_.end(), additional_samples.begin(), additional_samples.end());
}

/*
首先，计算可行的线速度和角速度范围，这里先对最大线速度进行一个限制，
即保证速度既不超过预设的最大速度限制，也不超过“起点与目标直线距离/总仿真时间”。

生成一系列的速度空间(离散点)，前提是知道机器人的线速度和角速度，离散采样，需要知道采样的间隔和范围

*/
void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,// 速度限制
    const Eigen::Vector3f& vsamples,// 
    bool discretize_by_time) 
{
  
  // We actually generate all velocity sample vectors here, from which to generate trajectories later on
   
  // 声明最大/小线速度，最大/小角速度
  double max_vel_th = limits->max_vel_theta;
  // 旋转速度是有方向的，有最大也有最小，最小为负的最大
  double min_vel_th = -1.0 * max_vel_th;
  discretize_by_time_ = discretize_by_time;
  //三个加速度
  Eigen::Vector3f acc_lim = limits->getAccLimits();
  pos_ = pos;
  vel_ = vel;
  limits_ = limits;
  next_sample_index_ = 0;
  sample_params_.clear();

  double min_vel_x = limits->min_vel_x;
  double max_vel_x = limits->max_vel_x;
  double min_vel_y = limits->min_vel_y;
  double max_vel_y = limits->max_vel_y;

  // x,y方向和角速度的采样个值不能为0
  if (vsamples[0] * vsamples[1] * vsamples[2] > 0) 
  {
    //compute the feasible velocity space based on the rate at which we run
    // 基于运行频率计算可行的速度空间
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();
    // ①  不使用dwa法，则用的是整段仿真时间sim_time_
    if ( ! use_dwa_) // 需要一定时间才能够达到的速度
    {
      // 计算当前位置和目标位置之间的距离 / 仿真时间 ，目的是模拟仿真时间内匀减速到0，刚好到达目标点的情景
      double dist = hypot(goal[0] - pos[0], goal[1] - pos[1]);
      max_vel_x = std::max(std::min(max_vel_x, dist / sim_time_), min_vel_x);
      max_vel_y = std::max(std::min(max_vel_y, dist / sim_time_), min_vel_y);

      // 然后基于acc_lim * sim_time得到一种边界，还有设置的速度参数限制作为一种边界，
      // 选取边界中空间较小的边界。这种策略，能够获得较大的采样空间（因为用了sim_time == 1.7）?
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_);
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);
      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_time_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_time_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);
    }
    // ② 使用dwa法，则用的是轨迹前向模拟的周期sim_period_（专用于dwa法计算速度的一个时间间隔）
    else // 瞬时才能够达到的速度
    { 
      // sim_period_ == 0.1
      // DWA方法，直接用acc_lim * sim_period得到边界，还有设置的速度参数限制作为边界，然后选取两种边界中空间较小的边界
      // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);
    }

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    // 在一定的范围之内生成多少样本
    VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);
    VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
    VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);
    // 得到速度空间边界后，根据x，y，theta三个采样个数进行插补，进而组合出整个速度采样空间
    for(; !x_it.isFinished(); x_it++) 
    {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) 
      {
        vel_samp[1] = y_it.getVelocity();
        for(; !th_it.isFinished(); th_it++) 
        {
          // 每三个速度为一组
          vel_samp[2] = th_it.getVelocity();
          //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
          // 储存速度采样空间(小窗口)中的每个采样点，一个采样点有三个速度(vx,vy,vΘ)
          sample_params_.push_back(vel_samp);
        }
        th_it.reset();
      }
      y_it.reset();
    }
  }
}

void SimpleTrajectoryGenerator::setParameters(
    double sim_time,
    double sim_granularity,
    double angular_sim_granularity,
    bool use_dwa,
    double sim_period) {
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  use_dwa_ = use_dwa;
  continued_acceleration_ = ! use_dwa_;
  sim_period_ = sim_period;
}

/**
 *  产生器是否可以生成多个局部路径
 */
bool SimpleTrajectoryGenerator::hasMoreTrajectories() {
  return next_sample_index_ < sample_params_.size();
}

/**
 * 产生和返回下个采样局部路径
 */
bool SimpleTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories())
  {
    // 产生轨迹
    if (generateTrajectory(
        pos_,// 机器人当前位姿
        vel_,// 机器人当前速度
        sample_params_[next_sample_index_],// 采样空间中的一个速度组合
        comp_traj))// 储存当前采样速度组合下的轨迹
    {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

/**
 * @param pos 机器人当前位姿
 * @param vel 机器人当前位姿
 */
// 这个函数根据给定的速度和角速度采样生成单条路径和其代价。
bool SimpleTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,// 机器人当前位姿
      Eigen::Vector3f vel,// 机器人当前速度
      Eigen::Vector3f sample_target_vel,// 采样空间中的一个速度组合
      base_local_planner::Trajectory& traj)// 输出一条轨迹
{
  // 非全向机器人 线速度，hypot(x,y) = √x² + y²
  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
  double eps = 1e-4;
  traj.cost_   = -1.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();

  // 先判断速度是否满足下面调节的其中一个
  // 1. 平移速度不小于min_trans_vel且旋转速度不小于min_rot_vel
  // 2.平移速度不大于max_trans_vel
  // 如果满足，则直接返回false，此时，traj为空路径
  // make sure that the robot would at least be moving with one of
  // the required minimum velocities for translation and rotation (if set)
  if ((limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) 
  {
    return false;
  }

  // 如果在x和y方向的速度向量之和的大小超过了速度限制，返回false
  if (limits_->max_vel_trans >=0 && vmag - eps > limits_->max_vel_trans) 
  {
    return false;
  }
  
  // 确定仿真步数 计算仿真步数和每一步对应的时间，朝向打分与否对应的步数计算方法略有不同。
  int num_steps;
  // 根据时间来离散
  if (discretize_by_time_) 
  { 
    // sim_granularity_：仿真点之间的距离间隔，生成的轨迹是基于等时间步长
    // 步数 = 总仿真时间/距离间隔，四舍五入
    num_steps = ceil(sim_time_ / sim_granularity_);
  } 
  else
  {
    //compute the number of steps we must take along this trajectory to be "safe"
    // 由当前速度推出在sim_time_内走过的距离
    double sim_time_distance = vmag * sim_time_; 
    // 由当前速度推出在sim_time_内转过的角度
    double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_;
    // 步数 = max(速度模×总仿真时间/距离间隔，角速度/角速度间隔)，四舍五入
    num_steps =
        ceil(std::max(sim_time_distance / sim_granularity_,
            sim_time_angle    / angular_sim_granularity_));
  }
  // 步数不能为零
  if (num_steps == 0) 
  {
    return false;
  }

  // Δt = 每一步的时间 = 总仿真时间/步数
  double dt = sim_time_ / num_steps;
  traj.time_delta_ = dt;

  Eigen::Vector3f loop_vel;

  // 是否使用最大加速度 continued_acceleration_ = ! use_dwa_;
  if (continued_acceleration_) 
  {
    // use_dwa==false，则采用连续加速的策略
    // 即仿真出的轨迹中不同点对应的速度是变化的，此时将轨迹中保存的对应速度设为基于当前速度第一次加速出的速度。
    // 否则，轨迹中的各个点为同样的速度，即sample_target_vel，此时轨迹中保存的速度也是该速度
    loop_vel = computeNewVelocities(sample_target_vel, 
    vel,// 机器人当前速度 
    limits_->getAccLimits(),// 速度限制 
    dt);// 每一步的时间
    
    // 使用采样空间中的速度组合或者最大加速度得到的速度，作为当前轨迹的驱动速度
    traj.xv_     = loop_vel[0];
    traj.yv_     = loop_vel[1];
    traj.thetav_ = loop_vel[2];
  }
  else // 恒速模型
  {
    // 如果use_dwa==true，用恒速计算轨迹点位姿
    // 保存采样空间中的这一组速度
    loop_vel = sample_target_vel;

    // 保存采样空间中的速度组合，作为当前轨迹的驱动速度
    traj.xv_     = sample_target_vel[0];
    traj.yv_     = sample_target_vel[1];
    traj.thetav_ = sample_target_vel[2];
  }

  // 遍历步数(Δt的个数)
  for (int i = 0; i < num_steps; ++i)
  {

    // 第一次添加的是机器人当前位姿，不断的储存下一时刻机器人的预测位姿，就组成的机器人的预测轨迹
    traj.addPoint(pos[0], pos[1], pos[2]);

    // 是否需要使用最大加速度来持续加速
    if (continued_acceleration_)
    {
      // 计算下一时刻的速度，速度会由于加速度不断的增大，导致每个路径点的对应速度都不一样
      loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
      //ROS_WARN_NAMED("Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
    }

    // 恒速模型：使用采样空间的一组速度来当做一条轨迹中所有路径点的速度
    // 计算下一时刻机器人的位姿，参数：当前机器人位姿、采样空间的一组速度(下一时刻的速度)、每一步的时间(时间间隔)
    pos = computeNewPositions(pos, loop_vel, dt);

  } // 仿真阶段结束

  return true; // 局部路径至少要包含一个点
}

Eigen::Vector3f SimpleTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */
Eigen::Vector3f SimpleTrajectoryGenerator::computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
    const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt) 
{
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();
  // 遍历x、y、Θ三个变量的速度
  for (int i = 0; i < 3; ++i)
  {
    // 如果机器人的速度小于采样速度
    if (vel[i] < sample_target_vel[i]) 
    {
      // 下一时刻的速度为 采样速度和当前机器人速度+Δv，中最小的
      new_vel[i] = std::min(double(sample_target_vel[i]), vel[i] + acclimits[i] * dt);
    } 
    else // 如果机器人的速度大于采样速度
    {
      new_vel[i] = std::max(double(sample_target_vel[i]), vel[i] - acclimits[i] * dt);
    }
  }
  return new_vel;
}

} /* namespace base_local_planner */
