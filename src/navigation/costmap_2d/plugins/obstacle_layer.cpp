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
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstacleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

/*
障碍层地图通过订阅传感器话题，将传感器输出的障碍物信息存进buffer（剔除过高、过远的点），
在本层地图上将观测到的点云标记为障碍物，将传感器到点云点连线上的点标记为FREE_SPACE
最后，在bound范围内，将本层地图合并到主地图上。

*/
/*
首先从参数服务器加载参数，确定rolling_window_、track_unknown_space的值，
并调用matchSize函数，根据主地图的参数来设置障碍层地图。
*/

void ObstacleLayer::onInitialize()
{

  // 创建多个节点句柄nh("~/" + name_)、g_nh、prefix_nh和source_node(nh, source)，
  // 通过方法param(param_name, param_val, default_val)从参数服务器检索指示的值param_name,
  // 若成功，则将结果存储在param_val中，否则将default_val赋值给param_val。
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();
  // ROS_INFO(" obtstace_layer's rolling_window_ = : %d", rolling_window_);

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  // 设置障碍物层的代价值、x值、y值、分辨率和原点坐标
  ObstacleLayer::matchSize();// 子类调用父类虚函数
  current_ = true;
  // 全局坐标系
  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  // 从参数服务器获取所订阅的话题
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;

  // 从参数服务器加载订阅的话题名称，即障碍物(观测)信息的来源。
  nh.param("observation_sources", topics_string, std::string(""));
  // scan
  ROS_INFO("Subscribed observation_sources Topics: %s", topics_string.c_str());

  // now we need to split the topics based on whitespace which we can use a stringstream for
  // 使用字符串流,方便用空格分开话题
  std::stringstream ss(topics_string);

  std::string source;

  // 可能有多个激光雷达
  // 接下来进入循环，将topics_string记录的观测源逐个输出到source字符串，并找到对应每个观测源的参数
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // 从特定话题获取参数
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);

    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
    {
      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
    }

    std::string raytrace_range_param_name, obstacle_range_param_name;

    // 确定传感器所能探测到障碍物的最大范围值
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // 确定传感器所能感知到周围环境的最大范围值
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());
    // 为每个观测源创建一个buffer，并将其观测值指针存放进observation_buffers_中进行管理。
    // 并根据标志位确定是否将其添加到marking_buffers与clearing_buffers中。
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance)));

    // 检查是否将此缓冲区添加到标记观察缓冲区
    // 此时marking_buffers_虽添加了观察区的数据，但还未调用传感器数据回调函数，其值为空值
    if (marking)
      // 储存观测缓存，标记障碍
      marking_buffers_.push_back(observation_buffers_.back());

    // 检查是否还要将此缓冲区添加到清除观察缓冲区
    if (clearing)
      // 储存观测缓存，清除障碍
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic
    // 分别针对不同的sensor类型（LaserScan、PointCloud、PointCloud2）
    // 如LaserScan PointCloud等注册不同的回调函数(用于传感器数据的处理)，如针对LaserScan的回调函数为laserScanCallback
    if (data_type == "LaserScan")
    {
      // notice: 话题通信，订阅连续型消息类型：sensor_msgs::LaserScan、话题：scan，初始化智能指针sub
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));
      // 初始化filter指针：消息过滤器，过滤缓存消息
      boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan> > filter(
        new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, g_nh));

      if (inf_is_valid)
      {
        filter->registerCallback(boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1,
                                            observation_buffers_.back()));
      }
      else
      {
        // notice: 回调函数laserScanCallback，调用处理雷达数据的回调函数
        filter->registerCallback(boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));
      }
      // 对观测信息进行过滤，可能有多个激光雷达，使用一个容器来储存多个订阅方
      observation_subscribers_.push_back(sub);
      // 确保各个sensor坐标变换有效
      observation_notifiers_.push_back(filter);

      observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
    }
    else if (data_type == "PointCloud")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

        boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud>
        > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50, g_nh));
        filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }
    else
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud2>
      > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, g_nh));
      filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "")
    {
      std::vector < std::string > target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void ObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
      &ObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

ObstacleLayer::~ObstacleLayer()
{
    if (dsrv_)
        delete dsrv_;
}
void ObstacleLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  combination_method_ = config.combination_method;
}

/*
这里以LaserScan数据的回调函数为例，先将收到的message的laser数据转换为sensor_msgs::PointCloud2格式的数据
再调用ObservationBuffer类的bufferCloud函数，将点云数据存到buffer中。
*/
void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // 创建储存点云容器
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  // 一帧激光数据msg转换成点云数据，存到cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
    projector_.projectLaser(*message, cloud);
  }
  catch (std::runtime_error &ex)
  {
    ROS_WARN("transformLaserScanToPointCloud error, it seems the message from laser sensor is malformed. Ignore this laser scan. what(): %s", ex.what());
    return; //ignore this message
  }

  // 缓冲点云信息
  buffer->lock();
  // ObservationBuffer类是专门用于存储观测数据的类，它是ObstacleLayer的类成员。
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void ObstacleLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // 把正的无限值转换成max_range
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[ i ];
    if (!std::isfinite(range) && range > 0)
    {
      message.ranges[ i ] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try
  {
    // projector_是LaserProjection的实例，laserProjection也是ros中的库，不在navigation代码中
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
             global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  }
  catch (std::runtime_error &ex)
  {
    ROS_WARN("transformLaserScanToPointCloud error, it seems the message from laser sensor is malformed. Ignore this laser scan. what(): %s", ex.what());
    return; //ignore this message
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

// 处理缓冲点云pointcloud消息的回调函数
void ObstacleLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                               const boost::shared_ptr<ObservationBuffer>& buffer)
{
  sensor_msgs::PointCloud2 cloud2;

  if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
  {
    ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud2);
  buffer->unlock();
}

// 处理缓冲点云pointcloud2消息的回调函数
void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

/*


传入参数 robot_x、robot_y和robot_yaw 机器人位姿 min_y、min_y、max_x和 max_y用于 更新边界轮廓
这个函数主要完成：clearing、marking以及确定bound。
*/

void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  // 判断是否调用滚动窗口(当前是否是局部代价地图)，若是，则进行更新原点坐标
  // updateOrigin() 确定更新边界轮廓的原点坐标
  // getSizeInMetersX() 返回 （size_x_ - 1 + 0.5) * resolution_
  // getSizeInMetersY() 返回 （size_y_ - 1 + 0.5) * resolution_
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  // 确定更新边界轮廓尺寸
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  // 在接收到传感器数据后，buffer将被更新，同样，marking_buffers_和clearing_buffers_也更新（即buffer中的内容）
  
  // 容器中的每个元素代表一帧点云的数据
  std::vector<Observation> observations, clearing_observations;

  // 获取标记观测，更新observations
  current = current && getMarkingObservations(observations);

  // 获取清除观测，更新clearing_observations
  current = current && getClearingObservations(clearing_observations);

  // 更新全局当前状态
  current_ = current;

  // 接下来是第一步，对clearing_observations中的点云点执行clearing操作，即将其与传感器的连线上的点标记为FREE_SPACE。
  // 遍历clearing_observations中的所有帧点云
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    // 将传感器与被测物之间的连线经过的栅格，标记为FREE_SPACE
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }
  /*
  marking操作，即将点云中的点标记为障碍物， 将新障碍物放入优先级队列...每个都以零优先级开始
  在标记时通过二重循环，外层迭代各观测轮次，内层迭代一次观测得到的点云点，剔除本身太高（z
  坐标过大）、与传感器距离太远的点，将符合要求的障碍点坐标从global系转换到map系，
  并在本层地图上标记致命障碍。并调用touch函数，确保标记的障碍点包含在 bound内。
  然后开始进入mark操作，对于每个测量到的点，标记为obstacle
  即迭代点云中的点，本身太高太远的障碍和跟其他障碍点离得太远的点都不考虑
  */
  // 遍历每一轮观测中的所有帧点云
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;
    // 获取一帧点云数据
    const sensor_msgs::PointCloud2& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    // 遍历一帧点云中的所有点
    for (; iter_x !=iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too high or too far away from the robot we won't add it
      // 如果障碍物离机器人太高，将不被考虑在内
      if (pz > max_obstacle_height_)
      {
        ROS_DEBUG("The point is too high");
        // 跳过当前点
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      // 计算从命中点到点云原点的平方距离
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // 如果该点(障碍物)足够远，将不会考虑
      if (sq_dist >= sq_obstacle_range)
      {
        ROS_DEBUG("The point is too far away");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      // 获得障碍点在地图坐标系上的坐标，如果不在则跳过
      if (!worldToMap(px, py, mx, my))
      {
        ROS_DEBUG("Computing map coords failed");
        continue;
      }
      // 获取当前有效障碍点的一维坐标
      unsigned int index = getIndex(mx, my);
      // 将有效障碍点的一维坐标对应栅格的代价值设置为致命
      costmap_[index] = LETHAL_OBSTACLE;
      // 更新参数中指定的边界框以包括位置(x,y)，确保标记的障碍点包含在 bound内
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  // 它的作用是基于机器人当前位置确定该位置下的足迹，并在内部调用touch函数保证足迹包含在bound范围内。
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

}

// 更新机器人的footprint
void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

/*
(*pugin)->updateCosts 更新障碍地图代价更新障碍地图代价
这个函数就是将机器人足迹范围内设置为FREE_SPACE，并且在bound范围内将本层障碍地图的内容合并到主地图上

*/
void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (footprint_clearing_enabled_)
  {
    // 设置机器人所在区域为FREE_SPACE
    // 把机器人轮廓多边形边缘及内部的所有cell的cost设置为FREE_SPACE
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  // 更新master_grid的方法(在bound范围内将本层障碍地图的内容合并到主地图上)
  switch (combination_method_)
  {
    case 0:  // Overwrite
      // 覆盖更新，地图层的每个有效值（不含未知状态（NO_INFORMATION））写进master_grid
    	// master_grid：地图层的特定边界矩形的主栅格对象，
    	// min_i、min_j: 特定边界矩形的左下角顶点的x、y坐标
    	// max_i、max_j: 特定边界矩形的右上角顶点的x、y坐标
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      // 最大值更新，主栅格的新值取master_grid中各层中的最大值
      // 特殊情况:
    	// a) 如果主代价值是未知状态（NO_INFORMATION），则被覆盖更新
    	// b) 如果所操作的地图层的代价值是未知状态（NO_INFORMATION），则主代价值不变
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

// 添加静态观察
void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

// 清除静态观察
void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

// 获取标记观察
bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

// 获取清除观察，与标记观察是相对的
bool ObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // 获取用于清除的观察
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

// 清理传感器到障碍物间的cell(将栅格设置为空闲状态，与cartographer类似)
void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{

  // 一帧点云的原点坐标
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  // 当前这一帧点云
  const sensor_msgs::PointCloud2 &cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  // 获取传感器原点的地图坐标
  unsigned int x0, y0;

  if (!worldToMap(ox, oy, x0, y0))
  {
    // 处理越界问题
    ROS_WARN_THROTTLE(
        1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy);
    return;
  }

  // 地图起点
  double origin_x = origin_x_, origin_y = origin_y_;
  // 地图终点
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  // 保证传感器原点在bound范围内
  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  // 点云中的每个点与原点建一条线，如果有障碍物在这条线上，则被清除. 障碍物层是二维的，所以没有z方向
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  // 遍历一帧中的每一个点云
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
  {
    // wx wy是当前点云中的点的坐标
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    // 确保光线跟踪到的点没有在代价地图外，如果有必要，做尺度缩放
    // a、b是该点跟传感器原点的距离
    double a = wx - ox;
    double b = wy - oy;

    // 光线追踪最小值是原点
    // 如果当前点x方向比地图原点还小
    if (wx < origin_x)
    {
      // t（比例）=（地图原点-传感器原点）/（点云中的该点-传感器原点）
      double t = (origin_x - ox) / a;
      // 当前点云中的点的坐标x = 地图原点x
      wx = origin_x;
      // 实际上还是把点云点和传感器连线之间清空，只是通过相似三角形丢弃了超出地图原点范围外的部分，下面三个判断结构同理
      wy = oy + b * t;
    }
    if (wy < origin_y)
    {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // 光线追踪的最大值是地图终点
    if (wx > map_end_x)
    {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    // 成适当尺寸后，获得终点在map下的坐标
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    // 
    MarkCell marker(costmap_, FREE_SPACE);

    // 最终可以执行光线追踪，沿着光线清理传感器原点和障碍物点之间的cell(设置为空闲状态)
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);
    // 用来根据测量的距离，更新扩张，用是保证连线上的一点（距离传感器特定范围内）被包含进bound
    updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}
void ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

}  // namespace costmap_2d