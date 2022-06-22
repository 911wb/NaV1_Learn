/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

StaticLayer::StaticLayer() : dsrv_(NULL) {}

StaticLayer::~StaticLayer()
{
  if (dsrv_)
    delete dsrv_;
}

// Costmap2DROS的构造函数会调用各Layer中的initialize函数，
// 而initialize函数会调用onInitialize函数，真正的初始化工作在这里完成。
void StaticLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  // global_frame_在global_costmap时为map, local_costmap时为odom_combined
  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  // 从参数服务器加载订阅静态地图的话题名称；默认静态地图只接收一次，不进行更新；
  // 以及默认追踪未知区域（主地图默认关闭）
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);

  //静态地图层中没有配置，使用默认的
  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  //致命cost 阈值，使用默认的为100
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  //未知的区域，cost 值为255
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));

  // ??三值图：空闲、占据、未知
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  // Only resubscribe if topic has changed
  // 订阅map_topic，map_received_和has_updated_data_标志位设置为false
  // （一旦收到地图，进入回调函数，它们会被置为真），没有接收到地图时，阻塞。
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the SLAM_MAP...................");
    // notice: 订阅slam发布的地图数据，或者是map_server发布的地图数据
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    // 如果map_received_一直是false，则一直阻塞在这里
    // 只有在接收订阅到SLAM静态地图数据后进入回调函数更新为true后才会继续进行
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
    // 判断是否接收static map的更新（默认否），若接收，则开启topic对map_topic + "_updates"的更新
    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayer::incomingUpdate, this);
    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }
  // 最后开启参数动态配置服务。
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}

/*
代价值(栅格值)总结：
  静态地图的代价值：
    1.SLAM_Map的栅格值为-1时，被转换成的代价地图的代价值为255，代表未知区域
    2.SLAM_Map的栅格值为0时，被转换成的代价地图的代价值为0，代表自由区域
    3.SLAM_Map的栅格值为100时，被转换成的代价地图的代价值为254，代表致命障碍栅格
    注意： 代价值： 0 .........255(白色)
          灰度值： 255........0(自由)
          栅格值： 100  -1    0
          占据值： 1..........0
                 占据  未知  空闲
  膨胀区域的代价值：
    距离 == 膨胀半径内的栅格到达(0,0)点的距离
    1.若距离为0，即(0,0)点，则设置代价地图的代价值为254，若此时障碍物与机器人中心重叠，必然发生碰撞；
    2.若距离 ≤ 机器人内切圆半径，则设置代价地图的代价值为253，若此时障碍物处于机器人的内切圆内，必然发生碰撞；
    3.若机器人足迹内切圆半径 < 距离 ≤ cell_inflation_radius_，则以距离远近为比例（指数型）设置代价值：[128,252] = 252*α
      此时障碍物处于其机器人的「外切圆内，内切圆外」，处于碰撞临界，不一定发生碰撞；
*/
unsigned char StaticLayer::interpretValue(unsigned char value)
{
  // 1. 如果阈值lethal_threshold视为障碍物
  // 2. 没有信息的就认为是未探测的区域
  // 3. 否则为自由空间，true, -1

  // SLAM_Map的栅格值为-1时
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;// 255
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  // SLAM_Map的栅格值为100 时，>=100 就是致命的障碍物，会进入这里
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;// 254
  // 如果是三值图，排除以上两种情况就剩一种情况：SLAM_Map数据为-1时
  else if (trinary_costmap_)
    return FREE_SPACE;// 0

  // 如果不是三值图，那就根据比例来计算得分
  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}


// notice: 该回调函数处理SLAM发布的静态地图数据，SLAM发布的地图数据类型：OccupancyGridConstPtr
// 获取接收到的静态地图的尺寸，当地图不随机器人移动时，若接收到的静态地图和主地图的尺寸/分辨
// 率/起点不同，以接收到的地图为准，调整主地图的参数。
// 不断调用回调函数
void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  // 从地图消息中获取地图的宽和高
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  // 返回Costmap2D costmap_的地址
  Costmap2D* master = layered_costmap_->getCostmap();
  //ROS_INFO(" static_layer's rolling window = : %d", layered_costmap_->isRolling());
  // 如果当前slam发布过来的地图数据与上一次的不一样
  if (!layered_costmap_->isRolling() &&
      (master->getSizeInCellsX() != size_x ||
       master->getSizeInCellsY() != size_y ||
       master->getResolution() != new_map->info.resolution ||
       master->getOriginX() != new_map->info.origin.position.x ||// mater grid 的原点不断在变化
       master->getOriginY() != new_map->info.origin.position.y))
  {
    // Resizing costmap to 120 X 114 at 0.050000 m/pix
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    // 若获取SLAM的静态地图尺寸与master地图不一致，则重新设置主地图参数
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y,
                                true /* set size_locked to true, prevents reconfigureCb from overriding map size*/);
  }
  // 若本层的数据和接收到的静态SLAM地图的参数不同时，继续以接收到的地图为准，调整本层静态地图参数。
  // 一开始size_x_为0,所以会进入并仅仅更新static costmap 层
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // Resizing static layer to 122 X 114 at 0.050000 m/pix
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    // 仅更新static costmap 层信息
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  // size_x_，size_y_是从Costmap2D类中继承过来的
  // 此时size_x_，size_y_，resolution_，origin_x_，origin_y_都已经和输入的map消息同步
  }
  unsigned int index = 0;

  // initialize the costmap with static data
  // 根据SLAM发布的地图中的占据栅格值，来更新代价地图的值，具体转换在interpretValue函数
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];

      // notice: SLAM地图传来的占据栅格值为：-1(未知),0(空闲) 和100(致命障碍物)
      // 栅格值转代价值，并初始化master栅格地图的代价值
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  // 有了新地图，更新高和宽
  x_ = y_ = 0;
  width_ = size_x_;// 这个是以像素坐标系(地图左下角)为参考的坐标，是int类型
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // 如果first_map_only_标志位是打开的，关闭map订阅
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}

void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

// 激活静态层，以订阅接收SLAM地图
void StaticLayer::activate()
{
  onInitialize();
}

void StaticLayer::deactivate()
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void StaticLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}
// (*plugin)->updateBounds，多态公有继承，各家对象指针调用虚函数
// 参数 min_x=min_y=1e30，max_x=max_y=-1e30. 没有用到robot_x，robot_y,robot_yaw
void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  // ROS_INFO("updateBounds of static map....");
  // 当前是否是局部代价地图，则不断更新边界，如果全局代价地图那么只更新一次
  if( !layered_costmap_->isRolling() )
  {
    //has_extra_bounds_初始值为false，而在第一次结束前，has_updated_data_被设置为false
    //且订阅话题在第一次接收到消息后关闭，故对于static layer，updateBounds只进行一次
    //Bounds的范围是整张地图的大小，在updateBounds过程中没有对静态地图层做任何的更新
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }

  // 初始化边界
  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;
  /*
  ^
  |
  |
  |
  0----------------->
  */
  // 将map系中的起点（x_， y_）与终点（x_ + width_,， y_ + height_）转换到世界系，
  // 计算静态costmap地图左下角像素的坐标
  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  // 计算静态costmap地图右上角像素的坐标
  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}
// (*plugin)->updateCosts，多态公有继承，各家对象指针调用虚函数
void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;
  // 如果是全局静态地图那么直接将静态地图层bound范围内的内容合并到master地图，因为二者的尺寸也一样
  if (!layered_costmap_->isRolling())
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    // notice: 静态costmap 会进入这里面，会获取static costmap 的cost 值，然后更新master costmap
    //  static costmap 的cost 值是在incomingMap函数中处理初始化的，该函数在costmap_2d 中实现
    if (!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else// 如果局部静态代价地图 rolling window, master_grid和这个局部layer层的坐标系不一样了
  {
    // 若为rolling地图，则找到静态地图和global系之间的坐标转换，通过master地 -> global -> 静态地图的转换
    // 过程，找到主地图的cell在静态地图上对应的cost，赋值给主地图。
    unsigned int mx, my;
    double wx, wy;
    // 可能不在同个坐标系
    geometry_msgs::TransformStamped transform;
    try
    {
      // T_global2rolling_map
      // 首先获得rolling_map坐标系相对于global坐标系的位置，这个时候rolling_map坐标系随着机器人的运动而运动
      transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);
    // 遍历整个地图
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        // master map静态坐标(i,j)→global坐标(wx,wy)
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        // p_global * T_global2roing_map = p_rolling_map
        p = tf2_transform*p;
        // Set master_grid with cell from map
        // rolling_map坐标系p_rolling_map → 静态地图坐标(mx, my)master_map
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)
            // 将静态地图的数据复制给master_map
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

}  // namespace costmap_2d
