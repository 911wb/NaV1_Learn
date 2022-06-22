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
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) :
    costmap_(),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    minx_(0.0),
    miny_(0.0),
    maxx_(0.0),
    maxy_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
  if (track_unknown)
    costmap_.setDefaultValue(NO_INFORMATION);
  else
    costmap_.setDefaultValue(FREE_SPACE);
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}

/*
这个函数在Costmap2DROS动态配置参数的回调函数ReconfigureCB中被调用，作用是在开启地图更
新线程之前，调用Costmap2D的resizeMap函数，用给定参数重新设置主地图的尺寸、原点、分辨
率，再通过plugin指针调用各层地图的matchSize，使其以上参数和主地图匹配。
*/
void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  size_locked_ = size_locked;
  // 调用costmap_的resizeMap方法
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  // 遍历静态、障碍无、膨胀的插件
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    // 不同的对象指针调用各自的虚函数(多态公有继承)
    (*plugin)->matchSize();
  }
}

/*
这个函数在Costmap2DROS的地图更新线程中被循环调用。它分为两步：第一步：更新bound，即确
定地图更新的范围；第二步：更新cost，更新每层地图cell对应的cost值后整合到主地图上。
*/
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

  // 如果我们使用窗口滚动(如果是局部代价地图)，我们需要使用机器人的位置为更新局部地图原点origin
  if (rolling_window_)
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    // 更新Origin 点位置，入参是局部地图的左下角坐标原点
    costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  if (plugins_.size() == 0)
    return;

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;
  /*
  接下来进行地图更新的第一步：更新bound
  设置好minx_、miny_、maxx_、maxy_的初始值，然后对每一层的子地图调用其updateBounds函
  数，传入minx_、miny_、maxx_、maxy_，函数将新的bound填充进去。
  updateBounds函数在Layer类中声明，在各层地图中被重载，

  Step 1 UpdateBounds 更新每个Layer的范围,StaticLayer只在第一次做更新, 范围是加载的整张
  Map的大小,在UpdateBounds过程中没有对Static层的数据做过任何的修改。ObstacleLayer在这个阶
  段主要的操作是根据传感器信息来更新该层的数据,然后更新Bounds。InflationLayer则保持上一次的
  Bounds。

  第二步使用到的updateCosts函数也是如此。这两个函数的具体内容在各层地图部分详述
  */
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if(!(*plugin)->isEnabled())
      continue;
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;

    // step1 这个阶段会更新每个Layer的范围区域，这样在每个运行周期内减少了数据拷贝的操作时间。
    // 调用虚函数updateBounds
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    
    //
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }
  }

  int x0, xn, y0, yn;
  // 接下来调用Costmap2D类的worldToMapEnforceBounds函数，
  // 将得到的bound转换到地图坐标系。这个函数可以防止转换后的坐标超出地图范围
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);
  // 范围更新
  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0)
    return;
  // 接下来，调用resetMap，将主地图上bound范围内的cell的cost恢复为默认值??
  costmap_.resetMap(x0, y0, xn, yn);

  // 遍历层插件
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if((*plugin)->isEnabled())
      // step2 第二步，调用layer类的虚函数updatecosts
      (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
    /*
    UpdateCosts:将各层数据复制到Master Map
    在(a)中,有三个Layer和Master costmap,StaticLayer和ObstaclesLayer维护自己的栅格地图,但
    inflation Layer并没有自己的栅格。为了更新costmap,算法首先在各层上调用自己的UpdateBounds方法(b)。为
    了决定新的bounds,Obstacles Layer利用新的传感器数据更新它的costmap。然后每个层轮流用
    UpdateCosts方法更新Master costmap的某个区域,从Static Layer开始(c),然后是Obstacles
    Layer(d),最后是inflation Layer(e)。
    layer会被LayeredCostmap调用,用于明确多大的地图范围用来更新。
    */
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  // 
  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    if((*plugin)->isEnabled())
      current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d
