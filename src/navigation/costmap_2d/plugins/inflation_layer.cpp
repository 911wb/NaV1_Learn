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
#include <algorithm>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

InflationLayer::InflationLayer()
  : resolution_(0)
  , inflation_radius_(0)
  , inscribed_radius_(0)
  , weight_(0)
  , inflate_unknown_(false)
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , dsrv_(NULL)
  , seen_(NULL)
  , cached_costs_(NULL)
  , cached_distances_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
  inflation_access_ = new boost::recursive_mutex();
}

void InflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    need_reinflation_ = false;

    dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb = boost::bind(
        &InflationLayer::reconfigureCB, this, _1, _2);

    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
  }

  matchSize();
}

void InflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
{
  setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

  if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown) {
    enabled_ = config.enabled;
    inflate_unknown_ = config.inflate_unknown;
    need_reinflation_ = true;
  }
}

/*
由于InflationLayer没有继承Costmap2D，所以它和静态地图与障碍地图这两层不同
它没有属于自己的栅格地图要维护，所以matchSize函数自然不需要根据master地图的参数来调节本层地图。
这个函数先获取主地图的分辨率，接着调用cellDistance函数，这个函数可以把global系以米为单位的长度
转换成以cell为单位的距离，故获得了地图上的膨胀参数cell_inflation_radius_。
*/
void InflationLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  // master
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  // 获取master地图的分辨率
  resolution_ = costmap->getResolution();
  
  // 把global系以米为单位的长度转换成以cell为单位的距离，通过膨胀半径计算膨胀栅格个数
  cell_inflation_radius_ = cellDistance(inflation_radius_);

  // notice: 两个“参考矩阵”的填充
  computeCaches();

  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  // 根据主地图大小创建seen_数组，它用于标记cell是否已经过计算
  seen_ = new bool[seen_size_];
}

// need_reinflation_默认false，更新bound这里和其他两层的主要区别是，膨胀层在传入的bound的值的基础上，通过inflation_radius_再次扩张
void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (need_reinflation_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
  else
  {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // 膨胀层在传入的bound的值的基础上，通过inflation_radius_再次扩张。
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void InflationLayer::onFootprintChanged()
{
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  need_reinflation_ = true;

  ROS_DEBUG("InflationLayer::onFootprintChanged(): num footprint points: %lu,"
            " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
            layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

// 用指针master_array指向主地图，并获取主地图的尺寸，确认seen_数组被正确设置
void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  if (cell_inflation_radius_ == 0)
    return;

  // 循环前膨胀层的list应该为空
  ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");
  // master地图的数组
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_ == NULL)
  {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  else if (seen_size_ != size_x * size_y)
  {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
    delete[] seen_;
    seen_size_ = size_x * size_y;
    // 这里分配了一个size和master map一样的bool数组
    seen_ = new bool[seen_size_];
  }
  // 清空
  memset(seen_, false, size_x * size_y * sizeof(bool));

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.

  // 边缘膨胀
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);
  /*
  Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
  We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

  接下来遍历bound中的cell，找到cost为LETHAL_OBSTACLE，即障碍物cell，将其以CellData形式放
  进inflation_cells_[0.0]中，inflation_cells_的定义如下：
  // <栅格到(0,0)的距离，对应的栅格的栅格坐标>
  std::map<double, std::vector<CellData> > inflation_cells_
  它是一个映射，由浮点数→CellData数组，是专门用来记录当前cell的索引和与它最近的障碍物的索引的。
  这个映射以距离障碍物的距离为标准给bound内的cell归类，为膨胀做准备；
  */
  // 障碍物到自身距离为0，目前得到的inflation_cells_只包括障碍物本身
  std::vector<CellData>& obs_bin = inflation_cells_[0.0];
  // 遍历边缘膨胀后的地图
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      // 获取旧地图一维坐标
      int index = master_grid.getIndex(i, j);
      // 获取坐标对应的代价值
      unsigned char cost = master_array[index];
      // 找到cost为LETHAL_OBSTACLE，说明该栅格为致命障碍物栅格，将其放到容器中，距离设置为0
      if (cost == LETHAL_OBSTACLE)
      {
        // 将当前栅格存放到致命栅格容器中，到达(0,0)的距离为0
        obs_bin.push_back(CellData(index, i, j, i, j));
      }
    }
  }



  
  /*
  notice: 以致命栅格(254)为中心向，膨胀半径为半径外扩散，代价值不断减少

  通过不断增加距离来处理栅格; 新的栅格被加到了 corresponding distance bin
  , 因此，它们可以超越先前插入的但距离更远的cell

  ① 当原cost为NO_INFORMATION：
    inflate_unknown_为true：当新cost>0，设为新cost；
    nflate_unknown_为false：新cost≥INSCRIBED_INFLATED_OBSTACLE，设为新cost
    
    区别就在于，如果inflate unknown为true，则当膨胀到主地图上的未知区域，只要有cost，就覆盖它；
    当inflate unknown关闭，当膨胀到主地图上的未知区域时，只有新cost是障碍物，才覆盖它，否则维持未知。
    后者膨胀的范围更窄！
  ② 否则，取二者中大的cost
  */
  std::map<double, std::vector<CellData> >::iterator bin;

  // 二重循环，外层遍历“键”：即距障碍物中心点的距离；内层遍历“值”：即cell
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
  {
    // second是“键”“值”对中的“值”，即遍历次数为celldata的个数
    // 内部循环记录当前cell的坐标和离它最近的cell的坐标
    for (int i = 0; i < bin->second.size(); ++i)
    {
      // process all cells at distance dist_bin.first
      // 在该“double键”(距离)下记录迭代到的celldata
      const CellData& cell = bin->second[i];
      // 一维坐标
      unsigned int index = cell.index_;

      // 如果访问过了就忽略
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;
      // 栅格cell坐标
      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      // 离cell最近的障碍物的坐标
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

      // 计算栅格m到致命栅格s的距离，并根据cached_costs_返回该栅格的cost
      unsigned char cost = costLookup(mx, my, sx, sy);
      // 通过旧master获取当前坐标对应的代价值
      unsigned char old_cost = master_array[index];
      // notice: 通过比较，更新主地图cost，cost > 0 : cost >= 254
      // 如果当前区域是未知区域并且膨胀未知区域
      if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)) )
        master_array[index] = cost;
      else// 如果当前区域不是未知区域，则取新值和旧值中较大的
        master_array[index] = std::max(old_cost, cost);

      // notice: 优先队列
      // 接下来enqueue函数会将其四周的cell按照距离远近加入inflation_cells_对应的键下
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy);
    }
  }

  inflation_cells_.clear();
}

/**
 * @brief  给定栅格的一维坐标, 把它放到一个待处理的障碍膨胀列表中
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx 栅格的x坐标 (can be computed from the index, but saves time to store it)
 * @param  my 栅格的y坐标 (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at起点
 * @param  src_y The y index of the obstacle point inflation started at起点
 */

// 接下来enqueue函数会将其四周的cell按照距离远近加入inflation_cells_对应的键下
inline void InflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index])
  {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    // 在我们的参考矩阵cached_distances_上，找到当前cell与最近的障碍物cell的距离，
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_)
      return;

    // push the cell data onto the inflation list and mark
    // 当距离在阈值内，将当前cell加入到inflation_cells_相应键（表示距离）下
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
  }
}



/*
这里称cached_distances_和cached_costs_为参考矩阵，因为它们是后续膨胀计算的参照物。

第二个if结构是用来初始化这两个参考矩阵的，只在第一次进入时执行，它们的大小都是
(cell_inflation_radius_ + 2)x(cell_inflation_radius_ + 2)
*/

void InflationLayer::computeCaches()
{
  // 由于是基于膨胀参数来设置参考矩阵的，所以当cell_inflation_radius_==0，直接返回。
  if (cell_inflation_radius_ == 0)
    return;

  // 基于膨胀半径，计算cached_distances_和cached_costs_，两者为下面膨胀计算的参照物
  if (cell_inflation_radius_ != cached_cell_inflation_radius_)
  {
    deleteKernels();

    // cached_distances_和cached_costs_的行数都是cell_inflation_radius_+2
    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];
    // 遍历膨胀区域(二维矩阵存放的是膨胀区域内的栅格到达(0,0)点的距离)
    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      // 每个元素的值为它到(0,0)点的距离
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];

      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        // 设置cached_distances_矩阵的元素值为每个栅格到障碍物中心(0,0)点的距离
        cached_distances_[i][j] = hypot(i, j);

      }
    }
    
    // 设置cached_cell_inflation_radius_，这个if内程序不会再次进入
    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
  {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
    {
      // 二维矩阵存放的是膨胀半径内的栅格到达(0,0)点的距离，计算这些距离对应的栅格的代价值
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
    }
  }
}

void InflationLayer::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  if (cached_costs_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_costs_[i])
        delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
    cached_costs_ = NULL;
  }
}

void InflationLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
  {
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }
}

}  // namespace costmap_2d
