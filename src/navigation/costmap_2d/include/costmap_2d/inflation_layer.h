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
#ifndef COSTMAP_2D_INFLATION_LAYER_H_
#define COSTMAP_2D_INFLATION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

namespace costmap_2d
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */

class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  // 一维坐标
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};
/*
InflationLayer没有自身的栅格地图要维护，直接在主地图上进行操作，它根据膨胀参数设置用来膨胀
的“参考矩阵”，并在主地图上从障碍物出发，不断传播更新，完成对整个地图障碍的膨胀，等效于完
成一个由“将机器人视为一个点”到“考虑机器人本身的足迹”的转变过程，防止因为忽视了足迹而碰上障碍

*/
// 派生类InflationLayer继承了Class Layer
class InflationLayer : public Layer
{
public:
  InflationLayer();

  virtual ~InflationLayer()
  {
    deleteKernels();
    if (dsrv_)
        delete dsrv_;
    if (seen_)
        delete[] seen_;
  }

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();

  virtual void reset() { onInitialize(); }

  /** @brief  Given a distance, compute a cost.
   * @param  distance The distance from an obstacle in cells
   * @return A cost value for the distance */

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
    距离 == 膨胀半径内的栅格到达障碍物中心(0,0)点的距离
    1.若距离为0，即(0,0)点，则设置代价地图的代价值为254，若此时障碍物与机器人中心重叠，必然发生碰撞；
    2.若距离 ≤ 机器人内切圆半径，则设置代价地图的代价值为253，若此时障碍物处于机器人的内切圆内，必然发生碰撞；
    3.若机器人足迹内切圆半径 < 距离 ≤ cell_inflation_radius_，则以距离远近为比例（指数型）设置代价值：[128,252] = 252*α
      此时障碍物处于其机器人的「外切圆内，内切圆外」，处于碰撞临界，不一定发生碰撞；
*/

  // 只计算膨胀区域的cost
  virtual inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0)
      cost = LETHAL_OBSTACLE;// 254
    else if (distance * resolution_ <= inscribed_radius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;// 253
    else
    {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;
      // α = e^(-w*(dist - r))，r代表内切圆半径
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      // cost = (253 - 1) * α
      cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  /**
   * @brief Change the values of the inflation radius parameters
   * @param inflation_radius The new inflation radius
   * @param cost_scaling_factor The new weight
   */
  void setInflationParameters(double inflation_radius, double cost_scaling_factor);

protected:
  virtual void onFootprintChanged();
  boost::recursive_mutex* inflation_access_;

  double resolution_;
  double inflation_radius_;
  double inscribed_radius_;
  double weight_;
  bool inflate_unknown_;

private:
  /**
   * @brief  Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }

  /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[dx][dy];
  }

  void computeCaches();
  void deleteKernels();
  void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);

  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);

  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  // <栅格到(0,0)的距离，对应的栅格坐标>
  std::map<double, std::vector<CellData> > inflation_cells_;

  bool* seen_;
  int seen_size_;

  unsigned char** cached_costs_;
  double** cached_distances_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig> *dsrv_;
  void reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level);

  bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_INFLATION_LAYER_H_
