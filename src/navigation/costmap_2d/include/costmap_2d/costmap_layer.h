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
#ifndef COSTMAP_2D_COSTMAP_LAYER_H_
#define COSTMAP_2D_COSTMAP_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

namespace costmap_2d
{
/*
这个类继承自Layer类和Costmap2D类，它是地图插件（静态层和障碍层）的基类。它的类方法主要
用于处理bound和用几种不同的策略合并子地图和主地图。
*/
// 派生类CostmapLayer继承了两个基类：Class Layer、Class Costmap2D
class CostmapLayer : public Layer, public Costmap2D
{
public:
  CostmapLayer() : has_extra_bounds_(false),
    extra_min_x_(1e6), extra_max_x_(-1e6),
    extra_min_y_(1e6), extra_max_y_(-1e6) {}

  bool isDiscretized()
  {
    return true;
  }
  // 函数用主地图的尺寸来设置该层地图的尺寸；
  virtual void matchSize();
  virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert_area=false);

  /**
   * If an external source changes values in the costmap,
   * it should call this method with the area that it changed
   * to ensure that the costmap includes this region as well.
   * @param mx0 Minimum x value of the bounding box
   * @param my0 Minimum y value of the bounding box
   * @param mx1 Maximum x value of the bounding box
   * @param my1 Maximum y value of the bounding box
   */
  // 函数将传入的bound与数据成员的值比较，如果传入的bound范围更大，则更新数据成员的值；
  void addExtraBounds(double mx0, double my0, double mx1, double my1);

protected:
  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * TrueOverwrite means every value from this layer
   * is written into the master grid.
   */
  // 函数用当前子地图数据（包括未知cell）覆盖主地图对应区域
  void updateWithTrueOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Overwrite means every valid value from this layer
   * is written into the master grid (does not copy NO_INFORMATION)
   */
  // 函数用当前子地图数据（不包括未知cell）覆盖主地图对应区域
  void updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Sets the new value to the maximum of the master_grid's value
   * and this layer's value. If the master value is NO_INFORMATION,
   * it is overwritten. If the layer's value is NO_INFORMATION,
   * the master value does not change.
   */
  // 函数用当前子地图数据（不包括未知cell）更新主地图对应区域，若对应cell的cost值比主地图大或主地图该cell为未知时，用子地图数据覆盖，否则保留主地图原数据
  void updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /*
   * Updates the master_grid within the specified
   * bounding box using this layer's values.
   *
   * Sets the new value to the sum of the master grid's value
   * and this layer's value. If the master value is NO_INFORMATION,
   * it is overwritten with the layer's value. If the layer's value
   * is NO_INFORMATION, then the master value does not change.
   *
   * If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE,
   * the master value is set to (INSCRIBED_INFLATED_OBSTACLE - 1).
   */
  // 函数用当前子地图数据（不包括未知cell）更新主地图对应区域，若主地图该cell为未知，
  // 用子地图数据覆盖；否则，在主地图原数据基础上+子地图数据（将进行限制避免cost值溢出）
  void updateWithAddition(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  /**
   * Updates the bounding box specified in the parameters to include
   * the location (x,y)
   *
   * @param x x-coordinate to include
   * @param y y-coordinate to include
   * @param min_x bounding box
   * @param min_y bounding box
   * @param max_x bounding box
   * @param max_y bounding box
   */
  // 函数传入一个bound和一个坐标，若坐标不在bound范围内，它扩张bound，使其包含坐标；
  void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

  /*
   * Updates the bounding box specified in the parameters
   * to include the bounding box from the addExtraBounds
   * call. If addExtraBounds was not called, the method will do nothing.
   *
   * Should be called at the beginning of the updateBounds method
   *
   * @param min_x bounding box (input and output)
   * @param min_y bounding box (input and output)
   * @param max_x bounding box (input and output)
   * @param max_y bounding box (input and output)
   */
  // 函数在调用addExtraBounds函数后使用，它将传入的bound与更新后的数据成
  // 员比较，将更大的范围通过传入的指针填充，并恢复数据成员初始值，认为将add的bound使用过
  void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);
  bool has_extra_bounds_;

private:
  // 在构造函数中被初始化为1e6和-1e6；
  double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_LAYER_H_
