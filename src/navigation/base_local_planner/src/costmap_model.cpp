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
*********************************************************************/
#include <base_local_planner/line_iterator.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>

using namespace std;
using namespace costmap_2d;
/*
CostmapModel类

这个类帮助局部规划器在Costmap上进行计算，footprintCost、lineCost、pointCost三个类方法分别
能通过Costmap计算出机器人足迹范围、两个cell连线、单个cell的代价，并将值返回给局部规划器。
这里简单看一下footprintCost函数。
  若预设的足迹点数<3，考虑足迹的形状没有意义，这时只计算机器人位置点在costmap上的代价；
  若预设的足迹点数≥3，把足迹视为多边形，循环调用lineCost计算多边形各边的cell，注意首尾闭合，最后返回代价。
返回值:
  -1.0 ：覆盖至少一个障碍cell
  -2.0 ：覆盖至少一个未知cell
  -3.0 ：不在地图上
  其他正cost

为什么这里只要多边形边缘不经过障碍就认为足迹不经过障碍呢？ 因为这个函数是在轨迹点生成过程
中，对新生成的轨迹点调用的，而既然能新生成轨迹点，说明该轨迹点必不是障碍物，当多边形边缘
不经过障碍、中心也不是障碍的时候，如果机器人比较小，可以认为足迹不经过障碍。


*/
namespace base_local_planner {
  CostmapModel::CostmapModel(const Costmap2D& ma) : costmap_(ma) {}

  double CostmapModel::footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
      double inscribed_radius, double circumscribed_radius){
    // returns:
    //  -1 if footprint covers at least a lethal obstacle cell, or
    //  -2 if footprint covers at least a no-information cell, or
    //  -3 if footprint is [partially] outside of the map, or
    //  a positive value for traversable space

    //used to put things into grid coordinates
    //声明地图坐标系上的坐标
    unsigned int cell_x, cell_y;
    /****************************************考虑机器人中心******************************************/
    //得到机器人中心点的cell坐标，存放在cell_x cell_y中
    //如果得不到坐标，说明不在地图上，直接返回-3

    //get the cell coord of the center point of the robot
    if(!costmap_.worldToMap(position.x, position.y, cell_x, cell_y))
      return -3.0;

    //if number of points in the footprint is less than 3, we'll just assume a circular robot
    //如果脚印（预先输入的）点数小于三，默认机器人形状为圆形，不考虑脚印，只考虑中心
    if(footprint.size() < 3)
    {
      unsigned char cost = costmap_.getCost(cell_x, cell_y);
      //如果中心位于未知代价的cell上，返回-2
      if(cost == NO_INFORMATION)
        return -2.0;
      //如果中心位于致命障碍cell上，返回-1
      if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
        return -1.0;
      //如果机器人位置既不是未知也不是致命，返回它的代价  
      return cost;
    }
    /****************************************考虑机器人脚印******************************************/

    //now we really have to lay down the footprint in the costmap grid
    //接下来在costmap上考虑脚印
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;
    //对footprint进行光栅化算法处理得到栅格点
    //对footprint进行光栅化算法处理得到栅格点
    //对footprint进行光栅化算法处理得到栅格点

    //we need to rasterize each line in the footprint
    for(unsigned int i = 0; i < footprint.size() - 1; ++i)
    {
      //得到脚印中第一个点的坐标x0, y0
      //get the cell coord of the first point
      if(!costmap_.worldToMap(footprint[i].x, footprint[i].y, x0, y0))
        //如果无法转换，说明第一个脚印点不在地图上，直接返回-3
        return -3.0;

      //get the cell coord of the second point
      //得到脚印中下一个点的坐标x1, y1
      if(!costmap_.worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
        //如果无法转换，说明下一个脚印点不在地图上，直接返回-3
        return -3.0;
      //得到连线的代价
      line_cost = lineCost(x0, x1, y0, y1);
      //不断用最大的连线代价迭代footprint_cost
      footprint_cost = std::max(line_cost, footprint_cost);

      //if there is an obstacle that hits the line... we know that we can return false right away
      //如果某条边缘线段代价<0，直接停止生成代价，返回这个负代价
      if(line_cost < 0)
        return line_cost;
    }

    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the last point
    //再把footprint的最后一个点和第一个点连起来，形成闭合
    if(!costmap_.worldToMap(footprint.back().x, footprint.back().y, x0, y0))
      return -3.0;

    //get the cell coord of the first point
    if(!costmap_.worldToMap(footprint.front().x, footprint.front().y, x1, y1))
      return -3.0;

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    if(line_cost < 0)
      return line_cost;

    //if all line costs are legal... then we can return that the footprint is legal
    //如果所有边缘线的代价都是合法的，那么返回足迹的代价
    return footprint_cost;

  }

  //calculate the cost of a ray-traced line
  double CostmapModel::lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for( LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
    {
      point_cost = pointCost( line.getX(), line.getY() ); //Score the current point

      if(point_cost < 0)
        return point_cost;

      if(line_cost < point_cost)
        line_cost = point_cost;
    }

    return line_cost;
  }

  double CostmapModel::pointCost(int x, int y) const {
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == NO_INFORMATION)
      return -2;
    if(cost == LETHAL_OBSTACLE)
      return -1;

    return cost;
  }

};
