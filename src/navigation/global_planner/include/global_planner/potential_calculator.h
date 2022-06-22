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
#ifndef _POTENTIAL_CALCULATOR_H
#define _POTENTIAL_CALCULATOR_H

#include <algorithm>

namespace global_planner {
// 潜力计算器
class PotentialCalculator {
    public:
        PotentialCalculator(int nx, int ny) 
        {
            // 设置地图大小
            setSize(nx, ny);
        }
        virtual ~PotentialCalculator() {}
        // notice: 
        // n是当前节点的邻居节点坐标 
        virtual float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential=-1)
        {
            // A*不进来，只有Dij进来
            if(prev_potential < 0)
            {
                // ROS_INFO("============================================ %.2f", prev_potential);
                // get min of neighbors
                // 分别求出前后、左右的 G 值
                float min_h = std::min( potential[n - 1], potential[n + 1] ),
                      min_v = std::min( potential[n - nx_], potential[n + nx_]);
                // 从最小值中再取最小，也就是四个最小的，该值对应的节点为父节点
                prev_potential = std::min(min_h, min_v);

            }

            // 返回当前点四周最小的代价值及累加前面累加的代价值。
            return prev_potential + cost;
        }

        /**
         * @brief  //设置或重置地图的大小
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        virtual void setSize(int nx, int ny) 
        {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        }

    protected:
        // 求出地图坐标点 x,y 在一维数组里面的下标
        inline int toIndex(int x, int y) 
        {
            return x + nx_ * y;
        }

        int nx_, ny_, ns_; /**< size of grid, in pixels */
};

} //end namespace global_planner
#endif
