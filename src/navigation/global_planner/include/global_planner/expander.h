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
#ifndef _EXPANDER_H
#define _EXPANDER_H
#include <global_planner/potential_calculator.h>
#include <global_planner/planner_core.h>

namespace global_planner {
//扩展器 虚基类
class Expander 
{
    public:
        // 构造函数初始化
        Expander(PotentialCalculator* p_calc, int nx, int ny) :
                unknown_(true), lethal_cost_(253), neutral_cost_(50), factor_(3.0), p_calc_(p_calc) 
        {
            setSize(nx, ny);
        }
        virtual ~Expander() {}
        // 纯虚函数 最主要的就是启发函数的接口，这个在地杰斯特拉算法和astar算法等具体算法中实现
        virtual bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) = 0;

        /**
         * @brief 设置或者重置地图的大小  , x,y
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        virtual void setSize(int nx, int ny) 
        {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        } 
        // 设置致命代价大小
        void setLethalCost(unsigned char lethal_cost) 
        {
            lethal_cost_ = lethal_cost;
        }
        // 设置中立代价大小
        void setNeutralCost(unsigned char neutral_cost) 
        {
            neutral_cost_ = neutral_cost;
        }
        // 设置因子大小
        void setFactor(float factor) 
        {
            factor_ = factor;
        }
        // 设置未知
        void setHasUnknown(bool unknown) 
        {
            unknown_ = unknown;
        }

        // 清空端点
        void clearEndpoint(unsigned char* costs, float* potential, int gx, int gy, int s)
        {
            // 返回一维数组下标
            int startCell = toIndex(gx, gy);
            // i代表x,j代表y;
            for(int i=-s;i<=s;i++)
            {
                for(int j=-s;j<=s;j++)
                {
                    // n是该次循环地图数组的下标
                    int n = startCell+i+nx_*j;
                    // POT_HIGH 是 未分配的栅格值
                    if(potential[n]<POT_HIGH)
                        continue;
                    float c = costs[n]+neutral_cost_;
                    float pot = p_calc_->calculatePotential(potential, c, n);
                    potential[n] = pot;
                }
            }
        }

    protected:
        //返回一维数组下标
        inline int toIndex(int x, int y) 
        {
            return x + nx_ * y;
        }

        int nx_, ny_, ns_; /**< size of grid, in pixels 地图的长宽和面积*/
        bool unknown_;
        unsigned char lethal_cost_, neutral_cost_;//致命 中立代价
        int cells_visited_;//已经遍历过的栅格
        float factor_;//因子,因素
        PotentialCalculator* p_calc_;// 可行扩展点矩阵计算器

};

} //end namespace global_planner
#endif
