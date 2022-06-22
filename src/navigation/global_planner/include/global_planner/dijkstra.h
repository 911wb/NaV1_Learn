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
#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#define PRIORITYBUFSIZE 10000
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>

// inserting onto the priority blocks
// 在current buffer 添加当前点
#define push_cur(n)  { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ && currentEnd_<PRIORITYBUFSIZE){ currentBuffer_[currentEnd_++]=n; pending_[n]=true; }}
// 在next buffer 添加当前点
#define push_next(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    nextEnd_<PRIORITYBUFSIZE){    nextBuffer_[   nextEnd_++]=n; pending_[n]=true; }}
// 在end buffer添加当前点 /也就是把当前点压入到 end队列里面
#define push_over(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    overEnd_<PRIORITYBUFSIZE){    overBuffer_[   overEnd_++]=n; pending_[n]=true; }}

namespace global_planner {
class DijkstraExpansion : public Expander 
{
    public:
        DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny);
        virtual ~DijkstraExpansion();
        // 启发函数
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                float* potential);

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        // 设置地图大小
        void setSize(int nx, int ny); /**< sets or resets the size of the map */
        // 设置中性代价值
        void setNeutralCost(unsigned char neutral_cost) {
            neutral_cost_ = neutral_cost;
            priorityIncrement_ = 2 * neutral_cost_;
        }
        void setPreciseStart(bool precise)
        { 
            precise_ = precise; 
        }
    private:

        /**
         * @brief  更新索引为n的单元格
         * @param costs The costmap
         * @param potential The potential array in which we are calculating
         * @param n n为要更新的索引
         */
        void updateCell(unsigned char* costs, float* potential, int n); /** updates the cell at index n */
        // 获取某点的代价
        float getCost(unsigned char* costs, int n) 
        {
            float c = costs[n];
            // 如果该点代价 小于 致命代价-1, 或者 未知参数为1, 并且 该点cost等于255
            if (c < lethal_cost_ - 1 || (unknown_ && c==255))
            {
                // 代价 =  当前代价 * 因子 + 中立代价(单格距离代价)
                c = c * factor_ + neutral_cost_;
                // 如果代价值是255
                if (c >= lethal_cost_)
                    c = lethal_cost_ - 1;// ??
                return c;
            }
            // 如果该栅格的代价值 >= 致命代价，返回致命代价
            return lethal_cost_;
        }

        // 块优先级缓冲区
        int *buffer1_, *buffer2_, *buffer3_; /**< storage buffers for priority blocks */
        // 当前缓冲区/下一个缓冲区/停止缓冲区
        int *currentBuffer_, *nextBuffer_, *overBuffer_; /**< priority buffer block ptrs */
        // 数组的终点
        int currentEnd_, nextEnd_, overEnd_; /**< end points of arrays */
        bool *pending_; /**< pending_ cells during propagation */
        bool precise_;

        /** block priority thresholds */
        // 终止扩展代价的临界值，刚开始为致命代价，如果搜索完毕未找到目标点则增大该临界值继续扩展直到扩展到目标点
        float threshold_; /**< current threshold */
        //优先级阈值增量
        float priorityIncrement_; /**< priority threshold increment */

};
} //end namespace global_planner
#endif
