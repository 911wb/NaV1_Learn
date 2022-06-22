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
#include <global_planner/grid_path.h>
#include <algorithm>
#include <stdio.h>
namespace global_planner {

// 回溯获取路径
bool GridPath::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) 
{
    std::pair<float, float> current;
    // 把终点作为当前点
    current.first = end_x;
    current.second = end_y;

    int start_index = getIndex(start_x, start_y);

    // step1 将终点的(x,y)作为当前点加入path
    path.push_back(current);
    int c = 0;
    int ns = xs_ * ys_;

    // step2 不断循环，直到找到起点
    while (getIndex(current.first, current.second) != start_index) 
    {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        // notice: 广度优先搜索邻居节点时只寻找周围四个节点，回溯时却寻找当前节点的八个邻居节点，来找其父节点
        // 回溯时肯定是找八个邻居节点的，如果找四个邻居节点，规划出的路线可能会很直，并且会绕远
        // step3.搜索当前点周围8个临近点，选取这8个临近点中potential的值最小的点(G值最小的为父节点)
        for (int xd = -1; xd <= 1; xd++) 
        {
            for (int yd = -1; yd <= 1; yd++) 
            {
                // 如果是中心点(x,y)，则跳过
                if (xd == 0 && yd == 0)
                    continue;
                // (x-1, y-1)、(x-1,y)、(x-1,y+1)..
                // 一开始current为终点坐标，获取其邻居节点
                int x = current.first + xd, y = current.second + yd;
                // 获取当前邻居节点的一维坐标
                int index = getIndex(x, y);
                // 如果当前节点的G值小于min_val，则更新min_val
                if (potential[index] < min_val)
                {
                    // 更新
                    min_val = potential[index];
                    // 父节点坐标的G值最小
                    min_x = x;
                    min_y = y;
                }
            }
        }

        if (min_x == 0 && min_y == 0)
            return false;
        
        // step4 保存当前父节点坐标，并添加到path
        current.first = min_x;
        current.second = min_y;

        // 添加当前节点的父节点到路径中
        path.push_back(current);
        
        if(c++>ns*4)
        {
            return false;
        }

    }
    return true;
}

} //end namespace global_planner

