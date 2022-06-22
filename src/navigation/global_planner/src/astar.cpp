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
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) 
{
}
// 计算路径代价的函数：
//  costs为代价地图的指针，potential为G值数组，cycles为循环次数 == 2*nx*ny
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) 
{
    // queue_为启发式搜索到的向量队列：<i , cost>
    queue_.clear();
    // 起点的索引，start_index，二维坐标转一维坐标
    int start_i = toIndex(start_x, start_y);
    // step1 将起点放入队列，维护一个优先级队列，使得算法复时间杂度为o(nlogn)
    // 将起点的 F 设置为0
    queue_.push_back(Index(start_i, 0));

    // std::fill(a,b,x) 将a到b的元素都赋予x值，ns_ = nx * ny
    // step2 potential数组 G 值全设为无穷大，如果其中的G值不为无穷大，说明访问过
    std::fill(potential, potential + ns_, POT_HIGH);
    // step3 将起点的势场值 G 设为0
    potential[start_i] = 0;
    // 目标索引，二维转一维
    int goal_i = toIndex(end_x, end_y);

    int cycle = 0;

    // 进入循环，继续循环的判断条件为只要队列大小大于0 且 循环次数小于所有格子数的2倍
    while (queue_.size() > 0 && cycle < cycles)
    {

        // step4 取出优先队列头部数据，得到最小cost(F)值的节点
        Index top = queue_[0];
        
        /*
        step5 
            pop_heap(Iter,Iter,_Compare) _Compare有两种参数，一种是greater（小顶堆），一种是less（大顶堆）,先对调，再排序
            将堆顶元素（即为数组第一个位置）和数组最后一个位置对调，并且将新的最大值置于所给范围的最前面,
            然后你可以调用数组pop_back，删除这个元素
        */
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        
        // step6 从队列中删除最小代价(F值)对应的的节点(在队列尾部)，实际上这是一个数组而不是队列，所以只能从尾部删除
        queue_.pop_back();

        // 取出队列头指针指向的数据的一维坐标
        int i = top.i;

        // step7 若是目标点则终止搜索，搜索成功
        if (i == goal_i)
            return true;

        // notice: DFS:遍历父节点的四个邻居节点
        // step8 将代价最小点i周围点加入搜索队里并更新代价值, 即对上下左右四个点执行add函数
        add(costs, potential, potential[i], i + 1, end_x, end_y);// 右
        add(costs, potential, potential[i], i - 1, end_x, end_y);// 左
        add(costs, potential, potential[i], i + nx_, end_x, end_y);// 下
        add(costs, potential, potential[i], i - nx_, end_x, end_y);// 上

        cycle++;
    }

    return false;
}

// 添加点并更新代价函数
// 参数：全局代价地图、势场矩阵、当前父节点的势场值G、当前节点的邻居节点的一维坐标、终点x、终点y
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) 
{
    // 若当前节点的邻居节点 超出范围， ns_为栅格总数
    if (next_i < 0 || next_i >= ns_)
        return;
    // 若当前节点的邻居节点 已经搜索过的点
    if (potential[next_i] < POT_HIGH)
        return;
    // 若当前节点的邻居节点 为障碍物点
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;
    /*
    采用简单方法计算值为：costs[next_i] + neutral_cost_+ prev_potential  
    当前邻居节点的G值 == 当前邻居节点对应的地图代价值(一般是0) + 单格距离代价(邻居节点到其父节点的代价) + 父节点的G值 
    notice: p_calc_决定是使用二次逼近算法计算G 还是 搜索上下左右邻居节点
    计算当前邻居节点的G值，G值(控制路径的长度，总G值越小路径越短)
    */
    potential[next_i] = p_calc_->calculatePotential(potential,// 势场矩阵
        costs[next_i] + neutral_cost_, // 0+邻居节点到其父节点的代价
        next_i,// 当前节点的邻居节点的一维坐标
        prev_potential);// 当前父节点的势场值G
    
    // 计算当前节点的邻居节点x,y坐标，一维坐标转二维坐标
    int x = next_i % nx_, y = next_i / nx_;

    // notice: BFS思想： H == 当前邻居节点到终点的曼哈顿距离(深度) TODO: 可以修改
    float distance = abs(end_x - x) + abs(end_y - y);

    /*
    potential[next_i]：    
        起始点到当前点的cost即g(n)
    distance * neutral_cost_(栅格与栅格之间的代价，即比例)：
        当前点到目的点的cost即h(n)。
    f(n)=g(n)+h(n)：  
        计算完这两个cost后，加起来即为f(n)，将其存入队列中
    */

    // 将当前访问过的邻居节点存入容器中，Index(当前邻居节点的坐标, F)
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    
    // notice: 将队列中的所有节点进行排列，F值最 小 的节点排在前面 (优先队列)
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace global_planner