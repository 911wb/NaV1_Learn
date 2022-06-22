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
#include<global_planner/dijkstra.h>
#include <algorithm>
namespace global_planner {

DijkstraExpansion::DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny) :
        Expander(p_calc, nx, ny), pending_(NULL), precise_(false) 
{
    // priority buffers
    // 初始化优先级队列 buf 长度为PRIORITYBUFSIZE = 10000;
    buffer1_ = new int[PRIORITYBUFSIZE];
    buffer2_ = new int[PRIORITYBUFSIZE];
    buffer3_ = new int[PRIORITYBUFSIZE];

    priorityIncrement_ = 2 * neutral_cost_;
}

DijkstraExpansion::~DijkstraExpansion() {
  delete[] buffer1_;
  delete[] buffer2_;
  delete[] buffer3_;
  if (pending_)
      delete[] pending_;
}

// 设置或初始化地图的长度
void DijkstraExpansion::setSize(int xs, int ys) 
{
    Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[ns_];
    // 初始化未决全部设置为0
    memset(pending_, 0, ns_ * sizeof(bool));
}

//
// main propagation function
// Dijkstra method, breadth-first notice: BFS
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)

// DFS：迪杰斯特拉主要的启发函数：BFS
bool DijkstraExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                           int cycles, float* potential) 
{
    // 设置已经遍历过的栅格为0
    cells_visited_ = 0;
    // 终止扩展代价的临界值，刚开始为致命代价
    threshold_ = lethal_cost_;
    // 将buffer1的地址传递给当前缓冲区
    currentBuffer_ = buffer1_;
    // 当前缓冲区的长度设置为0
    currentEnd_ = 0;
    // 把第二个缓冲区给下一个缓冲区
    nextBuffer_ = buffer2_;
    nextEnd_ = 0;
    overBuffer_ = buffer3_;
    overEnd_ = 0;
    // 初始化未决全部设置为0
    memset(pending_, 0, ns_ * sizeof(bool));
    // 初始化 potential里面的值全部为无穷大 POT_HIGH = 10e10 10的10次方
    std::fill(potential, potential + ns_, POT_HIGH);

    // 返回起点点的一维数组下标
    int k = toIndex(start_x, start_y);
    // 使用true新的方式 还是 false:老版navfn启发方式
    if(precise_)
    {
        double dx = start_x - (int)start_x, dy = start_y - (int)start_y;

        // 向下取整 保留小数点后两位 ??
        dx = floorf(dx * 100 + 0.5) / 100;
        dy = floorf(dy * 100 + 0.5) / 100;
        
        // 设置起始点以及其周围三个点的G值
        // 起始点的G值 =  中立代价 *2 * dx * dy  不太明白为啥乘dxdy
        potential[k] = neutral_cost_ * 2 * dx * dy;
        potential[k+1] = neutral_cost_ * 2 * (1-dx)*dy;
        potential[k+nx_] = neutral_cost_*2*dx*(1-dy);
        potential[k+nx_+1] = neutral_cost_*2*(1-dx)*(1-dy);

        // step1 将起始点四周的八个点(一维坐标)加入优先级current队列
        push_cur(k+2);
        push_cur(k-1);
        push_cur(k+nx_-1);
        push_cur(k+nx_+2);

        push_cur(k-nx_);
        push_cur(k-nx_+1);
        push_cur(k+nx_*2);
        push_cur(k+nx_*2+1);
    }
    else
    {
        // 当前点的代价设为0
        potential[k] = 0;
        // 把上下左右四个节点全部压入current队列
        push_cur(k+1);
        push_cur(k-1);
        push_cur(k-nx_);
        push_cur(k+nx_);
    }

    int nwv = 0;             // priority block的最大数量
    int nc = 0;             // priority blocks中的cell数
    int cycle = 0;         // 当前迭代次数

    // 记录起始位置的索引：就是目标点
    int startCell = toIndex(end_x, end_y);

    // 循环迭代更新potarr，判断条件：如果当前正在传播和下一步传播的集都为空，
    // 那么说明已经无法继续传播，可能有无法越过的障碍或其他情况，退出。
    for (; cycle < cycles; cycle++)
    {
        // step2 然后进入主循环,主循环先判断当前优先级队列是否为空，如果为空则代表没办法扩展了直接退出
        if (currentEnd_ == 0 && nextEnd_ == 0) 
            return false;

        // stats
        nc += currentEnd_;

        // step3记录一下最大优先级队列大小，最大优先级块大小 不能小于 当前队列大小
        if (currentEnd_ > nwv)
            nwv = currentEnd_;

        // reset pending_ flags on current priority buffer
        // 在当前优先级缓冲区上重置未决标志
        int *pb = currentBuffer_;
        // 重新获取当前队列的长度i
        int i = currentEnd_;

        // step4 将当前优先级队列的pending全部设为false
        while (i-- > 0)
            pending_[*(pb++)] = false;

        // 处理当前优先级缓冲区，重新获取当前队列的指针
        pb = currentBuffer_;
        // 重新获取当前队列的长度i
        i = currentEnd_;

        /*
        step5 对当前优先队列中每个坐标(栅格)调用updatecell函数，来更新G值数组
        并将其四周符合特定条件的点放入nextP或overP，用于下一步的传播。
        nextP和overP都来自从目标点开始传播的四周的cell，区别在于它们的“父cell”的
        pot值是否达到阈值curT，没达到则放入nextP，达到则放入overP。
        设置一个阈值来区分nextP和overP的传播先后顺序，结果是以目标点为圆心向外圆形传播，
        而不设阈值区分，则是以目标点为中心向外呈菱形传播，显然前者更合理。
        */
        // 遍历当前队列，直到当前队列不为空，一开始当前队列存放的是起点的八个邻居节点
        while (i-- > 0)
            // updateCell用于更新单个栅格的Potential值
            // 参数：全局代价地图、G值数组、优先队列中存放的当前节点周围节点的一维坐标
            updateCell(costs, potential, *pb++);

        /*
        step6 调用完成之后，交换当前队列与下一队列
        调用完成后，将nextP数组中的cell传递给curP，继续上述传播，
        若nextP没有cell可以用来传播，则引入overP中的cell。
        currentBuffer_ <=> nextBuffer_  swap buffers
        */
        currentEnd_ = nextEnd_;
        nextEnd_ = 0;
        // 保存当前队列的指针
        pb = currentBuffer_;
        // 当前队列的指针指向下一个队列        
        currentBuffer_ = nextBuffer_;
        nextBuffer_ = pb;

        // step7 查看当前优先级队列是否为空，空的话代表没有什么可以扩展的了
        // 若nextP没有cell可以用来传播，则引入overP中的cell，继续下一次传播
        if (currentEnd_ == 0)
        {
            // step8 如果为空则调大临界点的阈值（最开始是致命代价），将关闭列表与当前列表互换
            // 递增优先级阈值
            threshold_ += priorityIncrement_;    // increment priority threshold
            // 将当前设置为溢出块
            currentEnd_ = overEnd_;    // set current to overflow block
            overEnd_ = 0;
            pb = currentBuffer_;        // swap buffers
            currentBuffer_ = overBuffer_;
            // 交换over 和当前队列
            overBuffer_ = pb;
        }

        // 在从目标点向全地图传播的过程中检查，当起点的Potential值不再是被初始化的无穷大，
        // 而是有一个实际的值时，说明到达了目标点，传播停止。
        if (potential[startCell] < POT_HIGH)
            break;

        // step9 进行下一次扩展（这里表明了无论是否大于致命代价，地杰斯特拉算法总会扩展到目标点）
    }

    ROS_INFO("CYCLES: %d/%d ", cycle, cycles);
    
    if (cycle < cycles)
        return true; // finished up here
    else
        return false;
}


//二分之根号二
#define INVSQRT2 0.707106781
// 关键函数: 根据邻居的值计算单元更新后的潜在值
// 从四个网格中的两个最低相邻点进行平面更新计算
// 内插值的二次近似
// 这里没有边界检查,函数应该很快

/*
updateCell用于更新单个栅格的Potential值，
先获取当前cell四周邻点的potarr值，并取最小的值存入ta。
*/
inline void DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n) 
{
    // 已经遍历过的栅格 自加1
    cells_visited_++;

    // 获取当前邻居栅格的代价值
    // cost大小与cell离障碍物的远近对应，更大的cost对应更大的Potential，
    // 并且障碍物点不更新Potential，使得其值停留在无限大
    float c = getCost(costs, n);
    // 如果该栅格的代价值大于等于致命代价，则跳过该栅格，因为不能传播到障碍物
    if (c >= lethal_cost_)    // don't propagate into obstacles
        return;
    /*
    简单的方法计算G ： pot  =  前后左右最小的potential + 当前的costs 
    notice: 二次逼近的方法计算 G： 优化后的potential 扩散会由原来的矩形扩散优化为圆形扩散。
    输入：G值数组、当前栅格的代价值、当前栅格的坐标，输出当前栅格的G值
    计算当前邻居节点的G值，G值(控制路径的长度，总G值越小路径越短)
    */
    float pot = p_calc_->calculatePotential(potential, c, n);

    // 因为初始值无限大，故会被迭代
    if (pot < potential[n])
    {
        // 获得上下左右的 代价值, 但是不明白为啥要乘以二分之根号二 ?? 
        float le = INVSQRT2 * (float)getCost(costs, n - 1);
        float re = INVSQRT2 * (float)getCost(costs, n + 1);
        float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
        float de = INVSQRT2 * (float)getCost(costs, n + nx_);
        // 更新当前邻居节点的G值数组
        potential[n] = pot;
        //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
        
        // notice: G值小的优先传播(放入当前优先队列)，G值大的后面传播
        // threshold刚开始为致命代价
        if (pot < threshold_)    // low-cost buffer block
        {   
            // 将当前节点周围上下左右的四个节点(栅格)，push到next的优先队列，供下一次传播迭代时使用
            if (potential[n - 1] > pot + le)
                push_next(n-1);// 将其push到next的队列
            if (potential[n + 1] > pot + re)
                push_next(n+1);
            if (potential[n - nx_] > pot + ue)
                push_next(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_next(n+nx_);
        } 
        else // overflow block：pot >= threshold_
        {
            if (potential[n - 1] > pot + le)
                push_over(n-1);// 将其push到over的队列
            if (potential[n + 1] > pot + re)
                push_over(n+1);
            if (potential[n - nx_] > pot + ue)
                push_over(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_over(n+nx_);
        }
    }
}

} //end namespace global_planner


