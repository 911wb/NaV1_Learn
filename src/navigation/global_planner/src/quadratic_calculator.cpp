/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <global_planner/quadratic_calculator.h>

namespace global_planner 
{

// notice: 二次逼近的方法计算 G： 优化后的potential扩散会由原来的矩形扩散优化为圆形扩散。
// 参数：势场矩阵、0+邻居节点到其父节点的代价、当前节点的邻居节点的一维坐标、 当前父节点的势场值G
float QuadraticCalculator::calculatePotential(float* potential, unsigned char cost, int n, float prev_potential) 
{
    // get neighbors
    // 把当前节点的邻居节点看做是当前中心点
    float u, d, l, r;
    // 先获取当前cell四周邻点的potarr值，并取最小的值存入ta。
    l = potential[n - 1];// G左
    r = potential[n + 1];// G右
    u = potential[n - nx_];// G上
    d = potential[n + nx_];// G下
    // ROS_INFO("[Update] c: %f  l: %f  r: %f  u: %f  d: %f\n", potential[n], l, r, u, d);
    //  ROS_INFO("[Update] cost: %d\n", costs[n]);

    // 找到最小的potential，以及它相邻点的最小potential
    // 从四个栅格中的两个最低相邻点进行平面更新计算
    float ta, tc;
    if (l < r)
        tc = l;
    else
        tc = r;
    if (u < d)
        ta = u;
    else
        ta = d;

    /*
    在计算当前点Potential值时，有两种情况，即需要对“左右邻点最小pot值与上下邻点最小pot值之差的
    绝对值”和“当前cell的costarr值”比较，有pot = ta+hf和另一个更复杂的公式，这两个公式的功能相同
    但效果有区别，区别也是前面提到过的“圆形传播”和“菱形传播”，后者能够产生效果更好的菱形传播。
    只有当前cell的Potential计算值<原本的Potential值，才更新，这意味着从目标点开始，
    它的Potential值被初始化为0，不会被更新，接下来传播到它的四个邻点，才会开始更新他们的Potential值。
    */

    // 描述可通行性的因子 0+邻居节点到其父节点的代价
    float hf = cost; 

    // tc和tc之间的代价之差
    float dc = tc - ta;        // relative cost between ta,tc
    // 设置ta为最小的
    if (dc < 0)         // tc is lowest
    {
        dc = -dc;
        ta = tc;
    }

    // ta 和 tc 的代价值差如果太大, 只用ta更新
    /*
    为便于理解，这里分析第一个公式，当前点Potential值=四周最小的Potential值+当前点cost值。
    这说明，从目标点（Potential=0）向外传播时，它四周的可行cell的Potential值会变成0+cost，
    可以假设他们的cost都是50，那么它们的Potential值都被更新为50（因为初始值无限大，故会被迭代）
    第二轮次的传播中，假设邻点的邻点cost也为50，那么它们的Potential值将被更新为100(离起点越来越离)。
    这种传播过程中cost的累加造成Potential值的上升能够反映离目标点的远近。

    并且，cost大小与cell离障碍物的远近对应，更大的cost对应更大的Potential，并且障碍物点不更新
    Potential，使得其值停留在无限大，故Potential值的大小也能反映点与障碍物的接近程度。
    */

    if (dc >= hf)        
        // 四个节点最小的Potential值 + 当前点邻居节点的cost值。
        // cost(hf)的累加造成Potential值的上升能够反映离目标点越近，距离起点越远
        return ta + hf;
    else // 使用双线性插值来更新G 以目标点为圆心向外圆形传播
    {
        // 使用二次近似
        float d = dc / hf;
        float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
        // potential = ta + hf*v
        return ta + hf * v;
    }
}
}

