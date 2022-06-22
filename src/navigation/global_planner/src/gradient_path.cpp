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
#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

namespace global_planner {

GradientPath::GradientPath(PotentialCalculator* p_calc) :
        Traceback(p_calc), pathStep_(0.5) 
{
    // 梯度矩阵初始化
    gradx_ = grady_ = NULL;
}

GradientPath::~GradientPath() {

    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
}

void GradientPath::setSize(int xs, int ys) {
    Traceback::setSize(xs, ys);
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}
/*
双线性差值进行梯度计算
从终点开始逆向搜索，为了减少栅格对于路径精度的影响，
这里采用了利用双线性差值的的方法来计算梯度从而达到搜索对应路径节点而不是栅格的目的。
具体步骤为：
    1.将终点设置为当前计算节点 node_cur，并更新world坐标与取整后坐标的dx,dy差距
    2.将终点设置为当前计算节点 node_cur，并更新world坐标与取整后坐标的dx,dy差距
    3.计算node_cur的梯度，node_cur右侧相邻节点梯度，node_cur上侧相邻节点梯度，
        node_cur右上相邻节点梯度
    4.利用双线性差值原理计算实际计算节点world坐标对应的梯度值
    5.计算沿梯度搜索对应的实际坐标并更新node_cur以及dx,dy值
    6.重复第二步
该函数负责在potarr数组的基础上选取一些cell点来生成最终的全局规划路径，
从起点开始沿着最优行走代价值梯度下降的方向寻找到目标点的最优轨迹。
*/
// 回溯：获取梯度路径
bool GradientPath::getPath(float* potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float> >& path) 
{
    std::pair<float, float> current;
    // 终点的一维索引
    int stc = getIndex(goal_x, goal_y);

    // step xx 获得x,y 在目标点的差值和单元格总数
    // 设置偏移量 ==  终点小数部分
    float dx = goal_x - (int)goal_x;
    float dy = goal_y - (int)goal_y;
    int ns = xs_ * ys_;
    // memset是以字节为单位，初始化内存块
    memset(gradx_, 0, ns * sizeof(float));
    memset(grady_, 0, ns * sizeof(float));

    int c = 0;
    // 不断循环
    while (c++ < ns*4)
    {
        /*
        notice: 在循环中dx和dy不断被更新，回溯得到的父节点坐标：nx,ny，不断变小，直到接近起点
        y向索引 = 终点坐标/x方向栅格个数 + dy
        从终点开始沿着梯度下降的方向：终点的dx、dy最大，起点的dx、dy为接近0
        */
        // 二维坐标转换为一维坐标，一开始为终点坐标
        double nx = stc % xs_ + dx, ny = stc / xs_ + dy;

        // step2 如果当前坐标与起点坐标相差小于预设阈值，则认为搜索成功
        if (fabs(nx - start_x) < .5 && fabs(ny - start_y) < .5) 
        {
            current.first = start_x;
            current.second = start_y;
            // 将起点存入路径点中
            path.push_back(current);
            return true;
        }

        // step xx 检查是否有越界
        if (stc < xs_ || stc > xs_ * ys_ - xs_) 
        {
            printf("[PathCalc] Out of bounds\n");
            return false;
        }

        // 保存当前路径点
        current.first = nx;
        current.second = ny;

        //ROS_INFO("%d %d | %f %f ", stc%xs_, stc/xs_, dx, dy);
        // 添加至路径点数组
        path.push_back(current);

        //震荡检测，某一步和上上一步的位置是否一样
        bool oscillation_detected = false;
        int npath = path.size();
        if (npath > 2 && path[npath - 1].first == path[npath - 3].first
                && path[npath - 1].second == path[npath - 3].second) 
        {
            ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
            oscillation_detected = true;
        }

        // 当前点下方的点的索引
        int stcnx = stc + xs_;
        // 当前点上方的点的索引
        int stcpx = stc - xs_;

        // 检查当前到达节点的周边的8个节点是否有障碍物代价值，
        // 如果有的话，则直接将stc指向这8个节点中potential值最低的节点
        if (potential[stc] >= POT_HIGH || potential[stc + 1] >= POT_HIGH || potential[stc - 1] >= POT_HIGH
                || potential[stcnx] >= POT_HIGH || potential[stcnx + 1] >= POT_HIGH || potential[stcnx - 1] >= POT_HIGH
                || potential[stcpx] >= POT_HIGH || potential[stcpx + 1] >= POT_HIGH || potential[stcpx - 1] >= POT_HIGH
                || oscillation_detected)
        {
            ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int) path.size());
            //  检查8个邻点，找到最小potential的那个点
            int minc = stc;
            // 终点的G值是最大的
            int minp = potential[stc];
            int st = stcpx - 1;
            // 从左上角邻点开始
            if (potential[st] < minp) 
            {
                minp = potential[st];
                minc = st;
            }
            st++;
            // 上方邻点
            if (potential[st] < minp) 
            {
                minp = potential[st];
                minc = st;
            }
            st++;
            // 右上方邻点
            if (potential[st] < minp) 
            {
                minp = potential[st];
                minc = st;
            }
            st = stc - 1;
            // 左邻点
            if (potential[st] < minp) 
            {
                minp = potential[st];
                minc = st;
            }
            st = stc + 1;
            // 右邻点
            if (potential[st] < minp) 
            {
                minp = potential[st];
                minc = st;
            }
            st = stcnx - 1;
            // 左下方邻点
            if (potential[st] < minp) 
            {
                minp = potential[st];
                minc = st;
            }
            st++;
            // 下方邻点
            if (potential[st] < minp) 
            {
                minp = potential[st];
                minc = st;
            }
            st++;
            // 右下方邻点
            if (potential[st] < minp) 
            {
                minp = potential[st];
                minc = st;
            }
            // 更新最小G值对应的栅格坐标
            stc = minc;
            // 梯度置0
            dx = 0;
            dy = 0;

            //ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
            //    potential[stc], path[npath-1].first, path[npath-1].second);
            // 如果回溯找到的最小G值对应的栅格的G值为无穷大，说明周围都堵死了
            if (potential[stc] >= POT_HIGH) 
            {
                ROS_DEBUG("[PathCalc] No path found, high potential");
                //savemap("navfn_highpot");
                return 0;
            }
        }
        else // step xx 如果周围的点没有无穷大的G值(周围八个邻点没有障碍物)
            // 说明有好的梯度，则直接计算梯度，并沿着梯度方向查找下一个节点了
        {
            // get grad at four positions near cell
            // 计算梯度，把计算出的梯度存入梯度矩阵gradx、grady
            gradCell(potential, stc);//该点梯度
            gradCell(potential, stc + 1);//该点右侧点梯度
            gradCell(potential, stcnx);//该点下方点梯度
            gradCell(potential, stcnx + 1);//该点右下方点梯度

            // step4 利用双线性差值原理计算实际计算节点world坐标对应的梯度值
            float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
            float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

            // show gradients
            ROS_DEBUG(
                    "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", gradx_[stc], grady_[stc], gradx_[stc+1], grady_[stc+1], gradx_[stcnx], grady_[stcnx], gradx_[stcnx+1], grady_[stcnx+1], x, y);

            // 检查是否有梯度为零的，有则返回
            if (x == 0.0 && y == 0.0)
            {
                ROS_DEBUG("[PathCalc] Zero gradient");
                return 0;
            }
            // 向正确的方向(梯度的方向下降)移动，pathStep_是步长
            float ss = pathStep_ / hypot(x, y);
            /*
            增量就是梯度，梯度就是增量，可能方向不同，梯度下降就是使得增量不断减下接近0
            若接近于于0，增量便更新缓慢，看似不更新
            更新梯度：Δxk+1 = Δxk + Δx*λ， λ是步长
            使用梯度去不断更新父节点，沿着梯度的下降方向不断的找父节点，就能找到起点坐标
            
            */
            dx += x * ss;
            dy += y * ss;

            // 检查是否有溢出
            if (dx > 1.0) 
            {
                stc++;
                dx -= 1.0;
            }
            if (dx < -1.0) 
            {
                stc--;
                dx += 1.0;
            }
            if (dy > 1.0) 
            {
                stc += xs_;
                dy -= 1.0;
            }
            if (dy < -1.0) 
            {
                stc -= xs_;
                dy += 1.0;
            }

        }

        //printf("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //         potential[stc], dx, dy, path[npath-1].first, path[npath-1].second);
    }

    return false;
}

/*
 int
 NavFn::calcPath(int n, int *st)
 {
 // set up start position at cell
 // st is always upper left corner for 4-point bilinear interpolation
 if (st == NULL) st = start;
 int stc = st[1]*nx + st[0];

 // go for <n> cycles at most
 for (int i=0; i<n; i++)
 {



 }

 //  return npath;            // out of cycles, return failure
 ROS_DEBUG("[PathCalc] No path found, path too long");
 //savemap("navfn_pathlong");
 return 0;            // out of cycles, return failure
 }
 */

//
// gradient calculations
//
// calculate gradient at a cell
// positive value are to the right and down

//  该函数对当前单元格进行梯度计算，正值表示向右和向下
float GradientPath::gradCell(float* potential, int n) {
    if (gradx_[n] + grady_[n] > 0.0)    // check this cell
        return 1.0;
     // 如果越界的话
    if (n < xs_ || n > xs_ * ys_ - xs_)    // would be out of bounds
        return 0.0;
    float cv = potential[n];
    float dx = 0.0;
    float dy = 0.0;

    //  如果有障碍物
    if (cv >= POT_HIGH) {
        if (potential[n - 1] < POT_HIGH)
            dx = -lethal_cost_;
        else if (potential[n + 1] < POT_HIGH)
            dx = lethal_cost_;

        if (potential[n - xs_] < POT_HIGH)
            dy = -lethal_cost_;
        else if (potential[n + xs_] < POT_HIGH)
            dy = lethal_cost_;
    }

    else // 如果没有障碍物
    {
        // dx calc, average to sides
        if (potential[n - 1] < POT_HIGH)
            dx += potential[n - 1] - cv;
        if (potential[n + 1] < POT_HIGH)
            dx += cv - potential[n + 1];

        // dy calc, average to sides
        if (potential[n - xs_] < POT_HIGH)
            dy += potential[n - xs_] - cv;
        if (potential[n + xs_] < POT_HIGH)
            dy += cv - potential[n + xs_];
    }

    // 归一化
    float norm = hypot(dx, dy);
    if (norm > 0) 
    {
        norm = 1.0 / norm;
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;
    }
    return norm;
}

} //end namespace global_planner

