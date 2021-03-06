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
    // ???????????????????????? buf ?????????PRIORITYBUFSIZE = 10000;
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

// ?????????????????????????????????
void DijkstraExpansion::setSize(int xs, int ys) 
{
    Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[ns_];
    // ??????????????????????????????0
    memset(pending_, 0, ns_ * sizeof(bool));
}

//
// main propagation function
// Dijkstra method, breadth-first notice: BFS
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)

// DFS??????????????????????????????????????????BFS
bool DijkstraExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                           int cycles, float* potential) 
{
    // ?????????????????????????????????0
    cells_visited_ = 0;
    // ?????????????????????????????????????????????????????????
    threshold_ = lethal_cost_;
    // ???buffer1?????????????????????????????????
    currentBuffer_ = buffer1_;
    // ?????????????????????????????????0
    currentEnd_ = 0;
    // ??????????????????????????????????????????
    nextBuffer_ = buffer2_;
    nextEnd_ = 0;
    overBuffer_ = buffer3_;
    overEnd_ = 0;
    // ??????????????????????????????0
    memset(pending_, 0, ns_ * sizeof(bool));
    // ????????? potential?????????????????????????????? POT_HIGH = 10e10 10???10??????
    std::fill(potential, potential + ns_, POT_HIGH);

    // ????????????????????????????????????
    int k = toIndex(start_x, start_y);
    // ??????true???????????? ?????? false:??????navfn????????????
    if(precise_)
    {
        double dx = start_x - (int)start_x, dy = start_y - (int)start_y;

        // ???????????? ???????????????????????? ??
        dx = floorf(dx * 100 + 0.5) / 100;
        dy = floorf(dy * 100 + 0.5) / 100;
        
        // ??????????????????????????????????????????G???
        // ????????????G??? =  ???????????? *2 * dx * dy  ?????????????????????dxdy
        potential[k] = neutral_cost_ * 2 * dx * dy;
        potential[k+1] = neutral_cost_ * 2 * (1-dx)*dy;
        potential[k+nx_] = neutral_cost_*2*dx*(1-dy);
        potential[k+nx_+1] = neutral_cost_*2*(1-dx)*(1-dy);

        // step1 ??????????????????????????????(????????????)???????????????current??????
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
        // ????????????????????????0
        potential[k] = 0;
        // ???????????????????????????????????????current??????
        push_cur(k+1);
        push_cur(k-1);
        push_cur(k-nx_);
        push_cur(k+nx_);
    }

    int nwv = 0;             // priority block???????????????
    int nc = 0;             // priority blocks??????cell???
    int cycle = 0;         // ??????????????????

    // ?????????????????????????????????????????????
    int startCell = toIndex(end_x, end_y);

    // ??????????????????potarr??????????????????????????????????????????????????????????????????????????????
    // ????????????????????????????????????????????????????????????????????????????????????????????????
    for (; cycle < cycles; cycle++)
    {
        // step2 ?????????????????????,?????????????????????????????????????????????????????????????????????????????????????????????????????????
        if (currentEnd_ == 0 && nextEnd_ == 0) 
            return false;

        // stats
        nc += currentEnd_;

        // step3?????????????????????????????????????????????????????????????????? ???????????? ??????????????????
        if (currentEnd_ > nwv)
            nwv = currentEnd_;

        // reset pending_ flags on current priority buffer
        // ????????????????????????????????????????????????
        int *pb = currentBuffer_;
        // ?????????????????????????????????i
        int i = currentEnd_;

        // step4 ???????????????????????????pending????????????false
        while (i-- > 0)
            pending_[*(pb++)] = false;

        // ??????????????????????????????????????????????????????????????????
        pb = currentBuffer_;
        // ?????????????????????????????????i
        i = currentEnd_;

        /*
        step5 ????????????????????????????????????(??????)??????updatecell??????????????????G?????????
        ?????????????????????????????????????????????nextP???overP??????????????????????????????
        nextP???overP?????????????????????????????????????????????cell??????????????????????????????cell??????
        pot?????????????????????curT?????????????????????nextP??????????????????overP???
        ???????????????????????????nextP???overP???????????????????????????????????????????????????????????????????????????
        ???????????????????????????????????????????????????????????????????????????????????????????????????
        */
        // ???????????????????????????????????????????????????????????????????????????????????????????????????????????????
        while (i-- > 0)
            // updateCell???????????????????????????Potential???
            // ??????????????????????????????G???????????????????????????????????????????????????????????????????????????
            updateCell(costs, potential, *pb++);

        /*
        step6 ??????????????????????????????????????????????????????
        ?????????????????????nextP????????????cell?????????curP????????????????????????
        ???nextP??????cell??????????????????????????????overP??????cell???
        currentBuffer_ <=> nextBuffer_  swap buffers
        */
        currentEnd_ = nextEnd_;
        nextEnd_ = 0;
        // ???????????????????????????
        pb = currentBuffer_;
        // ??????????????????????????????????????????        
        currentBuffer_ = nextBuffer_;
        nextBuffer_ = pb;

        // step7 ???????????????????????????????????????????????????????????????????????????????????????
        // ???nextP??????cell??????????????????????????????overP??????cell????????????????????????
        if (currentEnd_ == 0)
        {
            // step8 ????????????????????????????????????????????????????????????????????????????????????????????????????????????
            // ?????????????????????
            threshold_ += priorityIncrement_;    // increment priority threshold
            // ???????????????????????????
            currentEnd_ = overEnd_;    // set current to overflow block
            overEnd_ = 0;
            pb = currentBuffer_;        // swap buffers
            currentBuffer_ = overBuffer_;
            // ??????over ???????????????
            overBuffer_ = pb;
        }

        // ??????????????????????????????????????????????????????????????????Potential???????????????????????????????????????
        // ???????????????????????????????????????????????????????????????????????????
        if (potential[startCell] < POT_HIGH)
            break;

        // step9 ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
    }

    ROS_INFO("CYCLES: %d/%d ", cycle, cycles);
    
    if (cycle < cycles)
        return true; // finished up here
    else
        return false;
}


//??????????????????
#define INVSQRT2 0.707106781
// ????????????: ???????????????????????????????????????????????????
// ??????????????????????????????????????????????????????????????????
// ????????????????????????
// ????????????????????????,??????????????????

/*
updateCell???????????????????????????Potential??????
???????????????cell???????????????potarr??????????????????????????????ta???
*/
inline void DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n) 
{
    // ???????????????????????? ??????1
    cells_visited_++;

    // ????????????????????????????????????
    // cost?????????cell???????????????????????????????????????cost???????????????Potential???
    // ???????????????????????????Potential?????????????????????????????????
    float c = getCost(costs, n);
    // ?????????????????????????????????????????????????????????????????????????????????????????????????????????
    if (c >= lethal_cost_)    // don't propagate into obstacles
        return;
    /*
    ?????????????????????G ??? pot  =  ?????????????????????potential + ?????????costs 
    notice: ??????????????????????????? G??? ????????????potential ?????????????????????????????????????????????????????????
    ?????????G????????????????????????????????????????????????????????????????????????????????????G???
    ???????????????????????????G??????G???(???????????????????????????G?????????????????????)
    */
    float pot = p_calc_->calculatePotential(potential, c, n);

    // ??????????????????????????????????????????
    if (pot < potential[n])
    {
        // ????????????????????? ?????????, ???????????????????????????????????????????????? ?? 
        float le = INVSQRT2 * (float)getCost(costs, n - 1);
        float re = INVSQRT2 * (float)getCost(costs, n + 1);
        float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
        float de = INVSQRT2 * (float)getCost(costs, n + nx_);
        // ???????????????????????????G?????????
        potential[n] = pot;
        //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
        
        // notice: G?????????????????????(????????????????????????)???G?????????????????????
        // threshold????????????????????????
        if (pot < threshold_)    // low-cost buffer block
        {   
            // ????????????????????????????????????????????????(??????)???push???next???????????????????????????????????????????????????
            if (potential[n - 1] > pot + le)
                push_next(n-1);// ??????push???next?????????
            if (potential[n + 1] > pot + re)
                push_next(n+1);
            if (potential[n - nx_] > pot + ue)
                push_next(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_next(n+nx_);
        } 
        else // overflow block???pot >= threshold_
        {
            if (potential[n - 1] > pot + le)
                push_over(n-1);// ??????push???over?????????
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


