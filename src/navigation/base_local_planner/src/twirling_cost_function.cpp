/*
 * twirling_cost_function.cpp
 *
 *  Created on: Apr 20, 2016
 *      Author: Morgan Quigley
 */

#include <base_local_planner/twirling_cost_function.h>

#include <math.h>

namespace base_local_planner {

// 对当前轨迹打分
double TwirlingCostFunction::scoreTrajectory(Trajectory &traj) 
{
  // 代价值 == 角速度，角速度越大，代价越大
  return fabs(traj.thetav_);  // add cost for making the robot spin
}

} /* namespace base_local_planner */
