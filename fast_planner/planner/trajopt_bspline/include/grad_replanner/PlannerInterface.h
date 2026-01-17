/**
 * @brief     Trajectory Optimize Base Bspline
 * @author    juchunyu@qq.com
 * @date      2025-05-10 17:22:01
 * @copyright Copyright (c) 2026, Institute of Robotics Planning and Control (irpc). All rights reserved.
 */
#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#include <time.h>
#include <random>
#include <memory>


#include "../quadrotor_msgs/Bspline.h"

#include "../grad_replanner/backward.hpp"
#include "../grad_replanner/bezier_base.h"

#include "../grad_replanner/grad_band_optimizer.h"
#include "../grad_replanner/non_uniform_bspline.h"
#include "../grad_replanner/bspline_replanner.h"

using namespace std;
using namespace Eigen;

#define INIT_TRAJ_ID 0
#define OPT_TRAJ_ID 1
#define INIT_PT_ID 2
#define OPT_PT_ID 3

namespace  Planner
{
    struct PathPoint
    {
        float x;
        float y;
        float z;
        float v;
    };

    struct ObstacleInfo
    {
        float x;
        float y;
        float z;
    };

    class PlannerInterface
    {


        private:

            double _MAX_Vel, _MAX_Acc;
            int _replan_traj_id = 1;
            shared_ptr<SDFMap> _sdf_map;
            unique_ptr<BsplineReplanner> _bspline_replanner;
            vector<Vector3d> _traj_pts;  
            Eigen::MatrixXd ctrl_pts_Vis; 
            double _replan_time_length;
            int succ_replan_num_;

            std::vector<PathPoint> _global_plan_traj_;
            std::vector<PathPoint> _plan_traj_results_;

            Eigen::Vector3d current_pose_;

            bool _on_replan;
            quadrotor_msgs::Time _replan_request_time;

        public:
            PlannerInterface();
            ~PlannerInterface();
            void initParam(double max_vel,double max_acc);
            void initEsdfMap(double x_size,double y_size,double z_size,double resolution_, Eigen::Vector3d org,double inflate_values);
            void setCurrentPose(PathPoint pose);
            void setPathPoint(std::vector<PathPoint> &plan_traj);
            void setObstacles(std::vector<ObstacleInfo> &obstacle);
            void makePlan();
            void getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results);
            void getLocalEsdfMap(std::vector<ObstacleInfo> &obstacle);



        private:

            void gradLocalReplan( const vector<Vector3d> & traj_pts, const double & time_interval, const MatrixXd & vel, const MatrixXd & acc , double local_to_global_time);
           
            void BsplineFeasibleCheck(NonUniformBspline traj, bool & is_feas, bool & is_safe, Eigen::Vector3d & collision_pt);
           
            quadrotor_msgs::Bspline getBsplineTraj(NonUniformBspline & traj_opt);
            
            void getTraj(NonUniformBspline bspline);


          
        
    };


}






#endif

