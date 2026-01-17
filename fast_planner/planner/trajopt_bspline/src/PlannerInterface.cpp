#include "../include/grad_replanner/PlannerInterface.h"

namespace Planner
{

    PlannerInterface::PlannerInterface()
    {

    }

    PlannerInterface::~PlannerInterface()
    {

    }

    void PlannerInterface::initParam(double max_vel,double max_acc)
    {
        _on_replan = true;

        succ_replan_num_ = 0;
        _MAX_Vel = max_vel;
        _MAX_Acc = max_acc;

        _bspline_replanner.reset(new BsplineReplanner());
        _bspline_replanner->initialize(_MAX_Vel, _MAX_Acc);

    }
    
    void PlannerInterface::initEsdfMap(double x_size,double y_size,double z_size,double resolution_, Eigen::Vector3d origin,double inflate_values)
    {
        _sdf_map.reset(new SDFMap);
        _sdf_map->initMap(x_size,y_size,z_size,resolution_,origin,inflate_values);
    }

    void PlannerInterface::setCurrentPose(PathPoint pose)
    {
        current_pose_[0] = pose.x;
        current_pose_[1] = pose.y;
        current_pose_[2] = 0.2;
    }


    void PlannerInterface::setPathPoint(std::vector<PathPoint> &plan_traj)
    {
        _global_plan_traj_.clear();
        _global_plan_traj_ = plan_traj;
    }
    
    void PlannerInterface::setObstacles(std::vector<ObstacleInfo> &obstacle)
    {
        _sdf_map->updateMapRolling(current_pose_);
        for(int i = 0; i < obstacle.size();i++)
        {
            Eigen::Vector3d obstacle_pos;
            obstacle_pos[0] = obstacle[i].x;
            obstacle_pos[1] = obstacle[i].y;
            obstacle_pos[2] = 0.2;
            _sdf_map->addLaserPoints(obstacle_pos, 1);
        }

        _sdf_map->startUpdateMapInfo(current_pose_);
    }


    void PlannerInterface::getLocalEsdfMap(std::vector<ObstacleInfo> &obstacle)
    {
        std::vector<Eigen::Vector3d> points;
        _sdf_map->getObstacleWorldPoints(points);
        for(int i = 0;i < points.size();i++)
        {
            Eigen::Vector3d obs = points[i];
            ObstacleInfo temp;
            temp.x = obs[0];
            temp.y = obs[1];
            obstacle.push_back(temp);
        }
    }

    void PlannerInterface::makePlan()
    {
    
        vector<Eigen::Vector3d> traj_pts;

        for(int i = 0; i< _global_plan_traj_.size();i++)
        {
            Vector3d plan_pt(_global_plan_traj_[i].x,_global_plan_traj_[i].y,0.2);
            traj_pts.push_back(plan_pt);
        }

        MatrixXd acc(2, 3), vel(2, 3);

        Vector3d start_vel(0.0 ,0, 0);
        Vector3d target_vel(0.0,0, 0);

        Vector3d start_acc(0.0, 0.0, 0.0);
        Vector3d target_acc(0.0, 0.0,  0.0);

        vel.row(0) = start_vel;
        vel.row(1) = target_vel;

        acc.row(0) = start_acc;
        acc.row(1) = target_acc;

        double plan_time_interval  = 0.1;
        double check_time_interval = 0.1;
        double local_to_global_time = 0;

        auto start = std::chrono::system_clock::now();

        gradLocalReplan(traj_pts, plan_time_interval, vel, acc, local_to_global_time);

        auto end = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        printf("MotionPlanner Total Running Time: %d  ms \n", elapsed.count());
    }

    void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results)
    {
        plan_traj_results = _plan_traj_results_;
    }

    void PlannerInterface::gradLocalReplan( const vector<Vector3d> & traj_pts, const double & time_interval, const MatrixXd & vel, const MatrixXd & acc , double local_to_global_time)
    {
        /* generate sample points set on a naive trajectory*/
        vector<Eigen::Vector3d> start_end_derivative;
        start_end_derivative.push_back(vel.row(0));
        start_end_derivative.push_back(vel.row(1));
        start_end_derivative.push_back(acc.row(0));
        start_end_derivative.push_back(acc.row(1));

        Eigen::MatrixXd pos_v(2, 3);
        Eigen::MatrixXd vel_v = vel;
        Eigen::MatrixXd acc_v = acc;

        pos_v.row(0) = traj_pts.front();
        pos_v.row(1) = traj_pts.back();
        
        for(int i = 0;i < traj_pts.size();i++)
        {
            std::cout << traj_pts[i] << std::endl;
        }

        /* ========================================================================== */
        _bspline_replanner->setInput(_sdf_map, time_interval, traj_pts, start_end_derivative);
        _bspline_replanner->resetLambda2();
        bool traj_feas = false;
        bool traj_safe = false;
        Eigen:Vector3d collision_pt;
        NonUniformBspline traj_opt;

        Eigen::MatrixXd ctrl_pts;
        NonUniformBspline::BsplineParameterize(time_interval, traj_pts,
                                            start_end_derivative, ctrl_pts);
        ctrl_pts_Vis = ctrl_pts;
        ctrl_pts_Vis = ctrl_pts;
        int iter = 0;
        while (iter < 5)
        {
            if (traj_feas && traj_safe) 
                break;
            
            /* ---------- call replanning ---------- */
            _bspline_replanner->optimize(true);
            traj_opt = _bspline_replanner->getTraj();
            BsplineFeasibleCheck(traj_opt, traj_feas, traj_safe, collision_pt);

            /* ---------- fill local minima; enlarge lambda2 ---------- */
            if (!traj_safe)
            {
                _bspline_replanner->renewLambda2(0.05);
            }

            iter++;
        }
        
        traj_safe = true;

        if( traj_safe )
        { 
            quadrotor_msgs::Bspline safe_spline = getBsplineTraj(traj_opt);
            double t_bspline_cmd_start, t_bspline_cmd_end;
            traj_opt.getRegion(t_bspline_cmd_start, t_bspline_cmd_end);

            double replan_traj_time = t_bspline_cmd_end - t_bspline_cmd_start;
            double time_extra = replan_traj_time - _replan_time_length;

            safe_spline.time_extra = time_extra;
            safe_spline.replan_to_global_time = local_to_global_time;

            _on_replan = true;

            getTraj(traj_opt);
            succ_replan_num_ += 3;
        }
        else{
            // do nothing
            cout<<"no safe replan, try it in next loop"<<endl;
        }

    }

    void PlannerInterface::BsplineFeasibleCheck(NonUniformBspline traj, bool & is_feas, bool & is_safe, Eigen::Vector3d & collision_pt)
    {
        Vector3d pos, vel, acc;
        double t_bspline_start, t_bspline_end, t_duration;
        NonUniformBspline vel_traj = traj.getDerivative();  
        NonUniformBspline acc_traj = vel_traj.getDerivative();  

        traj.getRegion(t_bspline_start, t_bspline_end);   
        t_duration = t_bspline_end - t_bspline_start;

        is_safe = true;
        is_feas = true;
        collision_pt.setZero();
            
        Eigen::Vector3d grad;
        double dis;
        for(double t = 0.0; t < t_duration; t += 0.05)
        {
            pos =     traj.evaluateDeBoor(t_bspline_start + t);
            vel = vel_traj.evaluateDeBoor(t_bspline_start + t);
            acc = acc_traj.evaluateDeBoor(t_bspline_start + t);

            for(int i = 0; i < 3; i++)
            {
                if( fabs(vel(i)) > _MAX_Vel || fabs(acc(i)) > _MAX_Acc )
                    is_feas = false;
            }
            
            dis = _sdf_map->getDistWithGradTrilinear(pos,  grad );
            
            if( dis <  0.075 )
            {
                    is_safe = false;
                    if(collision_pt.norm() < 1e-5) collision_pt = pos;
            }
        }

    }

    quadrotor_msgs::Bspline PlannerInterface::getBsplineTraj(NonUniformBspline & traj_opt)
    {
        quadrotor_msgs::Bspline bspline;
        bspline.order = 3;

        bspline.start_time = _replan_request_time;
        bspline.traj_id    = _replan_traj_id ++;

        Eigen::MatrixXd ctrl_pts = traj_opt.getControlPoint();

        for (int i = 0; i < ctrl_pts.rows(); ++i) 
        {
            Eigen::Vector3d pvt = ctrl_pts.row(i);
            geometry_msgs::Point pt;
            pt.x = pvt(0);
            pt.y = pvt(1);
            pt.z = pvt(2);
            bspline.pts.push_back(pt);
        }
        std::cout << "  bspline.pts size =" <<  bspline.pts.size() << std::endl;

        Eigen::VectorXd knots = traj_opt.getKnot();

        for (int i = 0; i < knots.rows(); ++i) 
        {
            bspline.knots.push_back(knots(i));
        }
        std::cout << "   knots size =" <<  knots.size() << std::endl;

        return bspline;
    }

    void PlannerInterface::getTraj(NonUniformBspline bspline)
    {
        vector<Eigen::Vector3d> traj_pts;
        double tm, tmp;
        bspline.getRegion(tm, tmp);

        NonUniformBspline vel_traj = bspline.getDerivative();  
        NonUniformBspline acc_traj = vel_traj.getDerivative();  
        _plan_traj_results_.clear();
        for (double t = tm; t <= tmp; t += 0.01) 
        {
            Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
            Vector3d  vel = vel_traj.evaluateDeBoor(t);
            traj_pts.push_back(pt);

            PathPoint temp;
            temp.x = pt[0];
            temp.y = pt[1];
            temp.z = pt[2];
            temp.v =  sqrt(pow(vel[0],2) + pow(vel[1],2));
            _plan_traj_results_.push_back(temp);
        }
    }

}