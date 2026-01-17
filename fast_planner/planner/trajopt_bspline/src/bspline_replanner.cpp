#include <grad_replanner/bspline_replanner.h>

BsplineReplanner::BsplineReplanner(/* args */) {}

BsplineReplanner::~BsplineReplanner() {}

void BsplineReplanner::initialize(double max_v, double max_a) {
  traj_optimizer.reset(new GradBandOptimizer());
  traj_optimizer->setParameterAuto(max_v, max_a);

  vel_limit = 2.0;
  acc_limit = 1.5;

  // ros::param::get("/local_replanner/max_vel", vel_limit);
  // ros::param::get("/local_replanner/max_acc", acc_limit);
}

void BsplineReplanner::setInput(const shared_ptr<SDFMap>& map, const double dt,
                                const vector<Eigen::Vector3d>& pt_set,
                                const vector<Eigen::Vector3d>& sed) {
  delta_t = dt;
  point_set = pt_set;
  start_end_derivative = sed;
  num_seg_ori = pt_set.size() - 1;
  num_seg_opt = -1;
  length_ori = getTrajLength(pt_set);
  traj_optimizer->setDistanceField(map);
}

void BsplineReplanner::renewLambda1(double lamda1_) {
  traj_optimizer->renewLambda1(lamda1_);
}

void BsplineReplanner::renewLambda2(double lamda2_) {
  traj_optimizer->renewLambda2(lamda2_);
}

void BsplineReplanner::resetLambda2() {
  traj_optimizer->resetLambda2();
}

bool BsplineReplanner::optimize(bool adjust_time) {
  /* parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::BsplineParameterize(delta_t, point_set,
                                         start_end_derivative, ctrl_pts);
  
  // std::cout << "delta_t =" << delta_t << " point_set.szie =" << point_set.size() << " start_end_derivative.szie =" << start_end_derivative.size() 
  // << " ctrl_pts =" << ctrl_pts << std::endl;
  /* ---------- optimization ---------- */
  traj_optimizer->setControlPoints(ctrl_pts);
  traj_optimizer->setBSplineInterval(delta_t);

  // std::cout << "optimizeTrajFixedEnd() 11" << std::endl;

  traj_optimizer->optimizeTrajFixedEnd();

  // std::cout << "optimizeTrajFixedEnd() 22" << std::endl;


  ctrl_pts = traj_optimizer->getControlPoints();
  NonUniformBspline traj_opt = NonUniformBspline(ctrl_pts, 3, delta_t);
  
  // std::cout << "optimizeTrajFixedEnd() 33" << std::endl;
  /* ---------- time adjustment ---------- */
  bool feasible;
  if (adjust_time) {
    traj_opt.setDynamicsLimit(vel_limit, acc_limit);
    feasible = traj_opt.checkFeasibility();

    for (int iter_num = 0; iter_num <= 2; ++iter_num) {
      if (feasible) break;

      traj_opt.adjustTime();
      feasible = traj_opt.checkFeasibility();
    }
  }

  // 6td::cout << "optimizeTrajFixedEnd() 44" << std::endl;

  traj = traj_opt;
  /* ---------- prepare for next iteration ---------- */
  point_set.clear();

  double tm, tmp;
  traj_opt.getRegion(tm, tmp);
  double duration = tmp - tm;
  
  if (num_seg_opt == -1) {
    /* recompute segment num */
    double length = getTrajLength(traj_opt.getControlPoint());
    // std::cout << "length =" << length << std::endl;
    num_seg_opt = double(num_seg_ori) * length / length_ori;
    // cout << "ori:" << num_seg_ori << ", opt:" << num_seg_opt << endl;
  }
  // std::cout << "optimizeTrajFixedEnd() 441" << std::endl;

  delta_t = duration / double(num_seg_opt);
  // std::cout << "optimizeTrajFixedEnd() 442" << std::endl;

  // std::cout << "tm =" << tm << " tmp =" << tmp << std::endl; 
  for (double t = tm; t <= tmp + 1e-5; t += delta_t) {
    Eigen::Vector3d pt = traj_opt.evaluateDeBoor(t);
    point_set.push_back(pt);
  }

  // std::cout << "optimizeTrajFixedEnd() 55" << std::endl;

  return feasible;
}

NonUniformBspline BsplineReplanner::getTraj() { return this->traj; }

vector<Eigen::Vector3d> BsplineReplanner::getAccControlPts() {
  Eigen::MatrixXd pts = traj.getDerivative().getDerivative().getControlPoint();
  vector<Eigen::Vector3d> acc_ctrl_pts(pts.rows());
  for (int i = 0; i < pts.rows(); ++i) {
    acc_ctrl_pts[i] = pts.row(i).transpose();
  }

  return acc_ctrl_pts;
}
vector<Eigen::Vector3d> BsplineReplanner::getJerkControlPts() {
  //'ros::Time t1, t2;

  NonUniformBspline der = traj.getDerivative();
  der = der.getDerivative();
  der = der.getDerivative();

  Eigen::MatrixXd pts = der.getControlPoint();
  vector<Eigen::Vector3d> jk_ctrl_pts(pts.rows());

  for (int i = 0; i < pts.rows(); ++i) {
    jk_ctrl_pts[i] = pts.row(i).transpose();
  }

  return jk_ctrl_pts;
}

double BsplineReplanner::getTrajLength(const vector<Eigen::Vector3d>& pt_set) {
  double length = 0.0;
  for (int i = 0; i < pt_set.size() - 1; ++i) {
    length += (pt_set[i + 1] - pt_set[i]).norm();
  }
  return length;
}

double BsplineReplanner::getTrajLength(const Eigen::MatrixXd& pt_set) {
  double length = 0.0;
  for (int i = 0; i < pt_set.rows() - 1; ++i) {
    length += (pt_set.row(i + 1) - pt_set.row(i)).norm();
  }
  return length;
}