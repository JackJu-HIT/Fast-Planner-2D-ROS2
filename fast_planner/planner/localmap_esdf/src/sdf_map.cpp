#include "sdf_map.h"

/* =================================================================
   1. 初始化与基础转换
   ================================================================= */

void SDFMap::initMap(double x_size, double y_size, double z_size, double resolution_, Eigen::Vector3d org, double inflate_values)
{
  inflate_val_ = inflate_values;
  ground_z_ = 1.0;
  resolution = resolution_;
  resolution_inv = 1.0 / resolution;

  origin = org;
  map_size = Eigen::Vector3d(x_size, y_size, z_size);

  for (int i = 0; i < 3; ++i)
    grid_size(i) = ceil(map_size(i) / resolution);

  min_range = origin;
  max_range = origin + map_size;
  last_fill_pt.setZero();

  int total_size = grid_size(0) * grid_size(1) * grid_size(2);
  occupancy_buffer.resize(total_size);
  distance_buffer.resize(total_size);
  occupancy_buffer_inflate_.resize(total_size);
  occupancy_buffer_neg.resize(total_size);
  distance_buffer_neg.resize(total_size);
  distance_buffer_all.resize(total_size);
  tmp_buffer1.resize(total_size);
  tmp_buffer2.resize(total_size);
  cache_all_.resize(total_size);
  cache_hit_.resize(total_size);

  fill(occupancy_buffer.begin(), occupancy_buffer.end(), 0);
  fill(occupancy_buffer_neg.begin(), occupancy_buffer_neg.end(), 0);
  fill(occupancy_buffer_inflate_.begin(), occupancy_buffer_inflate_.end(), 0);
  fill(distance_buffer.begin(), distance_buffer.end(), 10000.0);
  fill(distance_buffer_neg.begin(), distance_buffer_neg.end(), 10000.0);
  fill(distance_buffer_all.begin(), distance_buffer_all.end(), 10000.0);

  std::cout << "ESDF 地图初始化完成!" << std::endl;
}

void SDFMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin(i)) * resolution_inv);
}

void SDFMap::indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * resolution + origin(i);
}

bool SDFMap::isInMap(Eigen::Vector3d pos)
{
  if (pos(0) < min_range(0) || pos(1) < min_range(1) || pos(2) < min_range(2) ||
      pos(0) > max_range(0) || pos(1) > max_range(1) || pos(2) > max_range(2))
  {
    return false;
  }
  return true;
}

/* =================================================================
   2. 滚动窗口逻辑 (每一帧更新原点并重置)
   ================================================================= */

void SDFMap::updateMapRolling(Eigen::Vector3d robot_pos)
{
  Eigen::Vector3d new_origin = robot_pos - map_size / 2.0;
  
  for (int i = 0; i < 3; ++i) {
    new_origin(i) = floor(new_origin(i) * resolution_inv) * resolution;
  }

  origin = new_origin;
  min_range = origin;
  max_range = origin + map_size;

  fill(occupancy_buffer.begin(), occupancy_buffer.end(), 0);
  fill(occupancy_buffer_inflate_.begin(), occupancy_buffer_inflate_.end(), 0);
  fill(occupancy_buffer_neg.begin(), occupancy_buffer_neg.end(), 0);
  fill(distance_buffer.begin(), distance_buffer.end(), 10000.0);
  fill(distance_buffer_neg.begin(), distance_buffer_neg.end(), 10000.0);
  fill(distance_buffer_all.begin(), distance_buffer_all.end(), 10000.0);

  fill(cache_all_.begin(), cache_all_.end(), 0);
  fill(cache_hit_.begin(), cache_hit_.end(), 0);
  while (!cache_voxel_.empty()) cache_voxel_.pop();

  esdf_min_ = Eigen::Vector3i::Zero();
  esdf_max_ = (grid_size - Eigen::Vector3i::Ones());
}

void SDFMap::startUpdateMapInfo(Eigen::Vector3d robot_pos)
{
    // updateMapRolling(robot_pos);
    // raycastProcess();
    // clearAndInflateLocalMap();
    // updateESDF3d();
    // Step 1: 处理 cache_voxel_ 里的当前帧点云
    raycastProcess();

    // Step 2: 膨胀障碍物 (包含 Z 轴 0-50 层逻辑)
    clearAndInflateLocalMap();

    // Step 3: 计算 ESDF
    updateESDF3d();
}

/* =================================================================
   3. 占据处理与垂直投影膨胀
   ================================================================= */

void SDFMap::addLaserPoints(Eigen::Vector3d pos, int occ)
{
    if (occ != 1 && occ != 0)
    {
      std::cout << "occ != 1 && occ != " << std::endl;
      return;
    }
    if (!isInMap(pos)) 
    {
      std::cout << "!isInMap(pos)" << std::endl;
      return;
    }
    std::cout << "success add laser point" << std::endl;
    Eigen::Vector3i id;
    posToIndex(pos, id);
    int idx_ctns = id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2);

    if (cache_all_[idx_ctns] == 0) {
        cache_voxel_.push(id);
    }
    cache_all_[idx_ctns] += 1;
    if (occ == 1) cache_hit_[idx_ctns] = 1;
}

void SDFMap::raycastProcess()
{
  while (!cache_voxel_.empty())
  {
    Eigen::Vector3i idx = cache_voxel_.front();
    cache_voxel_.pop();
    int idx_ctns = idx(0) * grid_size(1) * grid_size(2) + idx(1) * grid_size(2) + idx(2);
    
    if(cache_hit_[idx_ctns] > 0)
      occupancy_buffer[idx_ctns] = 1;
    else 
      occupancy_buffer[idx_ctns] = 0;
  }
}

void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts)
{
  int num = 0;
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

void SDFMap::clearAndInflateLocalMap()
{
  int inf_step = ceil(inflate_val_ / resolution);
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  Eigen::Vector3i inf_pt;

  for (int x = 0; x < grid_size(0); ++x)
  {
    for (int y = 0; y < grid_size(1); ++y)
    {
      for (int z = 0; z < grid_size(2); ++z)
      {
        if (occupancy_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] > 0)
        {
          // --- 保留原代码对 Z 轴的限制逻辑：投影模式 ---
          for (int z1 = 0; z1 <= 50; ++z1)
          {
            if (z1 >= grid_size(2)) break;

            inflatePoint(Eigen::Vector3i(x, y, z1), inf_step, inf_pts);
            for (int k = 0; k < (int)inf_pts.size(); ++k)
            {
                inf_pt = inf_pts[k];
                if (inf_pt[0] < 0 || inf_pt[0] >= grid_size(0) ||
                    inf_pt[1] < 0 || inf_pt[1] >= grid_size(1) ||
                    inf_pt[2] < 0 || inf_pt[2] >= grid_size(2)) 
                {
                    continue;
                }
                occupancy_buffer_inflate_[inf_pt[0] * grid_size(1) * grid_size(2) + 
                                         inf_pt[1] * grid_size(2) + inf_pt[2]] = 1;
            }
          }
        }
      }
    }
  }
}

/* =================================================================
   4. Planner 查询接口 (解决 Undefined Reference)
   ================================================================= */

double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad)
{
  if (!isInMap(pos))
  {
    grad.setZero();
    return 0;
  }

  Eigen::Vector3d pos_m = pos - 0.5 * resolution * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * resolution_inv;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      for (int z = 0; z < 2; z++)
      {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];

  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;

  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * resolution_inv;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]) +
            (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]) +
            diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]) +
            diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= resolution_inv;

  return dist;
}

double SDFMap::getDistance(Eigen::Vector3d pos)
{
  if (!isInMap(pos)) return -1.0;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return distance_buffer_all[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)];
}

double SDFMap::getDistance(Eigen::Vector3i id,int sign)
{
  id(0) = std::max(std::min(id(0), grid_size(0) - 1), 0);
  id(1) = std::max(std::min(id(1), grid_size(1) - 1), 0);
  id(2) = std::max(std::min(id(2), grid_size(2) - 1), 0);
  return distance_buffer_all[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)];
}

int SDFMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos)) return -1;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return occupancy_buffer[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] > 0.1 ? 1 : 0;
}

int SDFMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos)) return -1;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return occupancy_buffer_inflate_[id(0) * grid_size(1) * grid_size(2) + id(1) * grid_size(2) + id(2)] > 0 ? 1 : 0;
}

/* =================================================================
   5. 获取所有障碍物世界坐标接口
   ================================================================= */

void SDFMap::getObstacleWorldPoints(std::vector<Eigen::Vector3d>& points)
{
  points.clear();
  Eigen::Vector3d pos;
  for (int x = 0; x < grid_size(0); ++x) {
    for (int y = 0; y < grid_size(1); ++y) {
      for (int z = 0; z < grid_size(2); ++z) {
        int idx = x * grid_size(1) * grid_size(2) + y * grid_size(2) + z;
        if (occupancy_buffer_inflate_[idx] > 0) {
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          points.push_back(pos);
        }
      }
    }
  }
}

/* =================================================================
   6. ESDF 核心计算算法 (Parabolic Envelope)
   ================================================================= */

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
  int v[grid_size(dim)];
  double z[grid_size(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++)
  {
    double s;
    do
    {
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
      if (s <= z[k]) k--;
      else break;
    } while (k >= start);

    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;
  for (int q = start; q <= end; q++)
  {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d()
{
  // 正向 DT
  for (int x = 0; x < grid_size(0); x++) {
    for (int y = 0; y < grid_size(1); y++) {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_inflate_[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] > 0 ? 0.0 : 1000000.0;
          },
          [&](int z, double val) { tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
          0, grid_size(2) - 1, 2);
    }
  }

  for (int x = 0; x < grid_size(0); x++) {
    for (int z = 0; z < grid_size(2); z++) {
      fillESDF([&](int y) { return tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int y, double val) { tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
               0, grid_size(1) - 1, 1);
    }
  }

  for (int y = 0; y < grid_size(1); y++) {
    for (int z = 0; z < grid_size(2); z++) {
      fillESDF([&](int x) { return tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int x, double val) {
                 distance_buffer[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = resolution * std::sqrt(val);
               },
               0, grid_size(0) - 1, 0);
    }
  }

  // 负向 DT
  for (int i = 0; i < (int)occupancy_buffer_inflate_.size(); ++i) {
    occupancy_buffer_neg[i] = (occupancy_buffer_inflate_[i] == 0) ? 0.1 : -0.1;
  }

  for (int x = 0; x < grid_size(0); x++) {
    for (int y = 0; y < grid_size(1); y++) {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_neg[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] > 0 ? 0.0 : 1000000.0;
          },
          [&](int z, double val) { tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
          0, grid_size(2) - 1, 2);
    }
  }

  for (int x = 0; x < grid_size(0); x++) {
    for (int z = 0; z < grid_size(2); z++) {
      fillESDF([&](int y) { return tmp_buffer1[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int y, double val) { tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = val; },
               0, grid_size(1) - 1, 1);
    }
  }

  for (int y = 0; y < grid_size(1); y++) {
    for (int z = 0; z < grid_size(2); z++) {
      fillESDF([&](int x) { return tmp_buffer2[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z]; },
               [&](int x, double val) {
                 distance_buffer_neg[x * grid_size(1) * grid_size(2) + y * grid_size(2) + z] = resolution * std::sqrt(val);
               },
               0, grid_size(0) - 1, 0);
    }
  }

  // 合并得到最终距离场
  for (int i = 0; i < grid_size(0) * grid_size(1) * grid_size(2); ++i) {
    if (distance_buffer_neg[i] > 0.0)
      distance_buffer_all[i] = distance_buffer[i] - distance_buffer_neg[i] + resolution;
    else
      distance_buffer_all[i] = distance_buffer[i];
  }
}