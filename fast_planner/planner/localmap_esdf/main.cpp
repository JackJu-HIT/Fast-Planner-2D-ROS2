/**
 * @Function: esdf map 
 * @Create by:juchunyu@qq.com
 * @Date:2025-03-22 23:49:00
 * @Last modifed:juchunyu@qq.com
 * */
#include "local_perception/sdf_map.h"
#include <memory>
#include <Eigen/Eigen>

int main()
{

    //初始化 
    shared_ptr<SDFMap> _sdf_map;
    _sdf_map.reset(new SDFMap);

    double resolution = 0.1;
    double x_size = 10.0;
    double y_size = 10.0;
    double z_size = 10.0;
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    double inflateValue = 0.15;
    _sdf_map->initMap(x_size,y_size,z_size,resolution,origin,inflateValue);

    //添加未占据的元素的位置
    Eigen::Vector3d unoccupied_pos(2.0, 2.0, 2.0);
    _sdf_map->addLaserPoints(unoccupied_pos, 0);

    //添加占据的元素的位置
    Eigen::Vector3d obstacle_pos(5.0, 5.0, 5.0);
    _sdf_map->addLaserPoints(obstacle_pos, 1);
 
    _sdf_map->startUpdateMapInfo();

    
    //查询点 unoccupied_pos
    Eigen::Vector3d grad;
    double distance = _sdf_map->getDistWithGradTrilinear(unoccupied_pos,grad);
    int unoccupied_pos_res = _sdf_map->getOccupancy(unoccupied_pos);

    //查询点 obstacle_pos
    Eigen::Vector3d grad2;
    int occupancy   =  _sdf_map->getOccupancy(obstacle_pos);
    double distance2 = _sdf_map->getDistWithGradTrilinear(obstacle_pos,grad2);

   
 
    // 输出结果 查询点 unoccupied_pos
    std::cout << "查询点坐标: (" << unoccupied_pos(0) << ", " << unoccupied_pos(1) << ", " << unoccupied_pos(2) << ")" <<
    "  占据状态:"<< unoccupied_pos_res << " 最近障碍物距离:" << distance << " 最近障碍物距离理论值:"<< (unoccupied_pos - obstacle_pos).norm() 
    << "  最近障碍物距离梯度:("<< grad.transpose() << ")\n";

    //  输出结果  查询点 obstacle_pos
    std::cout << "查询点坐标: (" << obstacle_pos(0) << ", " << obstacle_pos(1) << ", " << obstacle_pos(2) << ")" << "  占据状态:"
    << occupancy << " 最近障碍物距离:" << distance2 << "  最近障碍物距离理论值:"<< 0.0000
    << "      最近障碍物距离梯度:("<< grad2.transpose() << ")\n";
  

    return 0;
}