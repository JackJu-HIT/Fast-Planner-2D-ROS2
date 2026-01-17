# Fast-Planner-2D-ROS2

## 1. 项目简介 (Introduction)
**Fast-Planner-2D-ROS2** 是一个专为地面移动机器人（UGV）设计的轻量化实时轨迹规划器。

本项目基于 [FastPlanner (JackJu-HIT)](https://github.com/JackJu-HIT/FastPlanner) 进行了深度重构与功能迁移。我们提取了其后端核心的 **B 样条（B-Spline）轨迹优化算法**，并针对地面运动特性进行了“2D 降维适配”。通过在优化逻辑和 ESDF 构建中限制 Z 轴自由度，使该算法能够完美运行在平坦或坡度固定的二维环境。

### 核心改进点：
*   **维度限制与降解**：在 B 样条代价函数与 ESDF 梯度计算中，**强制限制了 Z 轴方向的优化约束**。这使得算法在保留原始框架强大的避障平滑性能的同时，消除了地面小车在三维空间中不必要的纵向抖动。
*   **框架解耦**：完全剥离了原版复杂的全局规划与地图管理逻辑，提供纯粹的 C++ 优化接口，单次规划耗时极低。
*   **ESDF 代码升级**：对比 [老版仓库](https://github.com/JackJu-HIT/FastPlanner)，本项目对 ESDF 的构建效率和索引逻辑进行了优化，更适合实时局部规划。
*   **原生 ROS2 接口**：使用 ROS2 (Humble/Foxy) 作为可视化接口与交互媒介，符合现代机器人软件开发标准。

---

## 2. 环境依赖 (Dependencies)
本项目核心优化器依赖 **NLopt** 非线性优化库。

该依赖源码已附带在 [FastPlanner 老版仓库](https://github.com/JackJu-HIT/FastPlanner/tree/master/files) 的 `files` 目录下。请下载并按照以下命令进行安装：

```bash
# 建议先从原仓库下载 nlopt-2.7.1
cd files/nlopt-2.7.1
mkdir build && cd build
cmake ..
make
sudo make install
```

---

## 3. 🚀 编译与运行 (How to Use)

### 编译项目
在 ROS2 工作空间下使用 `colcon` 工具构建：
```bash
colcon build --symlink-install
```

### 运行规划器
```bash
# 启动 2D 轨迹规划演示
./install/fast_planner/bin/fast_planner_plan
```

### 可视化接口说明 (RViz2)
启动 RViz2 并订阅以下话题，即可实时观测规划效果：

| 话题名称 | 消息类型 | 功能说明 |
| :--- | :--- | :--- |
| `/visual_local_trajectory` | `nav_msgs/Path` | **最终轨迹**：B 样条优化后的二维平滑避障路径 |
| `/visual_global_path` | `nav_msgs/Path` | **参考线**：起始点到目标点的原始参考直线 |
| `/visual_local_obstacles` | `sensor_msgs/PointCloud2` | **感知点云**：规划器当前考虑的局部障碍物分布 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | **交互设置**：在 RViz 中通过 `2D Pose Estimate` 设置机器人起点 |

---

## 4. 运行结果 (Results)
![规划结果展示](https://github.com/JackJu-HIT/Fast-Planner-2D-ROS2/blob/master/fast_planner/result.png)
*(注：图中展示了在限制 Z 轴优化后，算法依然保持了极佳的平滑度与避障能力)*

---

## 5. 参考项目 (References)
*   **[Teach-Repeat-Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan)** (HKUST Aerial Robotics Group)
*   **[FastPlanner 原版 (JackJu-HIT)](https://github.com/JackJu-HIT/FastPlanner)**

---

## 📚 教程与技术支持
关于 **B 样条优化公式推导**、**ESDF 降维实现细节**及更多视频教程，请关注：

*   **微信公众号**：`机器人规划与控制研究所`
*   **深度解析文章**：[点击阅读详细教程](你的文章链接)
*   **B 站视频演示**：[机器人算法研究所](你的B站链接)
