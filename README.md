# planner
该仓库是一个规划的整体框架，是我个人对规划的理解实践内容，该仓库将持续更新。

## 运行

需要有Eigen，Osqp-Eigen库

1. ``git clone https://github.com/Raiden49/planner.git``
2.  ``cd planner``
3. ``cmake -B ./build``
4. ``cmake --build ./build``
5. ``source ./build/devel/setup.bash(zsh)``
6. ``ros launch planner plan.launch``

## 项目内容

### 1. 全局规划

目前只使用了A*算法，待更新

### 2. 路径优化

- 二次规划优化路径(QP)
- 分段三次贝塞尔曲线优化路径(Bezier)
- 三次B样条曲线优化路径(BSpline)

### 3. 局部规划

- 三次样条生成几百条路径
- 五次多项式生成几十条路径
- 最优路径选择与后处理优化 

### 4. 控制器仿真

对于轨迹跟踪，并没有使用实际的控制器，只是利用ROS的TF变换进行模拟控制器完美跟踪路径的操作

## 效果展示

### 1. 全局规划

暂略

### 2. 路径优化

|                            效果图                            | 优化算法 |
| :----------------------------------------------------------: | -------- |
| <img src="B:\something_for_work\proj_github\planner\docs\optim_reult.png" alt="optim_reult" style="zoom:50%;" /> | -        |
| <img src="B:\something_for_work\proj_github\planner\docs\qp_result.png" alt="qp_result" style="zoom:50%;" /> | QP       |
| <img src="B:\something_for_work\proj_github\planner\docs\bezier_result.png" alt="bezier_result" style="zoom:50%;" /> | Bezier   |
| <img src="B:\something_for_work\proj_github\planner\docs\b_spline_result.png" alt="b_spline_result" style="zoom:50%;" /> | BSpline  |

### 3. 局部规划

|                            效果图                            | 算法       |
| :----------------------------------------------------------: | ---------- |
| <img src="B:\something_for_work\proj_github\planner\docs\offline_result.png" alt="offline_result" style="zoom:50%;" /> | 三次样条   |
| <img src="B:\something_for_work\proj_github\planner\docs\online_result.png" alt="online_result" style="zoom:50%;" /> | 五次多项式 |

### 4. 控制器跟踪

规划路径为蓝色，优化后的全局路径为绿色，最终局部规划引导的轨迹为粉红色

<img src="B:\something_for_work\proj_github\planner\docs\final_result.png" alt="final_result" style="zoom:50%;" />

## 更新日志

### 2024/06/26

初版本上传

## 目前存在的BUG

- 前后两个周期局部规划计算出的最优路径可能相差较大，导致抖动较为严重，尤其是路径数量较多的offline三次样条方法
