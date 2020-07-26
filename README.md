## Task 03
### What this repository for
* 熟悉粒子滤波的相关原理及操作

### Usage

```bash
$ git clone https://github.com/b51/robot_tutorial_03
$ cd robot_tutorial_03
$ mkdir build
$ cd build
$ cmake ..
$ make -j
$ ./particle_filter -num [particles number]
```

### Task
* 代码中 ideal_pose 表示没有任何噪声的理想控制指令得到的位姿, 图中用红色表示
* 代码中 real_pose 表示由于噪声干扰得到的实际位姿, 图中用蓝色表示
* 图中绿色的表示粒子滤波得到的位姿
* 设定机器人的观测范围为3.5m, 视角为60度
* 地图为半场地图，宽4.5m，球门柱分别位于 (4.0, +-1.5) 处

* 补充根据控制命令更新粒子位姿的函数 ParticleFilter::UpdatePose 
* 补充根据观测更新粒子权重的函数 ParticleFilter::UpdateParticlesWeight
* 补充更新权重后更新粒子的函数 ParticleFilter::Resample
* [扩展] 如果已知起始位姿态，如何初始化粒子更合适
* [扩展] 如果是全场地图，会出现什么问题，该如何解决
* [扩展] 将图片内容转换到 ROS rviz 内显示，包括场地，粒子位姿，理想位姿，真实位姿，粒子滤波位姿(以及真实位姿及粒子滤波姿态的不确定度)

### Reference
https://www.khanacademy.org/math/statistics-probability/modeling-distributions-of-data/more-on-normal-distributions/v/introduction-to-the-normal-distribution

### Solution
* 参考解决方案参照 solution branch

```bash
$ git pull origin
$ git checkout solution
```
