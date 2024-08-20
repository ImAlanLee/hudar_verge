<!--
 * @Author: AlanLee
 * @Date: 2024-08-20 13:30:31
 * @LastEditors: xuxin lisian_magic@163.com
 * @LastEditTime: 2024-08-20 18:36:24
 * @FilePath: /hudar_verge/README.md
 * @Description: 
-->
# hudar_verge
radar station





# Hudar2025

**总体思路：**采用相机纯视觉仿射变换方案，激光雷达辅助，多相机方案。

**预期目标：**

1. 实现我方半场到地方基地准确定位
2. 实现与哨兵联动，补盲，提供视野
3. 对常见位置进行猜测，留下便于修改接口
4. 利用激光雷达进行结果校正。
5. 完善录像，赛场数据记录方便后续调试。
6. 利用卡尔曼，匈牙利优化目标跟踪结果。
7. 实现便于修改参数代码架构。

**未来展望：**利用Mid-70实现点云聚类直接定位。


# Usage

以指定视频文件启动
```shell
ros2 run hudar_camera publish_video --ros-args -p /radar/video_path:="'/home/lisian/RM_workspace/CvTest/t1.mp4'"
```