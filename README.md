# ros-3d-vision-guided-robotic-arm-control
本项目是山东大学本科生东雪莲（东雪莲狂热粉丝，非B站虚拟主播本人）的毕业设计《基于ROS的3D视觉机械臂》的仿真部分。

操作系统：Ubuntu 20.04
ROS 版本：Noetic

本项目有四个包：
marm_description-urdf文件展示，部分插件储存在这个包。
marm_gazebo-控制器，部分xacro、urdf文件和gazebo仿真环境的启动文件
marm_moveit_config-通过moveit_setup_assistant插件配置的moveit
marm_vision-部分测试程序，全过程抓取程序未公开（主要是写得太烂了不好意思发）

打开项目方式：
安装好相关库后，输入命令行roslaunch marm_gazebo marm_bringup_moveit.launch+rosrun marm_vision ().py，括号内是marm_vision中的python程序名。

如有疑问，可以关注知乎账号東雪蓮Official（https://www.zhihu.com/people/seren-azuma-official）并私信联系作者

由于不知名原因，很多插件在作者的虚拟机上都不可用，所以抓取方式使用最简单粗暴的物理方法，即制作“T”字形工件并在机械臂夹爪上加装倒钩。如有更好方法引入夹取插件欢迎改进。

关注东雪莲喵，关注东雪莲谢谢喵
全世界最可爱的莲莲公主的B站空间：https://space.bilibili.com/1437582453?spm_id_from=333.337.0.0
