# 多机器人自动避障
## 1.配置环境与文件说明
将src下的各文件夹复制到本地对应的文件夹中，最好再次 catkin_make 编译一下。（其实只是在原来turtlebot的子文件夹中加入了几个launch、xml文件，为了不逐个复制，故将所有有文件加入的文件夹都放在了提供的src中。）

我们编写了几个插件来使障碍物运动，详见报告。在“插件”文件夹中启动终端，运行如下命令，复制插件到系统存放gazebo插件的文件夹中：
```sh
sudo cp libanimated_box.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/libanimated_box.so
sudo cp libanimated_box1.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/libanimated_box1.so
sudo cp libsquare_box.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/libsquare_box.so
sudo cp libsquare_box2.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/libsquare_box2.so
sudo cp libsquare_box3.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/libsquare_box3.so
sudo cp libsquare_box4.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/libsquare_box4.so
```
然后可以将home1.world和home2.world放入适当位置，这是我们创建的两种环境。最后是我们写好的动态避障算法代码，以apf_new.py文件为准。
## 2.启动gazebo
单机器人对应环境启动命令：
```sh
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=...
```
多机器人对应环境启动命令：
```sh
roslaunch turtlebot_gazebo turtlebot_world_.launch world_file:=...
```
在省略处加上home1.world或者home2.world的文件路径。
## 3.运行程序
在保存python代码文件的终端运行命令：
```
python apf_new.py
```
之后即可看到演示效果。