## 1、安装准备

```bash
# 打开终端
Ctrl+Alt+T

# 更新可用软件包列表
sudo apt-get update
# 安装这可用软件包的最新版本
sudo apt-get upgrade

# 查看系统版本
cat /proc/version
uname -a
lsb_release -a
```

## 2、安装 g++ 编译器和 python 解释器

```bash
# 安装 g++ 编译器
sudo apt-get install g++
# E: 无法获得锁 /var/lib/dpkg/lock-frontend - open (11: 资源暂时不可用)
# E: 无法获取 dpkg 前端锁 (/var/lib/dpkg/lock-front)
ps -e | grep apt  
sudo kill [PID]
sudo rm /var/lib/dpkg/lock-frontend 
sudo rm /var/cache/apt/archives/lock
sudo apt-get update

# 安装 python 解释器
sudo apt-get install python -y
```

## 3、编译运行与解释执行

```bash
# 编译
g++ c++_for.cpp -o c++_for
# 运行
./c++_for

# 解释执行
python python_for.py
```

## 4、安装 ROS

```bash
# 1、添加ROS软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2、添加密钥
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 3、安装 ROS
sudo apt update
sudo apt install ros-melodic-desktop-full

# 4、初始化 rosdep
sudo apt install python-rosdep
sudo rosdep init
#######
sudo gedit /usr/lib/python2.7/dist-packages/rosdep2/sources_list.py
# def download_rosdep_data(url):
# url = "https://mirror.ghproxy.com/" + url
sudo gedit /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py
# gbpdistro_url = "https://mirror.ghproxy.com/" + gbpdistro_url
sudo gedit /usr/lib/python2.7/dist-packages/rosdistro/__init__.py
sudo gedit /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py
sudo gedit /usr/lib/python2.7/dist-packages/rosdep2/sources_list.py
sudo gedit /usr/lib/python2.7/dist-packages/rosdep2/rep3.py
sudo gedit /usr/lib/python2.7/dist-packages/rosdistro/manifest_provider/github.py
# https://raw.githubusercontent.com/ 前添加 https://mirror.ghproxy.com/
#######
rosdep update

# 5、设置环境变量
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 6、安装 rosinstall
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## 5、ROS 命令行

```bash
# 启动 ros master
roscore

# 启动小海龟仿真器
rosrun turtlesim turtlesim_node

# 启动海龟控制节点
rosrun turtlesim turtle_teleop_key

# 图形化工具
rqt_graph

# 节点信息
rosnode list
rosnode info turtlesim

# 话题信息
rostopic list
# 发布一次话题
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
# 以10Hz频率发布话题
rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0"

# 查看ros消息数据结构
rosmsg show geometry_msgs/Twist

# 服务请求列表
rosservice list
# 发布服务请求
rosservice call /spawn "x: 2.0
y: 2.0
theta: 0.0
name: 'turtle2'"

# 话题记录
rosbag record -a -O cmd_record
# 话题复现
rosbag play cmd_record.bag
```

