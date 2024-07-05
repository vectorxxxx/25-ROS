## 1、安装准备

镜像下载地址：[https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/18.04.6/](https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/18.04.6/)

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
# ros版本需要和Ubuntu版本一致
# Ubuntu20.04 	noetic
# Ubuntu18.04	melodic
# Ubuntu16.04	kinetic
# Ubuntu14.04	indigo
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

## 5、ROS 命令行工具

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

## 6、创建工作空间与功能包

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
# 编译工作空间
cd ~/catkin_ws
catkin_make
catkin_make install

# 创建功能包
cd ~/catkin_ws/src
catkin_create_pkg test_pkg roscpp rospy std_msgs
# 编译功能包
cd ~/catkin_ws
catkin_make

# 设置环境变量
source ~/catkin_ws/devel/setup.bash
# 检查环境变量
echo $ROS_PACKAGE_PATH
```

## 7、发布者Publisher

```bash
# 创建功能包
cd ~/catkin_ws/src
catkin_create_pkg learning_topic roscpp rospy std_msgs geometry_msgs turtlesim

# 创建发布者Publisher
cd ~/catkin_ws/src/learning_topic/src
touch velocity_publisher.cpp
vi velocity_publisher.cpp
```

`velocity_publisher.cpp`

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "velocity_publisher");
    
    // 创建节点句柄
    ros::NodeHandle n;
    
    // 创建发布者，消息类型为geometry_msgs::Twist，发布名为turtle1/cmd_vel，队列长度为10
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    
    // 设置循环频率
    ros::Rate loop_rate(10);
    
    int count = 0;
    while (ros::ok())
    {
        // 初始化geometry_msgs::Twist类型消息
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5;
        vel_msg.angular.z = 0.2;
        
        // 发布消息
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s]",
                vel_msg.linear.x, vel_msg.angular.z);
        
        // 按照循环频率延时
        loop_rate.sleep();
    }
    
    return 0;
}
```

`CMakeLists.txt`

```bash
add_executable(velocity_publisher src/velocity_publisher.cpp)
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})
```

编译运行

```bash
# 编译
cd ~/catkin_ws
catkin_make

# 设置环境变量
source ~/catkin_ws/devel/setup.bash
# 如果不想每次或者怕忘记设置环境变量，可以按如下方式修改
cd ~
# 显示隐藏文件
Ctrl+H
vi .bashrc
# 添加设置环境变量命令（路径改成自己的）
source /home/vectorx/catkin_ws/devel/setup.bash

# 重启终端才能生效
roscore
rosrun turtlesim turtlesim_node
rosrun learning_topic velocity_publisher
```

------------------------------------------------------------------------------------------------

同理，python实现如下：

```bash
# 创建发布者Publisher
mkdir -p ~/catkin_ws/src/learning_topic/scripts
cd ~/catkin_ws/src/learning_topic/scripts
touch velocity_publisher.py
vi velocity_publisher.py
```

`velocity_publisher.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
    # ROS节点初始化
    rospy.init_node('velocity_publisher', anonymous=True)
    
    # 创建发布者，发布名为/turtle1/cmd_vel', 消息类型为geometry_msgs::Twist，队列长度为10
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # 设置循环的频率
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # 初始化geometry_msgs::Twist类型的消息
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.2
        
        # 发布消息
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s]",
				vel_msg.linear.x, vel_msg.angular.z)
        
        # 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```

## 8、订阅者Subscriber

```bash
cd ~/catkin_ws/src/learning_topic/src
touch pose_subscriber.cpp
vi pose_subscriber.cpp
```

`pose_subscriber.cpp`

```cpp
#include <ros/ros.h>
#include "turtlesim/Pose.h"

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f", msg->x, msg->y);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pose_subscriber");
    
    // 创建节点句柄
    ros::NodeHandle n;
    
    // 创建订阅者，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);
    
    // 循环等待回调函数
    ros::spin();
    
    return 0;
}
```

`CMakeLists.txt`

```bash
add_executable(pose_subscriber src/pose_subscriber.cpp)
target_link_libraries(pose_subscriber ${catkin_LIBRARIES})
```

编译运行

```bash
# 编译
cd ~/catkin_ws
catkin_make

# 因为之前已在.bashrc中添加过环境变量，这一步跳过

# 运行
roscore
rosrun turtlesim turtlesim_node
rosrun learning_topic pose_subscriber
rosrun turtlesim turtle_teleop_key
```

------------------------------------------------------------------------------------------------

同理，python实现如下：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.msg import Pose

def poseCallback(msg):
    rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)

def pose_subscriber():
	# ROS节点初始化
    rospy.init_node('pose_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("/turtle1/pose", Pose, poseCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()
```

运行之前需要勾选为可执行

![image-20240630161456720](https://s2.loli.net/2024/06/30/ycRuEBN2zW95kOV.png)

Python版本的无需编译，直接运行即可

```bash
rosrun learning_topic pose_subscriber.py
```

## 9、话题消息的定义与使用

### 9.1、话题消息的定义

```bash
mkdir -p ~/catkin_ws/src/learning_topic/msg
cd ~/catkin_ws/src/learning_topic/msg
touch Person.msg
vi Person.msg
```

`Person.msg`

```msg
string name
uint8 sex
uint8 age

uint8 unknown = 0
uint8 male    = 1
uint8 female  = 2
```

`package.xml`

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

`CMakeLists.txt`

```txt
find_package(......
  message_generation
)

add_message_files(FILES Person.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(......
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs turtlesim message_runtime
)
```

编译

```bash
catkin_make
```

### 9.2、话题消息的使用

`person_publisher.cpp`

```cpp
#include <ros/ros.h>
#include "learning_topic/Person.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "person_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher person_info_pub = n.advertise<learning_topic::Person>("/person_info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        // 初始化learning_topic::Person类型的消息
    	learning_topic::Person person_msg;
		person_msg.name = "Tom";
		person_msg.age  = 18;
		person_msg.sex  = learning_topic::Person::male;

        // 发布消息
		person_info_pub.publish(person_msg);
       	ROS_INFO("Publish Person Info: name:%s  age:%d  sex:%d", 
				  person_msg.name.c_str(), person_msg.age, person_msg.sex);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
```

`person_subscriber.cpp`

```cpp
#include <ros/ros.h>
#include "learning_topic/Person.h"

// 接收到订阅的消息后，会进入消息回调函数
void personInfoCallback(const learning_topic::Person::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg->name.c_str(), msg->age, msg->sex);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "person_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, personInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
```

`CMakeLists.txt`

```txt
add_executable(person_publisher src/person_publisher.cpp)
target_link_libraries(person_publisher ${catkin_LIBRARIES})
add_dependencies(person_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(person_subscriber src/person_subscriber.cpp)
target_link_libraries(person_subscriber ${catkin_LIBRARIES})
add_dependencies(person_subscriber ${PROJECT_NAME}_generate_messages_cpp)
```

编译

```bash
catkin_make
```

使用

```bash
roscore
rosrun learning_topic person_subscriber
rosrun learning_topic person_publisher
```

--------------------------------------------------------------------------------

同理，Python实现如下：

`person_publisher.py`

```python
import rospy
from learning_topic.msg import Person

def velocity_publisher():
	# ROS节点初始化
    rospy.init_node('person_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    person_info_pub = rospy.Publisher('/person_info', Person, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
		# 初始化learning_topic::Person类型的消息
    	person_msg = Person()
    	person_msg.name = "Tom";
    	person_msg.age  = 18;
    	person_msg.sex  = Person.male;

		# 发布消息
        person_info_pub.publish(person_msg)
    	rospy.loginfo("Publsh person message[%s, %d, %d]", 
				person_msg.name, person_msg.age, person_msg.sex)

		# 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```

`person_subscriber.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from learning_topic.msg import Person

def personInfoCallback(msg):
    rospy.loginfo("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg.name, msg.age, msg.sex)

def person_subscriber():
	# ROS节点初始化
    rospy.init_node('person_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/person_info", Person, personInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()
```

## 10、客户端Client

```bash
# 创建功能包
cd ~/catkin_ws/src
catkin_create_pkg learning_service roscpp rospy std_msgs geometry_msgs turtlesim
```

`turtle_spawn.cpp`

```cpp
#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "turtle_spawn");
    
    // 创建节点句柄
    ros::NodeHandle node;
    
    // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");
        
    // 初始化turtlesim::Spawn的数据
    turtlesim::Spawn srv;
    srv.request.x = 2.0;
    srv.request.y = 2.0;
    srv.request.name = "turtle2";
    ROS_INFO("Call service to spawn turtle[x:%0.6f, y:%0.6f, name:%s]",
             srv.request.x, srv.request.y, srv.request.name.c_str());
    
    // 请求服务调用
    add_turtle.call(srv);
    // 显示服务调用结果
    ROS_INFO("Spawn turtle successful [name:%s]", srv.response.name.c_str());
    
    return 0;
}
```

配置编译规则 `CMakeLists.txt`

```bash
add_executable(turtle_spawn src/turtle_spawn.cpp)
target_link_libraries(turtle_spawn ${catkin_LIBRARIES})
```

编译运行客户端

```bash
cd ~/catkin_ws
catkin_make
# 成功后，catkin_ws/devel/lib/learning_service下会生成可执行文件turtle_spawn

roscore
rosrun turtlesim turtlesim_node
rosrun learning_service turtle_spawn
```

------------------------------------------------------------------------------------------------

同理，python实现如下：

`turtle_spawn.py`

```python
import sys
import rospy
from turtlesim.srv import Spawn

def turtle_spawn():
	# ROS节点初始化
    rospy.init_node('turtle_spawn')

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/spawn')
    try:
        add_turtle = rospy.ServiceProxy('/spawn', Spawn)

		# 请求服务调用，输入请求数据
        response = add_turtle(2.0, 2.0, 0.0, "turtle2")
        return response.name
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	#服务调用并显示调用结果
    print "Spwan turtle successfully [name:%s]" %(turtle_spawn())
```

## 11、服务端Server

`turtle_command_server.cpp`

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

ros::Publisher turtle_vel_pub;
bool pubCommand = false;

bool commandCallback(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res)
{
    // 显示请求数据
    pubCommand = !pubCommand;
    ROS_INFO("Publish turtle velocity command [%s]", pubCommand ? "Yes" : "No");
    
    // 设置反馈数据
    res.success = true;
    res.message = "Change turtle command state!";
	
    return true;
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "turtle_command_server");
    
    // 创建节点句柄
    ros::NodeHandle n;
   
    // 创建一个名为/turtle_command的server，注册回调函数commandCallback
    ros::ServiceServer command_service = n.advertiseService("/turtle_command", commandCallback);
    
    // 创建一个Publisher，发布名为/turtle1/cmd_vel，消息类型为geometry_msgs::Twist，队列长度为10
    turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    
    // 循环等待回调函数
    ROS_INFO("Ready to receive turtle command.");
    
    // 设置循环的频率
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        // 查看一次回调函数队列
        ros::spinOnce();
        
        if (pubCommand)
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.5;
            vel_msg.angular.z = 0.2;
            turtle_vel_pub.publish(vel_msg);
        }
        
        // 按照循环频率延时
        loop_rate.sleep();
    }
    
    return 0;
}
```

查看 std_srvs/Trigger消息结构

```bash
rossrv show std_srvs/Trigger
---
bool success
string message
```

配置编译规则 `CMakeLists.txt`

```bash
add_executable(turtle_command_server src/turtle_command_server.cpp)
target_link_libraries(turtle_command_server ${catkin_LIBRARIES})
```

编译运行客户端

```bash
cd ~/catkin_ws
catkin_make
# 查看 ~/catkin_ws/devel/lib/learning_service 下会生成可执行文件turtle_spawn

roscore
rosrun turtlesim turtlesim_node
rosrun learning_service turtle_command_server
rosservice call /turtle_command "{}"
```

------------------------------------------------------------------------------------------------

同理，python实现如下：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import thread,time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

pubCommand = False;
turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

def command_thread():	
	while True:
		if pubCommand:
			vel_msg = Twist()
			vel_msg.linear.x = 0.5
			vel_msg.angular.z = 0.2
			turtle_vel_pub.publish(vel_msg)
			
		time.sleep(0.1)

def commandCallback(req):
	global pubCommand
	pubCommand = bool(1-pubCommand)

	# 显示请求数据
	rospy.loginfo("Publish turtle velocity command![%d]", pubCommand)

	# 反馈数据
	return TriggerResponse(1, "Change turtle command state!")

def turtle_command_server():
	# ROS节点初始化
    rospy.init_node('turtle_command_server')

	# 创建一个名为/turtle_command的server，注册回调函数commandCallback
    s = rospy.Service('/turtle_command', Trigger, commandCallback)

	# 循环等待回调函数
    print "Ready to receive turtle command."

    thread.start_new_thread(command_thread, ())
    rospy.spin()

if __name__ == "__main__":
    turtle_command_server()
```

## 12、服务数据的定义与使用

### 12.1、服务数据的定义

```bash
mkdir -p ~/catkin_ws/src/learning_service/srv
cd ~/catkin_ws/src/learning_service/srv
touch Person.srv
vi Person.srv
```

`Person.srv`

```
string name
uint8  age
uint8  sex

uint8  unknown = 0
uint8  male    = 1
uint8 female   = 2
---
string result
```

`package.xml`

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

`CMakeLists.txt`

```txt
find_package(......
  message_generation
)

add_service_files(FILES Person.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(......
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs turtlesim message_runtime
)
```

编译

```bash
catkin_make
# 查看 ~/catkin_ws/devel/include/learning_service下面是否存在下列三个文件
Person.h
PersonRequest.h
PersonResponse.h
```

### 12.2、服务数据的使用

`person_client.cpp`

```cpp
#include <ros/ros.h>
#include "learning_service/Person.h"

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "person_client");

    // 创建节点句柄
	ros::NodeHandle node;

    // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
	ros::service::waitForService("/show_person");
	ros::ServiceClient person_client = node.serviceClient<learning_service::Person>("/show_person");

    // 初始化learning_service::Person的请求数据
	learning_service::Person srv;
	srv.request.name = "Tom";
	srv.request.age  = 20;
	srv.request.sex  = learning_service::Person::Request::male;

    // 请求服务调用
	ROS_INFO("Call service to show person[name:%s, age:%d, sex:%d]", 
			 srv.request.name.c_str(), srv.request.age, srv.request.sex);

	person_client.call(srv);

	// 显示服务调用结果
	ROS_INFO("Show person result : %s", srv.response.result.c_str());

	return 0;
};
```

`person_server.cpp`

```cpp
#include <ros/ros.h>
#include "learning_service/Person.h"


// service回调函数，输入参数req，输出参数res
bool personCallback(learning_service::Person::Request  &req,
         			learning_service::Person::Response &res)
{
    // 显示请求数据
    ROS_INFO("Person: name:%s  age:%d  sex:%d", req.name.c_str(), req.age, req.sex);

	// 设置反馈数据
	res.result = "OK";

    return true;
}

int main(int argc, char **argv)ac
{
    // ROS节点初始化
    ros::init(argc, argv, "person_server");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个名为/show_person的server，注册回调函数personCallback
    ros::ServiceServer person_service = n.advertiseService("/show_person", personCallback);

    // 循环等待回调函数
    ROS_INFO("Ready to show person informtion.");

    ros::spin();

    return 0;
}
```

`CMakeLists.txt`

```txt
add_executable(person_server src/person_server.cpp)
target_link_libraries(person_server ${catkin_LIBRARIES})
add_dependencies(person_server ${PROJECT_NAME}_gencpp)

add_executable(person_client src/person_client.cpp)
target_link_libraries(person_client ${catkin_LIBRARIES})
add_dependencies(person_client ${PROJECT_NAME}_gencpp)
```

编译

```bash
catkin_make
# 查看 ~/catkin_ws/devel/lib/learning_service 下是否存在下面两个可执行文件
# person_client
# person_server
```

使用

```bash
roscore
rosrun learning_service person_server
rosrun learning_service person_client
```

--------------------------------------------------------------------------------

同理，python实现如下：

`person_client.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from learning_service.srv import Person, PersonRequest

def person_client():
	# ROS节点初始化
    rospy.init_node('person_client')

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/show_person')
    try:
        person_client = rospy.ServiceProxy('/show_person', Person)

		# 请求服务调用，输入请求数据
        response = person_client("Tom", 20, PersonRequest.male)
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	#服务调用并显示调用结果
    print "Show person result : %s" %(person_client())
```

`person_server.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from learning_service.srv import Person, PersonResponse

def personCallback(req):
	# 显示请求数据
    rospy.loginfo("Person: name:%s  age:%d  sex:%d", req.name, req.age, req.sex)
	# 反馈数据
    return PersonResponse("OK")

def person_server():
	# ROS节点初始化
    rospy.init_node('person_server')

	# 创建一个名为/show_person的server，注册回调函数personCallback
    s = rospy.Service('/show_person', Person, personCallback)

	# 循环等待回调函数
    print "Ready to show person informtion."
    rospy.spin()

if __name__ == "__main__":
    person_server()
```

## 13、参数的使用

使用准备

```bash
# 创建功能包
cd ~/catkin_ws/src
catkin_create_pkg learning_parameter roscpp rospy

# 启动小海龟
roscore
rosrun turtlesim turtlesim_node
```

参数命令

```bash
# 查询所有参数
rosparam list
# 查询背景色
rosparam get /turtlesim/background_b
rosparam get /turtlesim/background_g
rosparam get /turtlesim/background_r
# 查询发行版
rosparam get /rosdistro
rosparam get /rosversion

# 修改参数
rosparam set /turtlesim/background_b 100
# 修改并不能立即生效，需要发送clear请求
rosservice call /clear "{}"

# 转存参数
rosparam dump param.yaml
# 加载参数
# 修改 param.yaml 文件内容后执行（验证同理）
rosparam load param.yaml

# 删除参数（验证同理）
rosparam delete /turtlesim/background_b
```

代码实现

`parameter_config.cpp`

```cpp
#include <string.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
    int red, green, blue;
    
    // 初始化ROS节点
    ros::init(argc, argv, "parameter_config");
    
    // 创建节点句柄
    ros::NodeHandle node;
    
    // 读取背景色参数
    ros::param::get("/turtlesim/background_r", red);
    ros::param::get("/turtlesim/background_g", green);
    ros::param::get("/turtlesim/background_b", blue);
    ROS_INFO("Get Background Color[%d, %d, %d]", red, green, blue);
    
    // 设置背景色参数
    red   = 255;
    green = 255;
    blue  = 255;
    ros::param::set("/turtlesim/background_r", red);
	ros::param::set("/turtlesim/background_g", green);
	ros::param::set("/turtlesim/background_b", blue);
    ROS_INFO("Set Background Color[%d, %d, %d]", red, green, blue);

    // 读取背景色参数
    ros::param::get("/turtlesim/background_r", red);
    ros::param::get("/turtlesim/background_g", green);
    ros::param::get("/turtlesim/background_b", blue);
    ROS_INFO("Re-get Background Color[%d, %d, %d]", red, green, blue);
    
    // 调用服务，刷新背景色
    ros::service::waitForService("/clear");
    ros::ServiceClient clear_background = node.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clear_background.call(srv);
    
    sleep(1);
    
    return 0;
}
```

配置编译规则 `CMakeLists.txt`

```bash
add_executable(parameter_config src/parameter_config.cpp)
target_link_libraries(parameter_config ${catkin_LIBRARIES})
```

编译运行客户端

```bash
cd ~/catkin_ws
catkin_make
# 查看 ~/catkin_ws/devel/lib/learning_service 下会生成可执行文件turtle_spawn

roscore
rosrun turtlesim turtlesim_node
rosrun learning_parameter parameter_config
rosservice call /turtle_command "{}"
```

------------------------------------------------------------------------------------------------

同理，Python实现如下：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from std_srvs.srv import Empty

def parameter_config():
	# ROS节点初始化
    rospy.init_node('parameter_config', anonymous=True)

	# 读取背景颜色参数
    red   = rospy.get_param('/turtlesim/background_r')
    green = rospy.get_param('/turtlesim/background_g')
    blue  = rospy.get_param('/turtlesim/background_b')
    rospy.loginfo("Get Backgroud Color[%d, %d, %d]", red, green, blue)
    
	# 设置背景颜色参数
    red   = 255;
    green = 255;
    blue  = 255;
    rospy.set_param("/turtlesim/background_r", red);
    rospy.set_param("/turtlesim/background_g", green);
    rospy.set_param("/turtlesim/background_b", blue);
    rospy.loginfo("Set Backgroud Color[%d, %d, %d]", red, green, blue)

	# 读取背景颜色参数
    red   = rospy.get_param('/turtlesim/background_r')
    green = rospy.get_param('/turtlesim/background_g')
    blue  = rospy.get_param('/turtlesim/background_b')
    rospy.loginfo("Get Backgroud Color[%d, %d, %d]", red, green, blue)

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/clear')
    try:
        clear_background = rospy.ServiceProxy('/clear', Empty)
        
		# 请求服务调用，输入请求数据
        response = clear_background()
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    parameter_config()
```

## 14、ROS坐标系管理系统

```bash
# 安装tf包
sudo apt-get install ros-melodic-turtle-tf

# 启动launch脚本
roslaunch turtle_tf turtle_tf_demo.launch

# 运行海龟键盘控制节点
rosrun turtlesim turtle_teleop_key

# tf树工具
rosrun tf view_frames
# 打开当前文件夹下的frames.pdf文件即可

# 坐标变换关系命令行工具
rosrun tf tf_echo turtle1 turtle2

# 坐标变换关系可视化工具（三维关系）
rosrun rviz rviz -d `rospack find turtle_tf` /rviz/turtle_rviz.rviz
# 1、Global Options > Fixed Frame 修改为 world
# 2、Add > By display type > rviz > tf > OK
```

## 15、tf坐标系广播与监听

```bash
# 创建功能包
cd ~/catkin_ws/src
catkin_create_pkg learning_tf roscpp rospy tf turtlesim
```

`turtle_tf_broadcaster.cpp`

```cpp
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    // 创建tf广播器
    tf::TransformBroadcaster br;
    
    // 初始化tf数据
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    
    // 广播world与海龟坐标系的tf数据
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "my_tf_broadcaster");
    
    // 输入参数作为海龟的名字
    if (argc != 2)
    {
        ROS_ERROR("need turtle name as argument");
        return -1;
    }
	turtle_name = argv[1];    

    // 订阅海龟的位姿话题
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name + "/pose", 10, &poseCallback);
    
    // 循环等待回调函数
    ros::spin();
    
    return 0;
}
```

`turtle_tf_listener.cpp`

```cpp
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "my_tf_listener");
    
    // 创建节点句柄
    ros::NodeHandle node;
    
    // 请求产生turtle2
    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);
    
    // 创建发布turtle2速度控制指令的发布者
    ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
    // 创建tf的监听器
    tf::TransformListener listener;
    
    ros::Rate rate(10.0);
    while(node.ok())
    {
        // 获取turtle1与turtle2坐标系之间的tf数据
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        // 根据turtle1与turtle2坐标系之间的位置关系，发布turtle2速度控制指令
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
				                        transform.getOrigin().x());
		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
				                      pow(transform.getOrigin().y(), 2));
        turtle_vel.publish(vel_msg);
        
        rate.sleep();
    }
    
    return 0;
}
```

配置编译规则 `CMakeLists.txt`

```bash
add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
```

编译运行客户端

```bash
cd ~/catkin_ws
catkin_make
# 查看 ~/catkin_ws/devel/lib/learning_tf 下会生成可执行文件 turtle_tf_broadcaster 和 turtle_tf_listener

roscore
rosrun turtlesim turtlesim_node
# ros重映射机制，控制节点名称不同，防止冲突
rosrun learning_tf turtle_tf_broadcaster __name:=turtle1_tf_broadcaster /turtle1
rosrun learning_tf turtle_tf_broadcaster __name:=turtle2_tf_broadcaster /turtle2
rosrun learning_tf turtle_tf_listener
rosrun turtlesim turtle_teleop_key

# 报错日志：[ERROR] [1720012351.995199109]: "turtle2" passed to lookupTransform argument target_frame does not exist. [ERROR] [1720012356.027073796]: Lookup would require extrapolation into the past. Requested time 1720012353.161287154 but the earliest data is at time 1720012353.788391871, when looking up transform from frame [turtle1] to frame [turtle2]
```

------------------------------------------------------------------------------------------------

同理，python实现如下：

`turtle_tf_broadcaster.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
```

`turtle_tf_listener.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()
```

编译运行客户端

```bash
cd ~/catkin_ws
catkin_make
# 查看 ~/catkin_ws/devel/lib/learning_tf 下会生成可执行文件 turtle_tf_broadcaster 和 turtle_tf_listener

roscore
rosrun turtlesim turtlesim_node
# ros重映射机制，控制节点名称不同，防止冲突
rosrun learning_tf turtle_tf_broadcaster.py __name:=turtle1_tf_broadcaster _turtle:=turtle1
rosrun learning_tf turtle_tf_broadcaster.py __name:=turtle2_tf_broadcaster _turtle:=turtle2
rosrun learning_tf turtle_tf_listener.py
rosrun turtlesim turtle_teleop_key
```

## 16、launch启动文件

```bash
cd ~/catkin_ws/src
catkin_create_pkg learning_launch

mkdir ~/catkin_ws/src/learning_launch/launch
cd ~/catkin_ws/src/learning_launch/launch
touch simple.launch
```

`simple.launch`

```xml
<launch>
	<node pkg="learning_topic" type="person_subscriber" name="listener" output="screen"/>
    <node pkg="learning_topic" type="person_publisher" name="talker" output="screen"/>
</launch>
```

编译运行

```bash
cd ~/catkin_ws
catkin_make

roslaunch learning_launch simple.launch
```

`turtlesim_parameter_config.launch`

```xml
<launch>
	<param name="/turtle_number" value="2"/>
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node">
		<param name="turtle_name1" value="Tom"/>
		<param name="turtle_name2" value="Jerry"/>
		<rosparam file="$(find learning_launch)/config/param.yaml" command="load"/>
	</node>
    <node pkg="turtlesim" type="turtle_teleop_key" name="turtle_teleop_key" output="screen"/>
</launch>
```

`param.yaml`

```yaml
A: 123
B: "hello"

group:
  C: 456
  D: "hello"
```

编译运行

```bash
# 编译
cd ~/catkin_ws
catkin_make

# 启动
roslaunch learning_launch turtlesim_parameter_config.launch
# 验证
rosparam list
rosparam get /turtle_number
rosparam get /turtlesim_node/turtle_name1
rosparam get /turtlesim_node/turtle_name2
rosparam get /turtlesim_node/A
rosparam get /turtlesim_node/B
rosparam get /turtlesim_node/group/C
rosparam get /turtlesim_node/group/D
```

`start_tf_demo_c++.launch`

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>
    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
    <node pkg="learning_tf" type="turtle_tf_broadcaster" name="turtle1_tf_broadcaster" args="/turtle1"/>
    <node pkg="learning_tf" type="turtle_tf_broadcaster" name="turtle2_tf_broadcaster" args="/turtle2"/>
    <node pkg="learning_tf" type="turtle_tf_listener" name="listener"/>
</launch>
```

`start_tf_demo_py.launch`

```xml
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="sim"/>
	<node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
	<node pkg="learning_tf" type="turtle_tf_broadcaster.py" name="turtle1_tf_broadcaster">
	  <param name="turtle" type="string" value="turtle1" />
	</node>
	<node pkg="learning_tf" type="turtle_tf_broadcaster.py" name="turtle2_tf_broadcaster">
	  <param name="turtle" type="string" value="turtle2" /> 
	</node>
    <node pkg="learning_tf" type="turtle_tf_listener.py" name="listener" />
</launch>
```

`turtlesim_remap.launch`

```xml
<launch>
	<include file="$(find learning_launch)/launch/simple.launch" />
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node">
		<remap from="/turtle1/cmd_vel" to="/cmd_vel"/>
	</node>
</launch>
```

运行

```bash
# 启动
roslaunch learning_launch turtlesim_remap.launch

# 验证 /turtle1/cmd_vel 是否变成了 /cmd_vel
rostopic list
# 发布 /cmd_vel 话题，验证是否生效
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

## 17、可视化工具

### 17.1、Qt工具箱

```bash
# 日志输出工具
rqt_console
```

![image-20240704232024629](https://s2.loli.net/2024/07/04/vj745JAb9hIyW1N.png)

```bash
# 计算图可视化工具
rqt_graph
```

![image-20240704232157744](https://s2.loli.net/2024/07/04/4CAYIQV19kSxNl5.png)

```bash
# 数据绘图工具
rqt_plot
```

![image-20240704231951423](https://s2.loli.net/2024/07/04/KE4o5biTLBxWdJr.png)

```bash
# 图像渲染工具
rqt_image_view
```

![image-20240704232311320](https://s2.loli.net/2024/07/04/3qvY8MmI9JZaEip.png)

```bash
# 工具集成
rqt
```

![image-20240704232453207](https://s2.loli.net/2024/07/04/kJNTmohSGU6blfv.png)

![image-20240704232620663](https://s2.loli.net/2024/07/04/6nZRHYOBmhrXoLp.png)

### 17.2、Rviz

![image-20240704233014818](https://s2.loli.net/2024/07/04/pbJWSxKNtAqMD9j.png)

![image-20240704233026597](https://s2.loli.net/2024/07/04/TZuwWsabelnkC2f.png)

```bash
# 启动 
roscore
rosrun rviz rviz
```

### 17.3、Gazebo

![image-20240704233353053](https://s2.loli.net/2024/07/04/7m2oZtPxwSeFOW1.png)

![image-20240704233505370](https://s2.loli.net/2024/07/04/4Z1ICLuxXjFfw6h.png)

```bash
# 启动
roslaunch gazebo_ros willowgarage_world.launch
```

