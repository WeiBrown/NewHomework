实现语音应答控制机器人导航


终端说明:
语音模块：

1.roslaunch rchomeedu_speech lm.launch dict:=/home/dell/dev/src/homework/src/nav/config/command.dic lm:=/home/dell/dev/src/homework/src/nav/config/command.lm
2../recognizer.py #用于识别语音
3.roslaunch start_server.launch   #启动aiml对话系统
4.rosrun sound_play soundplay_node.py   #soundplay节点用于讲机器人语音说出来
5../robot_tts.py     向soundplay节点发送消息
6.roscore

导航模块：
#地图用kolourpaint处理，提前建好地图
1.roslaunch turtlebot_gazebo turtlebot_world.launch   #gazebo仿真

2.roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/dell/my_map.yaml #载入地图进行导航

3.roslaunch turtlebot_rviz_launchers view_navigation.launch #rviz查看

4../navigation.py #核心节点接受消息


