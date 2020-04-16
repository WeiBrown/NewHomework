实现了传统方法使用aiml的语音交互机器人
视频中的终端:
1.    roscore
2.    roslaunch rchomeedu_speech lm.launch dict:=/home/dell/dev/src/homework/src/talkbot/config/command.dic lm:=/home/dell/dev/src/homework/src/talkbot/config/command.lm
3.    rostopic echo /lm_data
4.    ./recognizer.py
5.    roslaunch start_server.launch
6.    rosrun sound_play sndplay_node.py
7.    ./robot_tts.py
