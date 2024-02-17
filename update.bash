cp -r monkey_interface/ /home/robot-user/ws_moveit/src/
catkin build monkey_interface
source /home/robot-user/ws_moveit/devel/setup.bash

scp -r monkey_listener/ rm@10.42.0.2:/home/rm/ws_moveit/src/

