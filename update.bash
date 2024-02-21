#git pull
cp -r /home/robot-user/monkey_robot_codebase/monkey_interface/ /home/robot-user/ws_moveit/src/

cd /home/robot-user/ws_moveit || exit
catkin build monkey_interface
source devel/setup.bash

scp -r /home/robot-user/monkey_robot_codebase/monkey_listener/ rm@10.42.0.2:/home/rm/ws_moveit/src/

cd /home/robot-user/monkey_robot_codebase || exit