#!/bin/bash
. /opt/ros/indigo/setup.bash  # Change directory to ros setup
. ~/WORKSPACE_ROS/devel/setup.bash # Change directory to workspace environment
#roscore &
scene_dir="/home/chalikonda/WORKSPACE_ROS/worlds/environment2.ttt" # Giving root to scene to be open
echo "Scene direcoty is $scene_dir"
VREP_ROOT="/home/chalikonda/WORKSPACE_ROS/src/V-REP_PRO_EDU_V3_2_1_64_Linux/"  # After pwd/continue/to/Vrep/Installation folder
cd "$VREP_ROOT"
echo "Vrep directory is $VREP_ROOT" # Will print the directory on terminal to cross check vrep installation folder
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$VREP_ROOT"
./vrep.sh -s ~/WORKSPACE_ROS/worlds/environment2.ttt "$@" 
exit 0


#./bivis_shell.sh instead of . bivis_shell.sh. In case of using ./bivis_shell.sh directory will not change while for . bivis_shell.sh directory will change.
# to start vrep, loading scene and start simulation commands can help
# ./vrep.sh -s ~/ros/workspace/worlds/xxx.ttt ---> This is with simulation
# ./vrep.sh ~/ros/workspace/worlds/xxx.ttt ---> Without simulation
# For more details refer to http://www.coppeliarobotics.com/helpFiles/en/commandLine.htm
