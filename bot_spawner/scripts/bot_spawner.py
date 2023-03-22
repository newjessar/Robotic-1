#!/usr/bin/python
import roslaunch
import rospkg 
import rospy 

rospy.init_node("bot_spawner")
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_files = {}

def run_launch_file(bot_name, position): 
  cli_args = ["bot_controller", "add_bot.launch",'botname:='+bot_name, 'position:='+str(position)]
  roslaunch_args = cli_args[2:]
  roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
  parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
  parent.start()
  launch_files[bot_name] = parent

## Add bot to spawn here via run_launch_file (bot_name, position)
run_launch_file("bot1", 0)
run_launch_file("bot2", 3)
run_launch_file("bot3", 12)
run_launch_file("bot4", 13)
# run_launch_file("bot5", 3)
# run_launch_file("bot6", 2)

rospy.spin()