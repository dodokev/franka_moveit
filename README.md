# franka_moveit

# USE THIS IF WANT PRIMITIVE SHAPE AS FILTER
ros2 launch robot_self_filter self_filter_tag.launch.py robot_description:="$(xacro
/home/labrob/franka_ros2_ws/src/moveit_resources/panda_moveit_config/config/selfrobot.urdf.xacro)" filter_config:=/home/labrob/franka_ros2_ws/src/robot_self_filter/params/example.yaml

# WHAT RECORD BAG
ros2 bag record /points_filtered_tag /points_raw_tag -o franka_bag
