import os
import sys
import yaml
from rosbag.bag import Bag
import math
# info_dict = yaml.load(Bag('input.bag', 'r')._get_yaml_info())




object_name = sys.argv[1]
particle_number = int(sys.argv[2])
scene_name = sys.argv[3]
# run_alg_flag = sys.argv[4]
rosbag_num = sys.argv[4]
slow_down_rate = sys.argv[5]
slow_down_rate = float(slow_down_rate)

wait_time_rate = math.ceil(1.0 / slow_down_rate)


# input("stop")
# info_dict = yaml.load(Bag(os.path.expanduser(f"~/rosbag/latest_rosbag/{object_name}_{scene_name}/{object_name}_{scene_name}_70_{rosbag_num}.bag"))._get_yaml_info(), Loader=yaml.FullLoader)
# info_dict = yaml.load(Bag(os.path.expanduser(f"~/pyvkdepth/rosbag/2_scene1_new_camera_MayoMilk{rosbag_num}.bag"))._get_yaml_info(), Loader=yaml.FullLoader)
# info_dict = yaml.load(Bag(os.path.expanduser(f"~/pyvkdepth/rosbag/1_{scene_name}_Mustard{rosbag_num}.bag"))._get_yaml_info(), Loader=yaml.FullLoader)
# info_dict = yaml.load(Bag(os.path.expanduser(f"~/pyvkdepth/rosbag/1_{scene_name}_{object_name}{rosbag_num}.bag"))._get_yaml_info(), Loader=yaml.FullLoader)
info_dict = yaml.load(Bag(os.path.expanduser(f"~/pyvkdepth/rosbag/2_scene1_crackersoup1.bag"))._get_yaml_info(), Loader=yaml.FullLoader)
# info_dict = yaml.load(Bag(os.path.expanduser(f"~/rosbag/pointcloud_cracker2.bag"))._get_yaml_info(), Loader=yaml.FullLoader)

dura = (info_dict['duration']) * wait_time_rate + 1

print(dura)
