import os
import rospy

param_target_path = rospy.get_param("abs_folder")
sub_dir_list = ["a", "b", "c"]
os.mkdir(param_target_path)

for sub_dir in sub_dir_list:
    os.mkdir(os.path.join(param_target_path, sub_dir))


