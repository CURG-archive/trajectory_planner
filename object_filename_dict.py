
import roslib
import roslib.packages

pkg_path = roslib.packages.get_pkg_dir('object_models')

dir_path = pkg_path + str('models/cgdb/arm_old/')
rock_file_name = dir_path + str('darparock.iv')
drill_file_name = dir_path + str('darpadrill.iv')
hammer_file_name = dir_path + str('darpahammer.iv')
flashlight_file_name = dir_path + str('darpaflashlight.iv')

dir_path = pkg_path + str('models/cgdb/model_database/')
large_shaving_gel_file_name = dir_path + str('gillette_shaving_gel.iv')
coke_file_name = dir_path + str('coke_can.iv')
odwalla_file_name = dir_path + str('odwalla_bottle.iv')
all_file_name = dir_path + str('all.iv')
garnier_file_name = dir_path + str('garnier_shampoo_bottle.iv')
gillette_file_name = dir_path + str('gillette_shaving_gel.iv')
milk_carton_file_name = dir_path + str('milk_carton.ply')

dir_path = pkg_path + str('models/cgdb/semantic/')
drill_custom_file_name = dir_path + str('drill_custom_in_meters.iv')
mug_custom_file_name = dir_path + str('mug_custom_in_meters.iv')
darpaphonehandset_file_name = dir_path + str('darpaphonehandset_1000_different_coordinate_system.iv')

dir_path = pkg_path + str('models/cgdb/ted/')
box_file_name = dir_path + str('box_in_meters.iv')
snapple_file_name = dir_path + str('snapple_in_meters.iv')
library_cup_file_name = dir_path + str('library_cup_in_meters.iv')
krylon_spray_file_name = dir_path + str('krylon_spray_in_meters.iv')


file_name_dict = dict()
file_name_dict['18555'] = rock_file_name
file_name_dict['18557'] = hammer_file_name
file_name_dict['18558'] = drill_file_name
file_name_dict['18716'] = large_shaving_gel_file_name
file_name_dict['18722'] = coke_file_name
file_name_dict['18707'] = odwalla_file_name
file_name_dict['18650'] = all_file_name

file_name_dict['garnier_shampoo_bottle'] = garnier_file_name
file_name_dict['all'] = all_file_name
file_name_dict['odwalla_bottle'] = odwalla_file_name
file_name_dict['darpaflashlight'] = flashlight_file_name
file_name_dict['gillette_shaving_gel'] = gillette_file_name
file_name_dict['milk_carton'] = milk_carton_file_name

file_name_dict['drill_custom'] = drill_custom_file_name
file_name_dict['mug_custom'] = mug_custom_file_name
file_name_dict['darpaphonehandset'] = darpaphonehandset_file_name
file_name_dict['box'] = box_file_name
file_name_dict['snapple'] = snapple_file_name
file_name_dict['darparock'] = rock_file_name
file_name_dict['krylon_spray'] = krylon_spray_file_name
file_name_dict['library_cup'] = library_cup_file_name