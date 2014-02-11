
import roslib
import roslib.packages
import rospy

pkg_path = roslib.packages.get_pkg_dir('object_models') + '/'



def get_path(package_name, resource_name):
    resources = roslib.packages.find_resource(package_name, resource_name)
    if len(resources) == 0:
        rospy.logerr("Failed to find resource %s in package %s"%(resource_name, package_name))
        return ""
    else:
        return resources[0]


rock_file_name = get_path('object_models','darparock.iv')
drill_file_name = get_path('object_models','darpadrill.iv')
hammer_file_name = get_path('object_models','darpahammer.iv')
flashlight_file_name = get_path('object_models','darpaflashlight.iv')


large_shaving_gel_file_name = get_path('object_models','gillette_shaving_gel.iv')
coke_file_name = get_path('object_models','coke_can.iv')
odwalla_file_name = get_path('object_models','odwalla_bottle.iv')
all_file_name = get_path('object_models','all.iv')
garnier_file_name = get_path('object_models','garnier_shampoo_bottle.iv')
gillette_file_name = get_path('object_models','gillette_shaving_gel.iv')
milk_carton_file_name = get_path('object_models','milk_carton.ply')


drill_custom_file_name = get_path('object_models','drill_custom_in_meters.iv')
mug_custom_file_name = get_path('object_models','mug_custom_in_meters.iv')
darpaphonehandset_file_name = get_path('object_models','darpaphonehandset_1000_different_coordinate_system.iv')


box_file_name = get_path('object_models','box_in_meters.iv')
snapple_file_name = get_path('object_models','snapple_in_meters.iv')
library_cup_file_name = get_path('object_models','library_cup_in_meters.iv')
krylon_spray_file_name = get_path('object_models','krylon_spray_in_meters.iv')


file_name_dict = dict()

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
