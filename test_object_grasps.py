import roslib; roslib.load_manifest( "trajectory_planner" )
import tf_conversions.posemath as pm
from numpy import *



rock_file_name = '/home/armuser/ros/RearmGraspit/cgdb/arm/darparock.iv'
flask_file_name = '/home/armuser/ros/RearmGraspit/models/objects/flask.iv'
drill_file_name = '/home/armuser/ros/RearmGraspit/cgdb/arm/darpadrill.iv'
large_shaving_gel_file_name = '/home/armuser/ros/RearmGraspit/cgdb/model_database/gillette_shaving_gel.iv'
coke_file_name = '/home/armuser/ros/RearmGraspit/cgdb/model_database/coke_can.iv'
odwalla_file_name = '/home/armuser/ros/RearmGraspit/cgdb/model_database/odwalla_bottle.iv'
all_file_name = '/home/armuser/ros/RearmGraspit/cgdb/model_database/all.iv'
hammer_file_name = '/home/armuser/ros/RearmGraspit/cgdb/arm/darpahammer.iv'
garnier_file_name =  '/home/armuser/ros/RearmGraspit/cgdb/model_database/garnier_shampoo_bottle.iv'
flashlight_file_name = '/home/armuser/ros/RearmGraspit/cgdb/arm/darpaflashlight.iv'
gillette_file_name = '/home/armuser/ros/RearmGraspit/cgdb/model_database/gillette_shaving_gel.iv'

drill_custom_file_name = '/home/armuser/ros/rearm/columbia_stacks/RearmGraspit/cgdb/semantic/drill_custom_in_meters.iv'
mug_custom_file_name = '/home/armuser/ros/rearm/columbia_stacks/RearmGraspit/cgdb/semantic/mug_custom_in_meters.iv'
darpaphonehandset_file_name = '/home/armuser/ros/rearm/columbia_stacks/RearmGraspit/cgdb/semantic/darpaphonehandset_1000_different_coordinate_system.iv'

box_file_name = '/home/armuser/ros/rearm/columbia_stacks/RearmGraspit/cgdb/ted/box_in_meters.iv'
snapple_file_name = '/home/armuser/ros/rearm/columbia_stacks/RearmGraspit/cgdb/ted/snapple_in_meters.iv'
library_cup_file_name = '/home/armuser/ros/rearm/columbia_stacks/RearmGraspit/cgdb/ted/library_cup_in_meters.iv'
krylon_spray_file_name = '/home/armuser/ros/rearm/columbia_stacks/RearmGraspit/cgdb/ted/krylon_spray_in_meters.iv'
milk_carton_file_name = '/home/armuser/ros/RearmGraspit/cgdb/model_database/milk_carton.ply'


file_name_dict = dict()
file_name_dict['18555'] = rock_file_name
file_name_dict['18556'] = flask_file_name
file_name_dict['18557'] = hammer_file_name
file_name_dict['18558'] = drill_file_name
file_name_dict['18716'] = large_shaving_gel_file_name
file_name_dict['18722'] = coke_file_name
file_name_dict['18707'] = odwalla_file_name
file_name_dict['18650'] = all_file_name

file_name_dict['garnier_shampoo_bottle'] = garnier_file_name
file_name_dict['/garnier_shampoo_bottle'] = garnier_file_name
file_name_dict['all'] = all_file_name
file_name_dict['/all'] = all_file_name
file_name_dict['odwalla_bottle'] = odwalla_file_name
file_name_dict['darpaflashlight'] = flashlight_file_name
file_name_dict['/darpaflashlight'] = flashlight_file_name
file_name_dict['gillette_shaving_gel'] = gillette_file_name
file_name_dict['/gillette_shaving_gel'] = gillette_file_name
file_name_dict['milk_carton'] = milk_carton_file_name
file_name_dict['/milk_carton'] = milk_carton_file_name
file_name_dict['flask'] = flask_file_name

file_name_dict['drill_custom'] = drill_custom_file_name
file_name_dict['mug_custom'] = mug_custom_file_name
file_name_dict['darpaphonehandset'] = darpaphonehandset_file_name
file_name_dict['box'] = box_file_name
file_name_dict['snapple'] = snapple_file_name
file_name_dict['darparock'] = rock_file_name
file_name_dict['krylon_spray'] = krylon_spray_file_name
file_name_dict['library_cup'] = library_cup_file_name
def grasp_str_to_grasp(grasp_str):
    grasp_str_list = grasp_str_list = grasp_str.split(';')
    grasp_id = int(grasp_str_list[0])
    def pg_str_to_array(array_str):
        return [float(a) for a in array_str.strip('"{').rstrip('}"').split(',')]

    def grasp_str_to_dof_list(grasp_str):
        grasp_list = pg_str_to_array(grasp_str)
        return (grasp_list[2], grasp_list[3], grasp_list[4], grasp_list[1])

    def grasp_str_to_tran(grasp_str):
        grasp_pose_list = pg_str_to_array(grasp_str)
        return pm.toMatrix(pm.fromTf(((grasp_pose_list[1]/1000.0, grasp_pose_list[2]/1000.0, grasp_pose_list[3]/1000.0),(grasp_pose_list[5], grasp_pose_list[6], grasp_pose_list[7], grasp_pose_list[4]))))
    
    
    
    pregrasp_dof = grasp_str_to_dof_list(grasp_str_list[3])
    grasp_dof = grasp_str_to_dof_list(grasp_str_list[4])
    grasp_tran =  grasp_str_to_tran(grasp_str_list[9])
    return (grasp_id, pregrasp_dof, grasp_dof, grasp_tran, file_name_dict[grasp_str_list[1]])


glass_rock_grasp_1 = grasp_str_to_grasp('793924;18555;12;"{5,1.24563,0.297563,0.297563,0.297563}";"{5,1.24563,0.790063,1.51069,1.23131}";1.32781e+130;-1;-1;"{0,2.2334,-39.3281,90.8804,-1.4097,4.43953,4.98491,-1.01533}";"{0,-0.749826,-39.0185,85.5087,-0.204376,0.643634,0.722702,-0.147201}";"{-6.68102,15.8097,0.120906,-32.6698,12.9468,-7.48714,48.8818,9.10376,-5.01302,53.3805,6.32813,8.67672,52.0725,7.88283,-6.87937}";FALSE;0.0377182;0.00409393;"";8')

glass_rock_grasp_2 = grasp_str_to_grasp('793900;18555;12;"{5,0,0.655507,0.655507,0.655507}";"{5,0,0.938632,0.953007,0.885507}";9.68349e-256;-1;-1;"{0,97.7208,4.88699,2.16642,-4.63069,-0.439303,4.38637,0.0545565}";"{0,85.2454,3.78583,2.78387,-0.724257,-0.0687086,0.686045,0.00853284}";"{-40.7456,9.3064,-0.591278,43.4741,9.84429,5.78375,49.5788,8.78068,-6.41793,-45.823,13.4326,7.86445,52.732,8.03781,-2.51042}";FALSE;0.0302783;0.0078579;"";8')


shaving_gel_large_grasp_1 = grasp_str_to_grasp('801537;18716;8;"{5,0.593586,0.720859,0.720859,0.720859}";"{5,0.593586,0.855859,0.904609,1.67086}";1.10254e+119;-1;-1;"{0,53.3978,-42.4468,99.4411,2.2132,-3.54294,2.19551,3.70341}";"{0,53.3978,-42.4468,99.4411,0.368937,-0.590604,0.365989,0.617353}";"{-26.023,12.9465,7.08476,23.1769,9.85638,8.78813,-23.4074,13.0151,-5.7,15.6414,8.52117,-10.1052,54.4715,6.87126,5.41234e-016}";FALSE;0.246112;1.06744;"";8')

shaving_gel_large_grasp_2 = grasp_str_to_grasp('794528;18716;8;"{5,0.870327,1.29618,1.29618,1.29618}";"{5,0.870327,1.53118,1.51368,1.68243}";2.47746e+119;-1;-1;"{0,16.081,29.8024,105.609,4.79734,2.31819,-4.12535,2.29648}";"{0,9.69169,20.7068,106.911,0.673874,0.325632,-0.579481,0.322583}";"{-41.568,-4.58335,-0.524635,-30.5225,12.6447,8.12099,30.0393,10.2423,8.1225,-41.6624,12.3733,-8.73118,-31.1891,13.2641,6.42568,31.6209,10.8456,-3.96729}";FALSE;0.241242;0.670789;"";8')

cola_grasp_1 = grasp_str_to_grasp('794702;18722;8;"{5,0.44156,1.30507,1.30507,1.30507}";"{5,0.44156,1.6107,1.49007,1.5757}";4.57372e+127;-1;-1;"{0,-45.3484,-29.9094,94.4091,-4.59913,4.38724,-3.47134,2.63479}";"{0,-33.5633,-25.1862,93.7232,-0.596775,0.569282,-0.450435,0.341886}";"{-7.78674,-3.59416,0.509729,-28.9815,9.79457,-9.6471,45.4055,8.39238,-9.18111,-33.2919,11.2917,9.44093,40.9329,9.41355,8.67672,32.9274,10.7326,5.01302}";FALSE;0.343722;0.900052;"";8')

cola_grasp_2 = grasp_str_to_grasp('801318;18722;8;"{5,0.869299,0.932375,0.932375,0.932375}";"{5,0.869299,1.60237,1.51362,0.974875}";3.93993e+127;-1;-1;"{0,43.7544,-36.8776,81.2363,-0.52903,-4.1431,1.99074,3.55332}";"{0,29.4148,-32.4399,77.4992,-0.0906822,-0.710178,0.341237,0.609082}";"{-30.7063,5.2607,-0.0500863,51.954,7.6491,8.11475,-16.5659,12.7207,5.99419,33.6634,10.9984,9.70578e-016,-36.9367,13.4231,-6.86115,34.7593,10.1729,-8.11475}";FALSE;0.344867;2.25364;"";8')

cola_grasp_3 = grasp_str_to_grasp('807040;18722;8;"{5,0.383024,1.29274,1.29274,1.29274}";"{5,0.383024,1.32305,1.45805,1.31305}";2.29135e-252;-1;-1;"{0,-54.6086,-11.2828,74.1451,3.56723,-2.47071,2.3174,-3.15246}";"{0,-54.6086,-11.2828,74.1451,0.610539,-0.422866,0.396627,-0.539549}";"{-36.8106,10.2751,-9.62584,50.079,7.82451,-8.87794,-29.4532,10.2402,9.78046,49.625,8.13164,8.67672,51.2788,8.7421,2.11526}";FALSE;0.349173;1.42728;"";8')


odwalla_grasp_1 = grasp_str_to_grasp('803545;18707;8;"{5,0.959163,1.21898,1.21898,1.21898}";"{5,0.959163,1.23148,1.36398,1.45398}";2.50813e-280;-1;-1;"{0,6.44615,-49.6562,90.659,-3.74851,3.03608,3.85814,4.02042}";"{0,6.44615,-49.6562,90.659,-0.508612,0.411946,0.523487,0.545505}";"{-19.5165,11.3968,8.97592,27.2831,9.51239,9.16709,-33.5092,12.1062,-8.68638,50.5098,8.26212,-7.93173,-24.4625,12.8768,7.09387,43.9233,10.1339,1.45477}";FALSE;0.306337;1.43949;"";8')

all_grasp_1 = grasp_str_to_grasp('802709;18650;8;"{5,0.928716,0.375428,0.375428,0.375428}";"{5,0.928716,0.620428,0.719803,0.560428}";3.15245e-260;-1;-1;"{0,-73.2368,-5.0819,117.075,4.80788,0.0729227,4.15831,0.836216}";"{0,-73.2368,-5.0819,117.075,0.749842,0.0113731,0.648536,0.130417}";"{-27.194,12.4909,8.13737,36.4837,9.76766,8.67672,36.1209,10.4934,-5.90334,-20.6111,12.7227,-6.86131,25.1218,9.94203,-8.67496}";FALSE;0.335564;1.42807;"";8')

all_grasp_2 = grasp_str_to_grasp('799608;18650;8;"{5,0.926077,0.384353,0.384353,0.384353}";"{5,0.926077,0.571853,0.455603,0.815603}";1.01094e+127;-1;-1;"{0,56.92,2.74012,142.265,4.03999,0.695816,-3.78857,-0.261352}";"{0,56.92,2.74012,142.265,0.722959,0.124517,-0.677967,-0.0467692}";"{32.162,10.2232,8.11475,-34.2587,12.8389,-7.87131,41.0068,8.94771,-9.18089,32.6262,9.66864,8.95318}";FALSE;0.315525;0.967775;"";8')







rock_grasp_1 = array([[-0.50634733,  0.75471653, -0.41715147,  0.0109033 ],
       [ 0.72082984,  0.63595453,  0.27561962, -0.048523  ],
       [ 0.47330405, -0.16113597, -0.86603607,  0.104361  ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
rock_grasp_2_id = 793230
rock_grasp_2_pregrasp = [0.211099,0.211099,0.211099, 0.58293]
rock_grasp_2_grasp = [1.05204,1.06172, 0.407349, 0.58293]
rock_grasp_2_tran  = pm.toMatrix(pm.fromTf(((21.8359/1000,55.0487/1000,54.2233/1000) ,(0.758541,0.477035,-0.236015,0.375965))))
rock_grasp_2 = (rock_grasp_2_id, rock_grasp_2_pregrasp, rock_grasp_2_grasp, rock_grasp_2_tran, rock_file_name)

rock_grasp_3_id = 793745
rock_grasp_3_pregrasp = (0.255066,0.255066,0.255066,0.432038)
rock_grasp_3_grasp = (0.726316,0.829441,1.02319, 0.432038)
rock_grasp_3_tran = pm.toMatrix(pm.fromTf(((8.17366/1000,-38.7327/1000,83.2182/1000),(-0.677905,0.699159,0.127056,0.188355))))
rock_grasp_3 = (rock_grasp_3_id, rock_grasp_3_pregrasp, rock_grasp_3_grasp, rock_grasp_3_tran, rock_file_name)


rock_grasp_4_id = 793632
rock_grasp_4_pregrasp =(0.678554,0.678554,0.678554,  0.419292)
rock_grasp_4_grasp = (0.841679,0.739179,0.684804, 0.419292)
rock_grasp_4_tran =  pm.toMatrix(pm.fromTf(((-51.5346/1000,-67.354/1000,5.5002/1000),(-0.547179,-0.457344,-0.671121,0.202555))))

rock_grasp_4 = (rock_grasp_4_id, rock_grasp_4_pregrasp, rock_grasp_4_grasp, rock_grasp_4_tran, rock_file_name)


hammer_grasp_1_id = 793247
hammer_grasp_1_pregrasp = (0.518563,0.518563,0.518563, 0.144425)
hammer_grasp_1_grasp = (0.769813,1.36356,1.82356,0.144425)
hammer_grasp_1_tran = pm.toMatrix(pm.fromTf(((-14.3017/1000,-27.5904/1000,89.7165/1000),(0.702297,-0.312021, -0.0982002, -0.63228))))

hammer_grasp_1 = (hammer_grasp_1_id, hammer_grasp_1_pregrasp, hammer_grasp_1_grasp, hammer_grasp_1_tran, hammer_file_name)

hammer_grasp_2_id = 793197
hammer_grasp_2_pregrasp = (0.77673,0.77673,0.77673, 0.358101)
hammer_grasp_2_grasp = (1.03673,1.06298,1.036,0.358101)
hammer_grasp_2_tran = pm.toMatrix(pm.fromTf(((-15.6494/1000,48.6476/1000,54.7623/1000),(-0.339974,-0.595159,0.610536,-0.396))))
hammer_grasp_2 = (hammer_grasp_2_id, hammer_grasp_2_pregrasp, hammer_grasp_2_grasp, hammer_grasp_2_tran, hammer_file_name)
