from franka_step import *

if __name__ == "__main__":

    # p = run_cmd("roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true")
    # time.sleep(5)
    
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    tutorial = MoveGroupPythonInterfaceTutorial()
    
    dict_func = {'get': get_pose,
                 'cam': up_cam,                             ## args: tutorial, exp_save_idx
                 'ready': go_to_ready,
                 'reach': reach_line,
                 'line2': line_second,                      ## args = tutorial, num_step
                 'line': go_through_line_scp,               ## args = tutorial, exp_save_idx
                 'line0': go_through_line_primitive,        ## args = tutorial, cam_on, exp_save_idx
                 'needle': go_to_needle,                    ## args = tutorial, needle_type
                 'thread': thread_needle,
                 'adjust': adjust_thread,                   ## args = tutorial, cam_on, exp_save_idx
                 'rl': adjust_thread_rl,                    ## args = tutorial, exp_save_idx
                 'mask': adjust_thread_rl_mask,             ## args = tutorial, exp_save_idx
                 'pose': pose,                              ## args = tutorial, pose_x, pose_y, pose_z
                 'pose_r':pose_relative,                    ## args = tutorial, pose_r_x, pose_r_y, pose_r_z
                 'p_r': pose_rot,                           ## args = tutorial, pose_r_x, pose_r_y, pose_r_z, angle
                 'joint': joint,                            ## args = tutorial, joints(7)
                 'joint_r': joint_relative,                 ## args = tutorial, joints_r(7)
                 'rot': rot_relative,                       ## args = tutorial, rot_r_x, rot_r_y, rot_r_z
                 'img_scp': save_img_scp,                   ## args = tutorial, exp_save_idx
                 'img': save_img,                           ## args = tutorial, exp_save_idx
                 }
    
    marker_pos = []
    pose_l = []

    while True:
        func = input("============Input the function name: ")
        args = input("============Input the args: ")
        args = [tutorial] + args.split(' ')
        
        if func == 'done':
            print("================Done!================")
            break
        
        
        if func not in dict_func.keys():
            print("{} not in the function list".format(func))
            continue
        
        
        if func == 'cam':
            marker_pos.append(dict_func[func](args)[-1])
            continue
        
        if func == 'pose':
            pose_l.append(dict_func[func](args))
            continue
        
        if func == 'needle':
            args = args + marker_pos[-1].tolist()
        
        dict_func[func](args)