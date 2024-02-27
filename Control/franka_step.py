from franka_tutorial import *
from RL_demo.gel_ppo import *

scp_root_dir = "/home/jeffsonyu/sb3_test/preprocess/line_exp_log"
exp_root_dir = "/home/franka/zhenjun/line_exp_log"
    
    
def go_to_ready(args):
    ### Input: ready
    tutorial = args[0]
    
    
    # input("============Press 'Enter' to go to the ready state...")
    tutorial.go_to_ready_state()
    
def up_cam(args):
    ### Input: cam
    ### args: tutorial, exp_save_idx
    tutorial = args[0]
    exp_save_idx = int(args[1])


    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    save_txt = os.path.join(save_dir, "marker_pos.txt")
    
    
    input(
        "============Press 'Enter' to ready and go up to use cam..."
    )
    # tutorial.go_to_ready_state()
    # tutorial.go_to_pose_goal_relative([0.2, 0, 0.2])
    tutorial.go_to_joint_state([0, -52, 8, -96, 4, 83, 53], True)
    time.sleep(1)

    marker_pos, marker_quat = get_marker_from_api()
    r_marker = R.from_quat(marker_quat)
    marker_mat = r_marker.as_matrix()
    marker_euler = r_marker.as_euler('xyz', degrees=True)
    print(marker_euler)

    franka_goal_pos = cam_to_base(tutorial, marker_pos)
    np.savetxt(save_txt, np.array(franka_goal_pos))
    print("Marker position: ", franka_goal_pos.tolist())

    
    return marker_mat, marker_euler, franka_goal_pos


def reach_line(args):
    ### Input: reach
    ### args: tutorial, line_type
    tutorial = args[0]
    
    input(
        "============Press `Enter` to execute a movement using a joint state goal..."
    )

    tutorial.go_to_joint_state([69, -42, -3, -95, -13, 143, -127], True)
    tutorial.go_to_joint_state([70, -9, -10, -52, -6, 113, -135], True)

    

def go_through_line_scp(args):
    ## Input: line
    ## Attention! This func is tricky
    ## args = tutorial, exp_save_idx
    tutorial = args[0]
    exp_save_idx = int(args[1])
    
    
    input(
        "============Press `Enter` to go through the line with scp..."
    )


    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)

    if not os.path.exists(save_dir):
        os.mkdir(save_dir)
    
    run_cmd("scp -r {} A2000:{}".format(save_dir, scp_root_dir))


    img_idx = 0
    try:
        while True:
        # for i in range(10):
            # _, frame = cam_gel.read()
            frame = get_gel_from_api()
            img_path_bmp = os.path.join(save_dir, "{:04d}.bmp".format(img_idx))
            cv2.imwrite(img_path_bmp, frame)
            

            run_cmd("scp {} A2000:{}".format(img_path_bmp, os.path.join(scp_root_dir, exp_save_dir)))


            while True:
                print("Waiting for img_mask_0_bmp")
                img_mask_0_bmp = os.path.join(save_dir, "{:04d}_mask_0.bmp".format(img_idx))
                img_mask_1_bmp = os.path.join(save_dir, "{:04d}_mask_1.bmp".format(img_idx))
                if os.path.exists(img_mask_0_bmp):
                    stop_flag = 0
                    break
                if os.path.exists(img_mask_1_bmp):
                    stop_flag = 1
                    break
            
            if not stop_flag:
                print("===================Go down 1cm...")
                tutorial.go_to_pose_goal_relative([0, 0, 0.01])
            
            if stop_flag:
                print("============Total step: {}".format(img_idx))
                print("===================STOP!======================")
                break

            input("======Continue?")
            img_idx += 1
    except KeyboardInterrupt:
        print("Interupt!!!")

def go_through_line_primitive(args):
    ## Input: line0
    ## For testing scp
    ## args = tutorial, cam_on, exp_save_idx
    
    tutorial = args[0]
    cam_on = int(args[1])
    exp_save_idx = int(args[2])
    
    
    input(
        "============Press `Enter` to go through the line..."
    )
    if cam_on:
    
        exp_save_dir = "test_{:04d}".format(exp_save_idx)
        save_dir = os.path.join(exp_root_dir, exp_save_dir)

        if not os.path.exists(save_dir):
            os.mkdir(save_dir)

    img_idx = 0
    while True:
        if cam_on:
            frame = get_line_gel_from_api()
            img_path_bmp = os.path.join(save_dir, "line_{:04d}.bmp".format(img_idx))
            cv2.imwrite(img_path_bmp, frame)


        stop_flag = input("======Continue?")

        if stop_flag == '0':
            print("============Total step: {}".format(img_idx))
            print("===================STOP!======================")
            break
        else:
            print("===================Go down 0.5cm...")
            tutorial.go_to_pose_goal_relative([0, 0, -0.005])
            
        img_idx += 1


def line_second(args):
    ## Input: line2
    ## args = tutorial, line_type, num_step
    tutorial = args[0]
    exp_save_idx = int(args[1])

    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)
    
    input(
        "============Press `Enter` to go through the line the second time..."
    )
    tutorial.go_to_pose_goal_relative([-0.05, 0, 0])
    tutorial.go_to_joint_state([69, -42, -3, -95, -13, 143, -127], True)

    tutorial.go_to_joint_state([70, -9, -10, -52, -6, 113, -135], True)

    input("===============Continue?")

    while True:

        stop_flag = input("======Continue?")

        if stop_flag == '0':
            frame = get_line_gel_from_api()
            img_path_bmp = os.path.join(save_dir, "line_stop.bmp")
            cv2.imwrite(img_path_bmp, frame)
            print("===================STOP!======================")
            break
        else:
            print("===================Go down 0.5cm...")
            tutorial.go_to_pose_goal_relative([0, 0, -0.005])
    
    

def go_to_needle(args):
    ## Input: needle
    ## args = tutorial, needle_type, exp_save_idx, franka_goal_pos_x, franka_goal_pos_y, franka_goal_pos_z
    tutorial = args[0]
    needle_type = int(args[1])
    exp_save_idx = int(args[2])


    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)
    save_txt = os.path.join(save_dir, "marker_pos.txt")

    
    franka_goal_pos = np.loadtxt(save_txt)
    
    input(
        "============Press `Enter` to reach the needle..."
    )
    
    tutorial.go_to_ready_state()
    tutorial.go_to_rot_relative([0, -23, 0])
    tutorial.go_to_pose_goal_relative([0.05, 0.17, -0.07])
    if needle_type == 1:
        tutorial.go_to_pose_goal(franka_goal_pos + np.array([-0.1, 0, -0.16]))
    
    print("============Go to needle complete...")


def thread_needle(args):
    ## Input: thread
    tutorial = args[0]
    
    input(
        "============Press `Enter` to thread the needle..."
    )
    
    tutorial.go_to_pose_goal_relative([0.02, 0, 0])
    


def adjust_thread(args):
    ## Input: adjust
    ## args = tutorial, cam_on, exp_save_idx

    tutorial = args[0]
    cam_on = int(args[1])
    exp_save_idx = int(args[2])

    tutorial.go_to_pose_goal_relative([-0.03, 0, 0])
    save_img([tutorial, exp_save_idx])
    tutorial.go_to_pose_goal_relative([0.03, 0, 0])
    
    stop_flag = '1'
    img_idx = 0

    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)

    img_needle_file = os.path.join(save_dir, "needle_result.bmp")
    img_needle = cv2.imread(img_needle_file, cv2.IMREAD_UNCHANGED)
    
    if cam_on == 1:
        frame = get_gel_from_api()
        img_path_bmp = os.path.join(save_dir, "ad_{:04d}.bmp".format(img_idx))
        cv2.imwrite(img_path_bmp, frame)

        img_diff = cv2.absdiff(img_needle, frame)
        img_path_bmp = os.path.join(save_dir, "ad_mask_{:04d}.bmp".format(img_idx))
        cv2.imwrite(img_path_bmp, img_diff)

        img_idx += 1


    while True:
        
        if stop_flag == '0':
            
            print("===================SUCCESS!======================")
            break
        
        else:
            
            # Input: 0 0.1
            vec = input("============Input values...")
            x, y = float(vec.split(' ')[0]), float(vec.split(' ')[1])
            tutorial.go_to_pose_goal_relative([-0.03, 0, 0])
            tutorial.go_to_pose_goal_relative([0, x, y])
            tutorial.go_to_pose_goal_relative([0.0325, 0, 0])
            print("=================Move line on the needle...")

            if cam_on == 1:
                frame = get_gel_from_api()
                img_path_bmp = os.path.join(save_dir, "ad_{:04d}.bmp".format(img_idx))
                cv2.imwrite(img_path_bmp, frame)

                img_idx += 1

        
        stop_flag = input("============Continue?")


def adjust_thread_rl(args):
    ## Input: rl
    ## args = tutorial, exp_save_idx

    tutorial = args[0]
    exp_save_idx = int(args[1])

    tutorial.go_to_pose_goal_relative([-0.03, 0, 0])
    save_img([tutorial, exp_save_idx])
    tutorial.go_to_pose_goal_relative([0.03, 0, 0])
    
    
    input(
        "============Press `Enter` to go through the line with scp..."
    )

    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)

    img_needle_file = os.path.join(save_dir, "needle_result.bmp")
    img_needle = cv2.imread(img_needle_file, cv2.IMREAD_UNCHANGED)

    x_idx = int(np.mean(list(set(np.where(img_needle == 255)[0].tolist()))))
    y_idx = int(np.mean(list(set(np.where(img_needle == 255)[1].tolist()))))
    print(x_idx, y_idx)

    

    if not os.path.exists(save_dir):
        os.mkdir(save_dir)
    
    run_cmd("scp -r {} A2000:{}".format(save_dir, scp_root_dir))

    
    img_idx = 0
    try:
        while True:

            frame = get_gel_from_api()
            frame = cv2.absdiff(img_needle, frame)

            img_path_bmp = os.path.join(save_dir, "rl_{:04d}.bmp".format(img_idx))
            cv2.imwrite(img_path_bmp, frame)

            run_cmd("scp {} A2000:{}".format(img_path_bmp, os.path.join(scp_root_dir, exp_save_dir)))

            stop_flag = 0
            while True:
                # print("Waiting for obs")
                img_poke_file = os.path.join(save_dir, "obs_{:04d}.bmp".format(img_idx))
                if os.path.exists(img_poke_file) and os.path.getsize(img_poke_file) == 5760054:
                    print("Received segmentation result!")
                    img_poke = cv2.imread(img_needle_file, cv2.IMREAD_UNCHANGED)
                    len_line = np.where(img_poke > 0)[0].shape[0]

                    x_idx_poke = int(np.mean(list(set(np.where(img_poke == 255)[0].tolist()))))
                    y_idx_poke = int(np.mean(list(set(np.where(img_poke == 255)[1].tolist()))))
                    print(x_idx_poke, y_idx_poke)

                    hole_ref_inv = img_needle / 2
                    img_hole_line_inv = img_poke / 2
                    
                    img_line_ref = hole_ref_inv + img_hole_line_inv
                    len_line_in_hole = np.where(img_line_ref > 200)[0].shape[0]

                    if len_line_in_hole > 0.5 * len_line:
                        stop_flag = 1

                    break
            
            if stop_flag == 1:
                print("Success!!!")
                break
                
            
            obs = np.array([x_idx - x_idx_poke, y_idx - y_idx_poke])
            step, info = model_ppo.predict(obs)

            # tutorial.go_to_pose_goal_relative([-0.02, 0, 0])
            # tutorial.go_to_pose_goal_relative([0, step[0]/100, step[1]/100])
            # tutorial.go_to_pose_goal_relative([0.02, 0, 0])
            print("Step conducted!!")



            flag = input("======Continue?")
            if flag == "0":
                print("Success!!!")
                break
            img_idx += 1

    except KeyboardInterrupt:
        print("Interupt!!!")

def adjust_thread_rl_mask(args):
    ## Input: mask
    ## args = tutorial, exp_save_idx

    tutorial = args[0]
    exp_save_idx = int(args[1])
    # cam_id = int(args[2])

    tutorial.go_to_pose_goal_relative([-0.03, 0, 0])
    save_img([tutorial, exp_save_idx])
    tutorial.go_to_pose_goal_relative([0.03, 0, 0])
    
    
    input(
        "============Press `Enter` to begin adjust with rl..."
    )
    # cam_kinova = cv2.VideoCapture(cam_id)
    # print("Gel Opened")


    # for _ in range(100): # Buffer
    #     _, frame = cam_kinova.read()



    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)

    img_needle_file = os.path.join(save_dir, "kinova_gel.bmp")
    img_needle = cv2.imread(img_needle_file, cv2.IMREAD_UNCHANGED)

    img_needle_result = os.path.join(save_dir, "needle_result.bmp")
    needle_result = cv2.imread(img_needle_result, cv2.IMREAD_UNCHANGED)

    x_idx = int(np.mean(list(set(np.where(needle_result == 255)[0].tolist()))))
    y_idx = int(np.mean(list(set(np.where(needle_result == 255)[1].tolist()))))
    print(x_idx, y_idx)

    

    if not os.path.exists(save_dir):
        os.mkdir(save_dir)

    
    img_idx = 0
    try:
        stop_flag = 0
        while True:
        # for i in range(10):
            # _, frame = cam_kinova.read()
            frame = get_gel_from_api()
            img_path_bmp = os.path.join(save_dir, "rl_origin_{:04d}.bmp".format(img_idx))
            cv2.imwrite(img_path_bmp, frame)

            frame = cv2.absdiff(img_needle, frame)
            img_path_bmp = os.path.join(save_dir, "rl_diff_{:04d}.png".format(img_idx))
            cv2.imwrite(img_path_bmp, frame)
            # _, mask = line_seg(img_path_bmp, "bump", 0.25, 0.25)

            # img_mask_3c = 255 * np.repeat(mask[...,np.newaxis], 3, 2).astype('uint8')

            # img_path_bmp = os.path.join(save_dir, "rl_bump_{:04d}.png".format(img_idx))
            # cv2.imwrite(img_path_bmp, img_mask_3c)

            # img_path_bmp = os.path.join(save_dir, "rl_mask_{:04d}.bmp".format(img_idx))
            # cv2.imwrite(img_path_bmp, frame)

            img_poke = np.zeros_like(frame)

            img_poke[np.where(frame > 50)[:2]] = 255

            img_path_bmp = os.path.join(save_dir, "rl_mask_{:04d}.bmp".format(img_idx))
            cv2.imwrite(img_path_bmp, img_poke)

            x_idx_poke = int(np.mean(list(set(np.where(img_poke == 255)[0].tolist()))))
            y_idx_poke = int(np.mean(list(set(np.where(img_poke == 255)[1].tolist()))))

            # len_line = np.where(img_mask_3c > 0)[0].shape[0]

            # x_idx_poke = int(np.mean(list(set(np.where(img_mask_3c == 255)[0].tolist()))))
            # y_idx_poke = int(np.mean(list(set(np.where(img_mask_3c == 255)[1].tolist()))))
            # print(x_idx_poke, y_idx_poke)


            # hole_ref_inv = img_needle / 2
            # img_hole_line_inv = img_mask_3c / 2
            
            # img_line_ref = hole_ref_inv + img_hole_line_inv
            # len_line_in_hole = np.where(img_line_ref > 200)[0].shape[0]

            
            x_diff = x_idx - x_idx_poke
            y_diff = y_idx - y_idx_poke
            obs = np.array([x_diff, y_diff])
            step, info = model_ppo.predict(obs)
            print("RL:", step)

            step = np.array([-x_diff, -y_diff]) / (200 * 1000)
            print(step)

            input("======Press Enter to conduct step...")
            tutorial.go_to_pose_goal_relative([-0.02, 0, 0])
            tutorial.go_to_pose_goal_relative([0, step[0], step[1]])
            tutorial.go_to_pose_goal_relative([0, step[0], 0])
            tutorial.go_to_pose_goal_relative([0.02, 0, 0])
            print("Step conducted!!")

            flag = input("======Continue?")
            if flag == "0":
                print("Success!!!")
                break
            img_idx += 1

    except KeyboardInterrupt:
        print("Interupt!!!")
            

def pose(args):
    ## Input: pose
    ## args = tutorial, pose_x, pose_y, pose_z
    tutorial = args[0]
    pose_x = float(args[1])
    pose_y = float(args[2])
    pose_z = float(args[3])
    pose = np.array([pose_x, pose_y, pose_z])
    
    
    input(
        "============ Press `Enter` to go to relative pose..."
    )
    
    tutorial.go_to_pose_goal(pose)


def pose_relative(args):
    ## Input: pose_r
    ## args = tutorial, pose_r_x, pose_r_y, pose_r_z
    tutorial = args[0]
    pose_r_x = float(args[1])
    pose_r_y = float(args[2])
    pose_r_z = float(args[3])
    pose_r = np.array([pose_r_x, pose_r_y, pose_r_z])
    
    
    input(
        "============ Press `Enter` to go to relative pose..."
    )
    
    tutorial.go_to_pose_goal_relative(pose_r)

def pose_rot(args):
    ## Input: pose_rot
    ## args = tutorial, pose_r_x, pose_r_y, pose_r_z, angle
    tutorial = args[0]
    pose_r_x = float(args[1])
    pose_r_y = float(args[2])
    pose_r_z = float(args[3])
    angle = float(args[4])
    pose_r = np.array([pose_r_x, pose_r_y, pose_r_z])
    
    
    input(
        "============ Press `Enter` to go to relative pose..."
    )
    
    tutorial.go_to_pose_goal_relative(pose_r)


def joint(args):
    ## Input: joint
    ## args = tutorial, joints(7)
    tutorial = args[0]
    joint_goal = [float(value) for value in args[1:]]

    
    
    input(
        "============ Press `Enter` to go to relative pose..."
    )
    
    tutorial.go_to_joint_state(joint_goal)


def joint_relative(args):
    ## Input: joint_r
    ## args = tutorial, joints_relative(7)
    tutorial = args[0]
    joint_goal = [float(value) for value in args[1:]]

    
    
    input(
        "============ Press `Enter` to go to relative pose..."
    )
    
    tutorial.go_to_joint_state_relative(joint_goal)

def rot_relative(args):
    ## Input: pose
    ## args = tutorial, pose_r_x, pose_r_y, pose_r_z
    tutorial = args[0]
    ror_r_x = float(args[1])
    ror_r_y = float(args[2])
    ror_r_z = float(args[3])
    rot_r = np.array([ror_r_x, ror_r_y, ror_r_z])
    
    
    input(
        "============Press `Enter` to go to relative pose..."
    )
    
    tutorial.go_to_rot_relative(rot_r)

def get_pose(args):
    ## Input: get
    tutorial = args[0]
    
    input(
        "============Press `Enter` to get pose..."
    )
    
    pose, rot = tutorial.get_pose()
    
    return pose, rot

def save_img_scp(args):
    ## Input: img_scp
    ## args = tutorial, cam_id

    tutorial = args[0]
    exp_save_idx = int(args[1])


    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)

    if not os.path.exists(save_dir):
        os.mkdir(save_dir)

    run_cmd("scp -r {} A2000:{}".format(save_dir, scp_root_dir))

    frame = get_gel_from_api()

    img_path_bmp = os.path.join(save_dir, "kinova_gel.bmp")
    cv2.imwrite(img_path_bmp, frame)

    scp_dir = os.path.join(scp_root_dir, exp_save_dir)
    run_cmd("scp {} A2000:{}".format(img_path_bmp, scp_dir))

    try:
        while True:
            print("Waiting for segment results for needle...")
            img_needle_result = os.path.join(save_dir, "needle_result.bmp")
            if os.path.exists(img_needle_result):
                print("Needle segmentation result received!!!")
                time.sleep(3)
                break
    except KeyboardInterrupt:
        print("Interupt!!!")



def save_img(args):
    ## Input: img
    ## args = tutorial, exp_save_idx

    tutorial = args[0]
    exp_save_idx = int(args[1])


    exp_save_dir = "test_{:04d}".format(exp_save_idx)
    save_dir = os.path.join(exp_root_dir, exp_save_dir)

    if not os.path.exists(save_dir):
        os.mkdir(save_dir)

    frame = get_gel_from_api()

    img_path_bmp = os.path.join(save_dir, "kinova_gel.bmp")
    img_mask_path_bmp = os.path.join(save_dir, "needle_result.bmp")
    cv2.imwrite(img_path_bmp, frame)

    annotated_frame_with_mask, img_mask_1c = line_seg(img_path_bmp, 'big hole', 0.3, 0.25)
    img_mask_3c = 255 * np.repeat(img_mask_1c[...,np.newaxis], 3, 2).astype('uint8')

    cv2.imwrite(img_mask_path_bmp, img_mask_3c)

