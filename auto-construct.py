import json
import re
import time
import datetime
import sys
import open3d as o3d
from contextlib import contextmanager
import os
from os.path import join
sys.path.append(".")
sys.path.append("files")
import make_fragments, register_fragments, refine_registration, integrate_scene, file, slac, slac_integrate
from initialize_config import initialize_config

class bcolors:
    OK = '\033[92m' #GREEN
    WARNING = '\033[93m' #YELLOW
    FAIL = '\033[91m' #RED
    RESET = '\033[0m' #RESET COLOR
    
@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout

# def get_file_list(path, extension=None):

#     def sorted_alphanum(file_list_ordered):
#         convert = lambda text: int(text) if text.isdigit() else text
#         alphanum_key = lambda key: [
#             convert(c) for c in re.split('([0-9]+)', key)
#         ]
#         return sorted(file_list_ordered, key=alphanum_key)

#     if extension is None:
#         file_list = [
#             path + f
#             for f in os.listdir(path)
#             if os.path.isfile(os.path.join(path, f))
#         ]
#     else:
#         file_list = [
#             path + f
#             for f in os.listdir(path)
#             if os.path.isfile(os.path.join(path, f)) and
#             os.path.splitext(f)[1] == extension
#         ]
#     file_list = sorted_alphanum(file_list)
#     return file_list


def write_reconstruction_log(folder_no, json_filename, make, register, refine, integrate, total):
    with open(join("stack", str(folder_no),"log.txt"), 'a') as f:
        f.write("\n\nConfiguration       %s" % json_filename)
        f.write("\nMaking fragments    %s" % make)
        f.write("\nRegister fragments  %s" % register)
        f.write("\nRefine registration %s" % refine)
        f.write("\nIntegrate frames    %s" % integrate)
        f.write("\nTotal               %s" % total)
    f.close()

def write_reconstruction_log_total(json_filename, make, register, refine, integrate, total):
    with open(join("stack", "log_sum.txt"), 'a') as f:
        f.write("\n\nConfiguration       %s" % json_filename)
        f.write("\nMaking fragments    %s" % make)
        f.write("\nRegister fragments  %s" % register)
        f.write("\nRefine registration %s" % refine)
        f.write("\nIntegrate frames    %s" % integrate)
        f.write("\nTotal               %s" % total)
    f.close()

if __name__ == '__main__':
    
    # read folder number in [stack] to construct
    # arr = [num for num in range(1, 18*2+1, 2)] # insert odd numbers from 1 to 36 (end of folder) with step size of 2 (for default config)
    # arr = [num for num in range(7, 13, 2)] # insert even numbers from 2 to 36 (end of folder) with step size of 2 (for custom config)
    stack = [8] # << input array of [folder_no] HERE, lookup: config-default-stack.json
    json_filename = 'config-custom-stack.json' # << change configuration file HERE
    sum_times = [0, 0, 0, 0, 0, 0, 0] # initialize total reconstruction time
    
    for folder_no in stack:
        if json_filename != None:
            with open(json_filename) as json_file:
                config = json.load(json_file)
                initialize_config(config)
                config["path_dataset"] = join("stack", str(folder_no))
                config["path_intrinsic"] = join(config["path_dataset"], "camera_intrinsic.json")
                print("path_root        :", config["path_dataset"])
                print("path_intrinsic   :", config["path_intrinsic"])
                file.check_folder_structure(config["path_dataset"])
        assert config != None

        # print("====================================")
        # print("Configuration -- folder no.", folder_no)
        # print("====================================")
        # for key, val in config.items():
        #     print("%40s : %s" % (key, str(val)))
        
        times = [0, 0, 0, 0, 0, 0, 0] # initialize reconstruction time for each folder
        
        print(f"{bcolors.OK}====================================\nMAKE FRAGMENTS{bcolors.WARNING}", "\t\tfolder no :", folder_no, f"{bcolors.RESET}")
        start_time = time.time()
        with suppress_stdout():
            make_fragments.run(config)
        times[0] = time.time() - start_time
        
        print(f"{bcolors.OK}====================================\nREGISTER FRAGMENTS{bcolors.WARNING}", "\tfolder no :", folder_no, f"{bcolors.RESET}")
        start_time = time.time()
        with suppress_stdout():
            register_fragments.run(config)
        times[1] = time.time() - start_time
         
        print(f"{bcolors.OK}====================================\nREFINE REGISTRATION{bcolors.WARNING}", "\tfolder no :", folder_no, f"{bcolors.RESET}")
        start_time = time.time()
        with suppress_stdout():
            refine_registration.run(config)
        times[2] = time.time() - start_time
        
        print(f"{bcolors.OK}====================================\nINTEGRATE SCENE{bcolors.WARNING}", "\tfolder no :", folder_no, f"{bcolors.RESET}", "\n====================================")
        start_time = time.time()
        with suppress_stdout():
            integrate_scene.run(config)
        times[3] = time.time() - start_time
        
        # start_time = time.time(); slac.run(config); times[4] = time.time() - start_time
        # start_time = time.time(); slac_integrate.run(config); times[5] = time.time() - start_time
        
        
        # path = config['path_dataset']
        
        # color_image_path = get_file_list(os.path.join(path, "color/"), extension=".jpg")
        # depth_image_path = get_file_list(os.path.join(path, "depth/"), extension=".png")
        
        # mesh = o3d.io.read_triangle_mesh(os.path.join(config['path_dataset'], 'scene/', 'integrated.ply'))
        
        # camera_trajectory = o3d.io.read_pinhole_camera_trajectory(os.path.join(config['path_dataset'], 'scene/', 'trajectory.log'))
            
        # rgbd_images = []
        # for i in range(len(depth_image_path)):
        #     depth = o3d.io.read_image(os.path.join(depth_image_path[i]))
        #     color = o3d.io.read_image(os.path.join(color_image_path[i]))
        #     rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False)
        #     rgbd_images.append(rgbd_image)    
        
        # start_time = time.time()
        # maximum_iteration = 10
        # with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        #     mesh_optimized = o3d.pipelines.color_map.run_rigid_optimizer(mesh, rgbd_images, camera_trajectory, o3d.pipelines.color_map.RigidOptimizerOption(maximum_iteration=maximum_iteration, maximum_allowable_depth=config["max_depth"]))
        # o3d.io.write_triangle_mesh(os.path.join(path, config["folder_scene"], "color_map_after_optimization.ply"), mesh_optimized)
        # times[6] = time.time() - start_time
        
        # mesh_optimized = o3d.io.read_triangle_mesh(os.path.join(config['path_dataset'], 'scene/', 'color_map_after_optimization.ply'))
        
        # print("====================================")
        print(f"{bcolors.OK}Elapsed time (in h:m:s){bcolors.WARNING}", "folder no :", folder_no, f"{bcolors.RESET}")
        print("====================================")
        print("- Making fragments    %s" % datetime.timedelta(seconds=times[0]))
        print("- Register fragments  %s" % datetime.timedelta(seconds=times[1]))
        print("- Refine registration %s" % datetime.timedelta(seconds=times[2]))
        print("- Integrate frames    %s" % datetime.timedelta(seconds=times[3]))
        # print("- SLAC                %s" % datetime.timedelta(seconds=times[4]))
        # print("- SLAC Integrate      %s" % datetime.timedelta(seconds=times[5]))
        # print("- Optimizing          %s" % datetime.timedelta(seconds=times[6]))
        print("- Total               %s" % datetime.timedelta(seconds=sum(times)))
        print("====================================")
        
        sum_times[0] = sum_times[0] + times[0]
        sum_times[1] = sum_times[1] + times[1]
        sum_times[2] = sum_times[2] + times[2]
        sum_times[3] = sum_times[3] + times[3]
        
        write_reconstruction_log(folder_no, json_filename,
            datetime.timedelta(seconds=times[0]),
            datetime.timedelta(seconds=times[1]),
            datetime.timedelta(seconds=times[2]),
            datetime.timedelta(seconds=times[3]),
            datetime.timedelta(seconds=sum(times)),
            )
        
        # o3d.visualization.draw_geometries([mesh])
        # o3d.visualization.draw_geometries([mesh_optimized])
        
    print(f"{bcolors.OK}Total Elapsed time (in h:m:s){bcolors.WARNING}", "folder no :", folder_no, f"{bcolors.RESET}")
    print("====================================")
    print("- Making fragments    %s" % datetime.timedelta(seconds=sum_times[0]))
    print("- Register fragments  %s" % datetime.timedelta(seconds=sum_times[1]))
    print("- Refine registration %s" % datetime.timedelta(seconds=sum_times[2]))
    print("- Integrate frames    %s" % datetime.timedelta(seconds=sum_times[3]))
    # print("- SLAC                %s" % datetime.timedelta(seconds=sum_times[4]))
    # print("- SLAC Integrate      %s" % datetime.timedelta(seconds=sum_times[5]))
    # print("- Optimizing          %s" % datetime.timedelta(seconds=sum_times[6]))
    print("- Total               %s" % datetime.timedelta(seconds=sum(sum_times)))
    print("====================================")
    
    write_reconstruction_log_total(json_filename,
            datetime.timedelta(seconds=sum_times[0]),
            datetime.timedelta(seconds=sum_times[1]),
            datetime.timedelta(seconds=sum_times[2]),
            datetime.timedelta(seconds=sum_times[3]),
            datetime.timedelta(seconds=sum(sum_times)))
    sys.stdout.flush()