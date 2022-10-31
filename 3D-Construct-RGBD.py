import json
import re
import time
import datetime
import sys
import os.path
import open3d as o3d
sys.path.append(".")
sys.path.append("files")
import make_fragments, register_fragments, refine_registration, integrate_scene, file, slac, slac_integrate
from initialize_config import initialize_config

class bcolors:
    OK = '\033[92m' #GREEN
    WARNING = '\033[93m' #YELLOW
    FAIL = '\033[91m' #RED
    RESET = '\033[0m' #RESET COLOR
    
if __name__ == '__main__':
    
    # validate and open configuration file
    if "config-custom.json" != None:
        with open('config-custom.json',) as json_file:
        config = json.load(json_file)
        initialize_config(config)
        file.check_folder_structure(config["path_dataset"])
    assert config != None

    print("====================================")
    print("Configuration")
    print("====================================")
    
    # display items in every process
    for key, val in config.items():
        print("%40s : %s" % (key, str(val)))

    # initialize time
    times = [0, 0, 0, 0, 0, 0, 0]
    print(f"{bcolors.OK}====================================\nMAKE FRAGMENTS\n===================================={bcolors.RESET}")
    start_time = time.time(); make_fragments.run(config); times[0] = time.time() - start_time
    print(f"{bcolors.OK}====================================\nREGISTER FRAGMENTS\n===================================={bcolors.RESET}")
    start_time = time.time(); register_fragments.run(config); times[1] = time.time() - start_time
    print(f"{bcolors.OK}====================================\nREFINE REGISTRATION\n===================================={bcolors.RESET}")
    start_time = time.time(); refine_registration.run(config); times[2] = time.time() - start_time
    print(f"{bcolors.OK}====================================\nINTEGRATE SCENE\n===================================={bcolors.RESET}")
    start_time = time.time(); integrate_scene.run(config); times[3] = time.time() - start_time
    
    # create triangle mesh from .ply file created
    mesh = o3d.io.read_triangle_mesh(os.path.join(config['path_dataset'], 'scene/', 'integrated.ply'))

    print("====================================")
    print("Elapsed time (in h:m:s)")
    print("====================================")
    print("- Making fragments %s" %
    datetime.timedelta(seconds=times[0]))
    print("- Register fragments %s" %
    datetime.timedelta(seconds=times[1]))
    print("- Refine registration %s" %
    datetime.timedelta(seconds=times[2]))
    print("- Integrate frames %s" %
    datetime.timedelta(seconds=times[3]))
    print("- Total %s" % datetime.timedelta(seconds=sum(times)))

    # create window for geometries visualization
    o3d.visualization.draw_geometries([mesh])

    # cleanup memory
    sys.stdout.flush()