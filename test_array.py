import json
import re
import time
import datetime
import sys
import os.path
import open3d as o3d
from os.path import exists, join
import numpy as np
sys.path.append(".")
sys.path.append("files")
import make_fragments, register_fragments, refine_registration, integrate_scene, file, slac, slac_integrate
from initialize_config import initialize_config

# dataset = [1,3,5,7]

# for folder_no in dataset:
#     print("folder no.:", folder_no)
#     if "config-custom.json" != None:
#         with open('config-default-stack.json',) as json_file:
#             config = json.load(json_file)
#             initialize_config(config)
#             config["path_dataset"] = join("stack", str(folder_no))
#             config["path_intrinsic"] = join(config["path_dataset"], "camera_intrinsic.json")
#             print("path_root        :", config["path_dataset"])
#             print("path_intrinsic   :", config["path_intrinsic"])
#             file.check_folder_structure(config["path_dataset"])
#     assert config != None
    

arr = [num for num in range(1, 25, 2)]
print(arr)