import pyrealsense2 as rs
import numpy as np
import cv2
from os import listdir, mkdir, remove, path, makedirs
from os.path import exists, join
from math import tan, radians, sqrt
import shutil
import json
import time
from enum import IntEnum

class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5

class bcolors:
    OK = '\033[92m' #GREEN
    WARNING = '\033[93m' #YELLOW
    FAIL = '\033[91m' #RED
    RESET = '\033[0m' #RESET COLOR

def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                    intrinsics.ppy, 1
                ]
            },
            outfile,
            indent=4)

def make_clean_folder(path_folder):
    if not exists(path_folder):
        mkdir(path_folder)
    else:
        user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            mkdir(path_folder)
        else:
            exit()

def guidelines(input_img):
    
    # put label for center_depth value
    depth_label_position = (int(width/2+2), int(height/2-5))     
    cv2.putText(input_img, depth_label, depth_label_position, cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 2)
    
    # put label for acceleromater value (x, y, z)
    accel_label = str(accel)
    textsize = cv2.getTextSize(accel_label, cv2.FONT_HERSHEY_PLAIN, 1, 2)[0]
    # get coords based on boundary
    textX = int((input_img.shape[1] - textsize[0])) // 2
    textY = int((input_img.shape[0] + textsize[1])) // 2 + 10
    # add accel_label text centered on image
    cv2.putText(input_img, accel_label, (textX, textY ), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 2)
    
    # center vertical line (green)
    cv2.line(input_img, (int(width/2), int(0)), (int(width/2), int(height)), (0, 255, 0), 1)
    # left vertical line (white)
    cv2.line(input_img, (int(width/2-height/4), int(0)), (int(width/2-height/4), int(height)), (255, 255, 255), 1)
    # right vertical line (white)
    cv2.line(input_img, (int(width/2+height/4), int(0)), (int(width/2+height/4), int(height)), (255, 255, 255), 1)
    
    # circle (white)
    cv2.circle(input_img, (int(width/2), int(height/2)), int(height/4), (255, 255, 255), 1)
    # cemter dot (red)
    cv2.circle(input_img, (int(width/2), int(height/2)), 1, (0, 0, 255), 5)
    
    # hotizontal line (green)
    cv2.line(input_img, (int(0), int(height/2)), (int(width), int(height/2)), (0, 255, 0), 1)
    
    # outer rectangle (red)
    cv2.rectangle(input_img, (int(width/8), int(height/(8/7))), (int(width/(8/7)), int(height/8)), (0, 0, 255), 1)
    # inner rectangle (green)            
    cv2.rectangle(input_img, (int(width/4), int(height/(4/3))), (int(width/(4/3)), int(height/4)), (0, 255, 0), 1)

def accel_data(accel):
    return np.asarray([round(accel.x, 3), round(accel.y, 3), round(accel.z, 3)])

class writeScanLog:
    def default_log(folder_no, json_filename, rotation):
        setup_log = ["Preset\t\t: " + str(json_filename), "rotation\t: " + str(rotation), "distance\t: " + str(0), "radius\t\t: " + str(0), "disparity\t: " + str(0)]
        with open(join("stack", str(folder_no),"log.txt"), 'w') as f:
            f.write('\n'.join(setup_log))
        f.close()
            
    def custom_log(folder_no, json_filename, rotation, input_distance, radius, disparity):
        setup_log = ["Preset\t\t: " + str(json_filename), "rotation\t: " + str(rotation), "distance\t: " + str(input_distance), "radius\t\t: " + str(radius), "disparity\t: " + str(disparity)]
        with open(join("stack", str(folder_no),"log.txt"), 'w') as f:
            f.write('\n'.join(setup_log))
        f.close()

if __name__ == "__main__":
    
    #create folder
    folder_no = 11 # << put folder number HERE
    path_root = join("stack", str(folder_no))
    path_color = join(path_root, "color")
    path_depth = join(path_root, "depth")
    
    make_clean_folder(path_root)
    make_clean_folder(path_color)
    make_clean_folder(path_depth)

    # create pipeline
    pipeline = rs.pipeline()
    
    # make realsense configuration
    config = rs.config()
    
    # import custom preset
    json_filename = "MediumDensityPreset-custom.json" # << change preset file HERE
    jsonObj = json.load(open(json_filename))
    json_string= str(jsonObj).replace("'", '\"')
    
    # resolution and fps
    width = int(jsonObj['viewer']['stream-width'])
    height = int(jsonObj['viewer']['stream-height'])
    fps = int(jsonObj['viewer']['stream-fps'])
    print("W: ", width)
    print("H: ", height)
    print("FPS: ", fps)
    
    # recorded frame count from rotation input
    frame_start = 30 # start from frame no. 30 to skip (1) stuttering from depth initialization and (2) RGB auto-exposure initialization
    frame_loop_at = 506 # 506 for mode 2; 348 for mode 3; # frame no.[506 or 348] as flag for 1 rotation from frame no.30 (namefile)
    frame_per_rotation = frame_loop_at - frame_start # counts how many frames recorded for 1 rotation (quantity)
    rotation_period = frame_per_rotation / fps # time (second) needed for 1 rotation
    rpm = 60 / rotation_period
    print("rpm:", rpm)
    
    rotation_count= int(input("Rotation count (int)\t\t: "))
    frame_max = frame_start + (frame_per_rotation * rotation_count) # frame no.[frame_max] as flag for end of streaming (namefile)
    
    print("RPM: ", rpm) # rotation per minute
    print("Streaming will end at frame number: " + str(frame_max-1) +  " (%d frames" % (frame_max-frame_start) + ")") # [frame_max-frame_start] counts for how many frames will be recorded until end of streaming
    
    # preparing setup for streaming
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
       
    # preparing seetup for IMU (Intertial Measurement Unit)
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)
    
    # start streaming
    profile = pipeline.start(config)
    
    # set depth sensors preset option -> MediumDensity = 5
    # depth_sensor = profile.get_device().first_depth_sensor()
    # depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
    
    depth_sensor = profile.get_device()
    depth_first_sensor = profile.get_device().first_depth_sensor()
    
    advnc_mode = rs.rs400_advanced_mode(depth_sensor)
    advnc_mode.load_json(json_string)
    
    input_distance, radius, disparity_shift = None, None, None
    
    # set optimal disparity
    depth_table_control_group = advnc_mode.get_depth_table()
    input_distance = float(input("Center distance from camera (cm): "))
    radius = float(input("Radius from center (cm)\t\t: "))
    max_range = input_distance + radius
    print("max_range\t\t\t:", max_range)
    x_res = width
    h_fov = 87
    focal_length = 0.5 * (x_res / (tan(radians(h_fov/2))))
    baseline = 50
    disparity_shift = int(((focal_length * baseline) / max_range) / 10) # divided by 10 for scaling from mm to cm
    print("disparity_shift\t\t\t:", disparity_shift)
    depth_table_control_group.disparityShift = disparity_shift
    advnc_mode.set_depth_table(depth_table_control_group)
        
    # for using available preset
    # depth_first_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
    
    # get depth sensors scale
    depth_scale = depth_first_sensor.get_depth_scale()
    
    # show depth sensor parameter
    # print("depth_scale:", depth_scale)   
    # print("depth_sensor:", depth_sensor)
    # print("depth_first_sensor:", depth_first_sensor)
    # print(depth_table_control_group)
    
    # setup post-processing filter
    # spat_filter = rs.spatial_filter() # Spatial - edge-preserving spatial smoothing
    # hole_filter = rs.hole_filling_filter() # Hole - fill holes in depth map
    # temp_filter = rs.temporal_filter() # Temporal - reduces temporal noise
    
    # clip background of object in X meter(s) away
    clip_background_distance = 3 # crop if over 3 meter
    clipping_distance = clip_background_distance / depth_scale
    
    # align depth frames
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    # Start stream
    frame_count = 0 # frame number initialization (quantity = namefile)
    rotation_init = 0 # rotation initialization
    imwrite_state = False # True for active recording RGBD, False for pause recording RGBD (cv2.imwrite state active/paused)
    
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.visual_preset, 0) # 0=Dynamic, 1=Fixed, 2=Near, 3=Far
    
    # clamping maximum and minimum distance for depth map
    # colorizer.set_option(rs.option.min_distance, 0)
    # colorizer.set_option(rs.option.max_distance, 16)
    
    try:
        while True:
            # get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # pose = frames.get_pose_frame()
            
            # get data from accelerometer
            for frame in frames:
                if frame.is_motion_frame():
                    accel = accel_data(frame.as_motion_frame().get_motion_data())
                    break

            # align depth frames to color frames
            aligned_frames = align.process(frames)
            
            # post-process depth frame
            # depth_frame = aligned_depth_to_color.get_depth_frame()
            # depth_frame = spat_filter.process(depth_frame)
            # depth_frame = hole_filter.process(depth_frame)
            # depth_frame = temp_filter.process(depth_frame)
            
            # get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frames = aligned_frames.get_color_frame()
            
            # Validate that both frames are valid
            if not aligned_depth_frame or not aligned_color_frames:
                continue
            
            # place depth and color image as array
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            
            # apply colormap (dynamic=0) in aligned_depth_frame for depth_colormap
            depth_colormap = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
            
            # depth_image = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            color_image = np.asanyarray(aligned_color_frames.get_data())
            
            # get depth from center of image
            color_intrinsic = aligned_color_frames.profile.as_video_stream_profile().intrinsics
            # use pixel value of depth-aligned color image to get 3D axes
            center_x, center_y = int(width/2), int(height/2)
            center_depth = aligned_depth_frame.get_distance(center_x, center_y)
            # print("Z-depth from camera surface to pixel surface:", center_depth)
            dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrinsic, [center_x, center_y], center_depth)
            distance = sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
            # print("Distance from camera to pixel:", distance)
            depth_label =  str(round(distance*100, 3))
            
            # write color and depth picture
            if frame_count == 0:
                save_intrinsic_as_json(join(path_root, "camera_intrinsic.json"), aligned_color_frames)
            
            if imwrite_state == True:
                cv2.imwrite("%s/%06d.png" % (path_depth, frame_count), depth_image) # save depth_image, not depth_colormap
                cv2.imwrite("%s/%06d.jpg" % (path_color, frame_count), color_image)
                print("Saved color + depth image %06d" % frame_count)
                frame_count += 1
            
            # color indicator for remove background
            grey_color = 153
            
            # depth image is 1 channel, color is 3 channels
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) # channel
            
            # remove depth > clipping distance
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            
            # # apply colormap in depth_images
                        
            # combine RGB (left) and depth (right) with guides
            guidelines(bg_removed)
            guidelines(depth_colormap)
            # guidelines(color_image)
            
            # combine with depth background removed in RGB
            images = np.hstack((bg_removed, depth_colormap))
            cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Recorder Realsense', images)
            
            # delay for prepare recording
            key = cv2.waitKey(1)
            
            # play and pause recording (not streaming) using SPACEBAR
            if (imwrite_state == False) and (key == 32):
                imwrite_state = True
            elif (imwrite_state == True) and (key == 32):
                imwrite_state = False
            
            # pause recording every 1 rotation
            if (frame_count > frame_start) and (frame_count != frame_max) and ((frame_count-frame_start) % (frame_per_rotation) == 0) and (frame_count > frame_per_rotation):
                # imwrite_state = True
                imwrite_state = False # hold recording state, while keep streaming mode on
                if key == 32: # press SPACEBAR to continue record
                    rotation_init += 1
                    print(f"{bcolors.OK}^ rotation{bcolors.WARNING}", rotation_init, f"{bcolors.RESET}") # number of rotation elapsed
                    imwrite_state = True
                elif (key == 27) and ((frame_count-frame_start) % (frame_per_rotation) == 0):
                    rotation_init += 1
                    print(f"{bcolors.OK}^ rotation{bcolors.WARNING}", rotation_init, f"{bcolors.RESET}") # number of rotation elapsed
                    cv2.destroyAllWindows()
                    break
                elif key == 27: # wait for ESC button to close the window in the middle of streaming
                    print(f"{bcolors.OK}^ rotation{bcolors.WARNING}", rotation_init, f"{bcolors.RESET}") # number of rotation elapsed
                    cv2.destroyAllWindows()
                    break
            elif (frame_count == frame_max): # frame_count reach frame_max+1 OR pressing ESC for close window
                rotation_init += 1
                print(f"{bcolors.OK}^ rotation{bcolors.WARNING}", rotation_init, f"{bcolors.RESET}") # number of rotation elapsed
                print(frame_count-frame_start, "frames recorded")
                cv2.destroyAllWindows()
                break
            
            if key == 27:
                print(f"{bcolors.OK}^ rotation{bcolors.WARNING}", rotation_init, f"{bcolors.RESET}") # number of rotation elapsed
                cv2.destroyAllWindows()
                break             
    finally:
        pipeline.stop()
        
        # delete file 000000 to [frame_start] in folder color and folder depth to skip (1) stuttering from depth initialization and (2) RGB auto-exposure initialization
        if exists(path_color):
            file_color_sorted = sorted(listdir(path_color))
            for file_color in file_color_sorted[:frame_start]:
                remove((join(path_color, file_color)))
        else:
            print(f"{bcolors.FAIL}ERROR: File doesn't exist{bcolors.RESET}")
            
        if exists(path_depth):
            file_depth_sorted = sorted(listdir(path_depth))
            for file_depth in file_depth_sorted[:frame_start]:
                remove((join(path_depth, file_depth)))
        else:
            print(f"{bcolors.FAIL}ERROR: File doesn't exist{bcolors.RESET}")
            
        # write log
        if disparity_shift != None: # for custom preset
            writeScanLog.custom_log(folder_no, json_filename, rotation_init, input_distance, radius, disparity_shift)
        else: # for default preset
            writeScanLog.default_log(folder_no, json_filename, rotation_init)