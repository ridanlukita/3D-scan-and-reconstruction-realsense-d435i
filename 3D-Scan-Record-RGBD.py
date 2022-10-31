import pyrealsense2 as rs
import numpy as np
import cv2
from os import listdir, mkdir, remove
from os.path import exists, join
from math import tan, radians
import shutil
import json

class bcolors:
    OK = '\033[92m' #GREEN
    WARNING = '\033[93m' #YELLOW
    FAIL = '\033[91m' #RED
    RESET = '\033[0m' #RESET COLOR

def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics with open(filename, 'w') as outfile:
        obj = json.dump(
        {
        'width': intrinsics.width,
        'height': intrinsics.height,
        'intrinsic_matrix': [
            intrinsics.fx, 0, 0,
            0, intrinsics.fy, 0,
            intrinsics.ppx, intrinsics.ppy, 1
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

if __name__ == "__main__":
    
    #create folder
    path_color = join("dataset", "color")
    path_depth = join("dataset", "depth")
    make_clean_folder(path_depth)
    make_clean_folder(path_color)

    # create pipeline
    pipeline = rs.pipeline()

    # make realsense configuration
    config = rs.config()

    # import custom preset
    jsonObj = json.load(open("MediumDensityPreset-custom.json"))
    json_string= str(jsonObj).replace("'", '\"')

    # resolution and fps
    width = int(jsonObj['viewer']['stream-width'])
    height = int(jsonObj['viewer']['stream-height'])
    fps = int(jsonObj['viewer']['stream-fps'])
    print("W: ", width)
    print("H: ", height)
    print("FPS: ", fps)

    # recorded frame count from rotation input
    frame_start = 30 # start from frame no.[30] to skip (1) stuttering from depth initialization and (2) RGB auto-exposure initialization
    frame_loop_at = 506 # frame no.[506] as flag for 1 rotation from frame no.[30] (namefile)
    frame_per_rotation = frame_loop_at - frame_start # counts how many frames recorded for 1 rotation (quantity)
    rotation_period = frame_per_rotation / fps # time (second) needed for 1 rotation
    rpm = 60 / rotation_period
    rotation_count= int(input("Rotation count (int): "))
    frame_max = frame_start + (frame_per_rotation * rotation_count) # frame no.[frame_max] as flag for end of streaming (namefile)
    print("RPM: ", rpm) # rotation per minute
    print("Streaming will end at frame number: " + str(frame_max-1) + " (%d frames" % (frame_max-frame_start) + ")") # [frame_max-frame_start] counts for how many frames will be recorded until end of streaming
    
    # preparing configuration for streaming
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    # start streaming
    profile = pipeline.start(config)

    # get depth sensors profile
    depth_sensor = profile.get_device()
    depth_first_sensor = profile.get_device().first_depth_sensor()

    # use advanced mode
    advnc_mode = rs.rs400_advanced_mode(depth_sensor)
    advnc_mode.load_json(json_string)

    # set optimal disparity
    depth_table_control_group = advnc_mode.get_depth_table()
    distance = float(input("Center distance from camera (cm): "))
    radius = float(input("Radius from center (cm): "))
    max_range = distance + radius
    print("max_range:", max_range)
    x_res = width
    h_fov = 87
    focal_length = 0.5 * (x_res / (tan(radians(h_fov/2))))
    baseline = 50
    disparity_shift = int(((focal_length * baseline) / max_range) / 10) #divided by 10 for scaling from mm to cm
    print("disparity_shift:", disparity_shift)
    depth_table_control_group.disparityShift = disparity_shift
    advnc_mode.set_depth_table(depth_table_control_group)

    # get depth sensors scale
    depth_scale = depth_first_sensor.get_depth_scale()

    # clip background of object in X meter(s) away
    clip_background_distance = 3 # crop if over 3 meter
    clipping_distance = clip_background_distance / depth_scale

    # align depth frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Start stream
    frame_count = 0 # frame number initialization (namefile = quantity)
    rotation_init = 1 # rotation initialization
    imwrite_state = False # True for active recording RGBD, False for pause recording RGBD ("cv2.imwrite" state active/paused)

    try:
        while True:
            # get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # align depth frames to color frames
            aligned_frames = align.process(frames)

            # get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frames = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frames:
                continue

            # place depth and color image as array
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frames.get_data())

            # write color and depth picture
            if frame_count == 0:
                save_intrinsic_as_json(join("/home/ridan/Development/3D-Scanner/dataset", "camera_intrinsic.json"), color_frames)
            if imwrite_state == True:
                cv2.imwrite("%s/%06d.png" % (path_depth, frame_count), depth_image)
                cv2.imwrite("%s/%06d.jpg" % (path_color, frame_count), color_image)
                print("Saved color + depth image %06d" % frame_count)
                frame_count += 1
                
            # color indicator for remove background
            grey_color = 153

            # stack depth and color frame, depth image is 1 channel, color is 3 channels
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) # channel

            # remove depth > clipping distance
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

            # apply colormap in depth_images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=0.09), cv2.COLORMAP_JET)

            # create guidelines for bg_removed and depth_colormap
            guidelines(bg_removed)
            guidelines(depth_colormap)

            # combine bg_removed (left) and depth_colormap (right) in 1 array for window
            images = np.hstack((bg_removed, depth_colormap))

            # create window
            cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Recorder Realsense', images)
            key = cv2.waitKey(1)

            # play and pause recording (not streaming) using SPACEBAR
            if (imwrite_state == False) and (key == 32):
                imwrite_state = True
            elif (imwrite_state == True) and (key == 32):
                imwrite_state = False

            # pause recording every 1 rotation after frame_start
            if (frame_count > frame_start) and (frame_count != frame_max) and ((frame_count-frame_start) % (frame_per_rotation) == 0) and (frame_count > frame_per_rotation):
                # imwrite_state = True
                imwrite_state = False # hold recording state, while keep streaming mode on
                if key == 32: # press SPACEBAR to continue record
                    print(f"{bcolors.OK}^ rotation{bcolors.WARNING}",
                    rotation_init, f"{bcolors.RESET}") # number of rotation elapsed
                    imwrite_state = True
                    rotation_init += 1
                elif key == 27: # wait for ESC button to close the window in the middle of streaming
                    cv2.destroyAllWindows()
                    break

            elif (frame_count == frame_max) or key==27: # frame_count reach frame_max+1 OR pressing ESC for close window
                print(f"{bcolors.OK}^ rotation{bcolors.WARNING}",
                rotation_init, f"{bcolors.RESET}") # number of rotation elapsed
                print(frame_count-frame_start, "frames recorded")
                cv2.destroyAllWindows()
            break

    finally:
        pipeline.stop()
        # delete file 000000 to 000030 in folder color and folder depth to skip (1) stuttering from depth initialization and (2) RGB auto-exposure initialization
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