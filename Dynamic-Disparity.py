import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import sys
from os import mkdir
from os.path import exists, join
import shutil
import json
import time
from math import tan, radians
from enum import IntEnum 
    
# def save_intrinsic_as_json(filename, frame):
#     intrinsics = frame.profile.as_video_stream_profile().intrinsics
#     with open(filename, 'w') as outfile:
#         obj = json.dump(
#             {
#                 'width':
#                     intrinsics.width,
#                 'height':
#                     intrinsics.height,
#                 'intrinsic_matrix': [
#                     intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
#                     intrinsics.ppy, 1
#                 ]
#             },
#             outfile,
#             indent=4)

# def make_clean_folder(path_folder):
#     if not exists(path_folder):
#         mkdir(path_folder)
#     else:
#         user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
#         if user_input.lower() == 'y':
#             shutil.rmtree(path_folder)
#             mkdir(path_folder)
#         else:
#             exit()

if __name__ == "__main__":    
    
    #create folder
    # path_color = join("/home/ridan/Development/3D-Scanner/dataset", "color")
    # path_depth = join("/home/ridan/Development/3D-Scanner/dataset", "depth")
    # make_clean_folder(path_depth)
    # make_clean_folder(path_color)
    
    # create pipeline
    pipeline = rs.pipeline()
    
    # make realsense configuration
    config = rs.config()
        
    # import custom preset
    jsonObj = json.load(open("MediumDensityPreset.json"))
    json_string= str(jsonObj).replace("'", '\"')
            
    print("W: ", int(jsonObj['viewer']['stream-width']))
    print("H: ", int(jsonObj['viewer']['stream-height']))
    print("FPS: ", int(jsonObj['viewer']['stream-fps']))
        
    # preparing configuration for streaming
    config.enable_stream(rs.stream.depth, int(jsonObj['viewer']['stream-width']), int(jsonObj['viewer']['stream-height']), rs.format.z16, int(jsonObj['viewer']['stream-fps']))
    config.enable_stream(rs.stream.color, int(jsonObj['viewer']['stream-width']), int(jsonObj['viewer']['stream-height']), rs.format.bgr8, int(jsonObj['viewer']['stream-fps']))
    
    # start streaming
    profile = pipeline.start(config)
    
    # set depth sensors preset option -> MediumDensity = 5
    # depth_sensor = profile.get_device().first_depth_sensor()
    # depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
    
    depth_sensor = profile.get_device()
    depth_first_sensor = profile.get_device().first_depth_sensor()
    
    advnc_mode = rs.rs400_advanced_mode(depth_sensor)
    advnc_mode.load_json(json_string)

    # set dynamic disparity
    depth_table_control_group = advnc_mode.get_depth_table()
    distance = float(input("Center distance from camera (cm): "))
    radius = float(input("Radius from center (cm): "))
    max_range = distance + radius
    print("max_range:", max_range)
    Xres = int(jsonObj['viewer']['stream-width'])
    HFOV = 87
    focal_length = 0.5 * (Xres / (tan(radians(HFOV/2))))
    baseline = 50    
    disparity_shift = int(((focal_length * baseline) / max_range) / 10)
    print("disparity_shift:", disparity_shift)
    depth_table_control_group.disparityShift = disparity_shift
    advnc_mode.set_depth_table(depth_table_control_group)
    
    # get depth sensors scale
    depth_scale = depth_first_sensor.get_depth_scale()
    
    #setup post-processing filter
    # spat_filter = rs.spatial_filter() # Spatial - edge-preserving spatial smoothing
    # hole_filter = rs.hole_filling_filter() # Hole - fill holes in depth map
    # temp_filter = rs.temporal_filter() # Temporal - reduces temporal noise
    
    #clip background of object in X meter(s) away
    clip_background_distance = 3 # crop if over 3 meter
    clipping_distance = clip_background_distance / depth_scale
    
    # align color frames
    align_color = rs.align(rs.stream.color)
    align_to = rs.stream.depth
    align = rs.align(align_to)
    frame_count = 0
    # Start stream
    try:
        while True:
            # get frameset of color and depth
            frames = pipeline.wait_for_frames()
            
            # align depth frames to color frames
            aligned_depth_to_color = align_color.process(frames)
            
            # Post-process depth frame
            # depth_frame = aligned_depth_to_color.get_depth_frame()
            # depth_frame = spat_filter.process(depth_frame)
            # depth_frame = hole_filter.process(depth_frame)
            # depth_frame = temp_filter.process(depth_frame)            
            
            # get aligned frames
            aligned_frames =  align.process(frames)
            depth_frames = aligned_depth_to_color.get_depth_frame()
            color_frames = aligned_depth_to_color.get_color_frame()
            aligned_color_frame = aligned_frames.get_color_frame()
            
            # Validate that both frames are valid
            if not depth_frames or not color_frames: continue
            
            color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
            depth_image = np.asanyarray(depth_frames.get_data())
            color_image = np.asanyarray(color_frames.get_data())
            
            # x, y = 424, 240
            x_center = int(int(jsonObj['viewer']['stream-width'])/2)
            y_center = int(int(jsonObj['viewer']['stream-height'])/2)
            range_pixels = 100
            upperLeftX = int(x_center - range_pixels/2)
            upperLeftY = int (y_center - range_pixels/2)
            x_pixel = upperLeftX            
            y_pixel = upperLeftY
            
            pixel_depth = []
            pixel_depth_count = 0
            
            # for y_pixel in range(range_pixels+upperLeftY):
            #     for x_pixel in range(range_pixels+upperLeftX):
            #         if (depth_frames.get_distance(x_pixel, y_pixel)) == 0:
            #             continue
            #         else:
            #             pixel_depth_count = pixel_depth_count+1
            #             pixel_depth = depth_frames.get_distance(x_pixel, y_pixel)
            #             dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x_center,y_center], pixel_depth)
            #             distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
            #             # print(distance)
                        
            
            # cv2.rectangle(images, (x_center-range_pixels, y_center-range_pixels), (x_center+range_pixels, y_center+range_pixels), (0, 255, 0), 2)
        
            
            
            # average_depth = depth_frames.get_distance(x_pixel_depth, y_pixel_depth)
            
            # depth = depth_frames.get_distance(x_center, y_center)
            # dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x_center,y_center], depth)
            # # dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
            # distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
            # print("Distance from camera to pixel:", distance)
            # print("Z-depth from camera surface to pixel surface:", depth)
            
            # write color and depth picture
            # if frame_count == 0:
            #     save_intrinsic_as_json(join("/home/ridan/Development/3D-Scanner/dataset", "camera_intrinsic.json"), color_frames)
            # cv2.imwrite("%s/%06d.png" % (path_depth, frame_count), depth_image)
            # cv2.imwrite("%s/%06d.jpg" % (path_color, frame_count), color_image)
            # print("Saved color + depth image %06d" % frame_count)
            # frame_count += 1            
            
            # color indicator for remove background
            grey_color = 153
            
            # stack depth and color frame
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) # channel
            
            # remove depth > clipping distance
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            
            # Render images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((bg_removed, depth_colormap))
                        
            # for y_pixel in range(range_pixels+upperLeftY):
            #     for x_pixel in range(range_pixels+upperLeftX):
            #         cv2.rectangle(images, (x_pixel, y_pixel), (x_pixel, y_pixel), (0, 255, 0), 1)
            #         # pixel_depth = depth_frames.get_distance(x_pixel, y_pixel)
            #         cv2.
            
            # cv2.rectangle(images, (x_center-range_pixels, y_center-range_pixels), (x_center+range_pixels, y_center+range_pixels), (0, 255, 0), 1)
            # cv2.rectangle(images, (upperLeftX, upperLeftY), (range_pixels+upperLeftX, range_pixels+upperLeftY), (0, 255, 0), 1)
                
            
            
            cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Recorder Realsense', images)
            key = cv2.waitKey(1)
            
            # ESC for close window
            if key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()