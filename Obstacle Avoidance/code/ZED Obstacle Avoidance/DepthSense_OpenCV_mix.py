import numpy as np
import pyzed.sl as sl
import cv2
import math
import sys 

"""
future function: Modularizing the depth_measuring to allow multiple points to run at the same
time without killing resource usage/long code
needed parameters:
    - zed :variable sl.Camera()
    - 


"""


def main():
    # Create a ZED camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720

    # Open the camerab 
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.texture_confidence_threshold = 100

    #Declaring sl matricies for Depth Sensing
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    tr_np = mirror_ref.m

    res = sl.Resolution()
    res.width = 720
    res.height = 404

    #Declare ROI Specs
    roi_height = res.height
    roi_width = 450

    # Calculate the x-coordinates to define the rectangular ROI
    roi_x1 = (frame_width - roi_width) // 2  # Center the ROI horizontally
    roi_x2 = roi_x1 + roi_width

    #Declare your sl.Mat matrices for OpenCV
    image_zed = sl.Mat(res.width / 2, res.height / 2, sl.MAT_TYPE.U8_C4)
    depth_image_zed = sl.Mat(res.width/ 2, res.height / 2, sl.MAT_TYPE.U8_C4)
 
    key = ' '
    #Press Q key to end program
    while(key != 113):
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # Retrieve depth map. Depth is aligned on the left image
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            # Get and print distance value in m at the center of the image
            # We measure the distance camera - object using Euclidean distance
            x = round(image.get_width() / 2)
            y = round(image.get_height() / 2)
            err, point_cloud_value = point_cloud.get_value(x, y)

            # Retrieve the left image, depth image in the half-resolution (OPENCV)
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, res)
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, res)
            # Retrieve the RGBA point cloud in half resolution (OPENCV)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, res)
            
            depth_roi = depth_image_zed[0:roi_height,roi_width]

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv = image_zed.get_data()
            depth_image_ocv = depth_roi.get_data()
            print(image_ocv)

            #these should draw a green circle on area that is being measured for distance
            #this one is for the regular camera image
            cv2.circle(image_ocv, (round(res.width/2) , round(res.height/2)), 1, (0,255,0), 2) 
            #this one is for the depth image
            cv2.circle(depth_image_ocv, (round(res.width/2) , round(res.height/2)), 1, (0,255,0), 2) 
            
            #display normal image and depth image
            cv2.imshow("Image", image_ocv)

            
            # Split camera in two
            cv2.line(mask, (frame_width // 2, 0), (frame_width // 2, frame_height), (255, 0, 0), 2)
            cv2.imshow("Depth", depth_image_ocv)

            #calculating distance using euclidean distance formula: sqrt(x^2 + y^2 + z^2)
            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                point_cloud_value[1] * point_cloud_value[1] +
                                point_cloud_value[2] * point_cloud_value[2])
            #updating point cloud
            point_cloud_np = point_cloud.get_data()
            point_cloud_np.dot(tr_np)

            if not np.isnan(distance) and not np.isinf(distance):
                # distance * 3.28084 is used for conversion of m to ft
                print("Distance to Camera at ({}, {}) (image center): {:1.3} ft".format(x, y, distance*3.28084), end="\r")
            else:
                print("Can't estimate distance at this position.")
                print("Your camera is probably too close to the scene, please move it backwards.\n")
            #press Q key to end program
            key = cv2.waitKey(10)
    cv2.destroyAllWindows()
    zed.close()
    return

main()