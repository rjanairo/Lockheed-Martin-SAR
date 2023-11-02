import numpy as np
import pyzed.sl as sl
import cv2
import math
import sys 

"""
Possible Ideas:
using depth map get highest contrast pixel and create a contour around that pixel
if possible use depth mapping calculating to calculate the center of that contour?


"""
def Ecludian_Distance(point_cloud_value):
    #calculating distance using euclidean distance formula: sqrt(x^2 + y^2 + z^2) at center of camera
    return math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                        point_cloud_value[1] * point_cloud_value[1] +
                        point_cloud_value[2] * point_cloud_value[2])
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
    res.height = 500

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

            # Retrieve the left image, depth image in the half-resolution (OPENCV)
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, res)
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, res)
            # Retrieve the RGBA point cloud in half resolution (OPENCV)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, res)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv = image_zed.get_data()
            depth_image_ocv = depth_image_zed.get_data()

            #Contour Area
            gray_frame = cv2.cvtColor(depth_image_ocv, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(gray_frame, 240, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [contour for contour in contours if cv2.contourArea(contour) > 1500]
            largest_contour = max(valid_contours, key=cv2.contourArea, default=None)
            centers = []
            if largest_contour is not None:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    centers.append((center_x, center_y))
                    cv2.circle(depth_image_ocv, (center_x, center_y), 10, (0, 0, 255), -1)
                    err, point_cloud_value = point_cloud.get_value(center_x, center_y)
                    distance = Ecludian_Distance(point_cloud_value)
                    if not np.isnan(distance) and not np.isinf(distance):
                        # distance * 3.28084 is used for conversion of m to ft
                        print("Distance to center of contour at ({}, {}) (image center): {:1.3} ft".format(center_x, center_y, distance*3.28084), end="\r")
                    point_cloud_np = point_cloud.get_data()
                    point_cloud_np.dot(tr_np)

            # Get and print distance value in m at the center of the image
            # We measure the distance camera - object using Euclidean distance
            #x = round(image.get_width() / 2)
            #y = round(image.get_height() / 2)
            #err, point_cloud_value = point_cloud.get_value(x, y)

            #these should draw a green circle on area that is being measured for distance
            #this one is for the regular camera image
            #cv2.circle(image_ocv, (round(res.width/2) , round(res.height/2)), 1, (0,255,0), 2) 
            #this one is for the depth image
            #cv2.circle(depth_image_ocv, (round(res.width/2) , round(res.height/2)), 1, (0,255,0), 2) 
            
            #display normal image and depth image
            cv2.imshow("Image", image_ocv)

            cv2.line(depth_image_ocv, (0, res.height - 100), (res.width, res.height - 100), (0, 0, 255), 2)
            # Split camera in two
            cv2.line(depth_image_ocv, (res.width // 2, 0), (res.width // 2, res.height-100), (255, 0, 0), 2)
            cv2.imshow("Depth", depth_image_ocv)

            #calculating distance using euclidean distance formula: sqrt(x^2 + y^2 + z^2) at center of camera
            #distance = Ecludian_Distance(point_cloud_value)
            #updating point cloud
            #point_cloud_np = point_cloud.get_data()
            #point_cloud_np.dot(tr_np)

            #if not np.isnan(distance) and not np.isinf(distance):
                # distance * 3.28084 is used for conversion of m to ft
                #print("Distance to center of camera at ({}, {}) (image center): {:1.3} ft".format(x, y, distance*3.28084), end="\r")
            #press Q key to end program
            key = cv2.waitKey(10)
    cv2.destroyAllWindows()
    zed.close()
    return
main()