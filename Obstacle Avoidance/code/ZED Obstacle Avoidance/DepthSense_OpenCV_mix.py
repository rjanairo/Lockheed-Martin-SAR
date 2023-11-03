import numpy as np
import pyzed.sl as sl
import cv2
import math
import sys 

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
    # tr_np = mirror_ref.m

    res = sl.Resolution()
    res.width = 1280
    res.height = 720

    #New adjusted size for ROI
    new_width = 700
    new_height = 404

    #Text stuff
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (0, 255, 0)  # Color in BGR format
    thickness = 2
    text_coord = (30, 30)

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
            image_ocv = cv2.resize(image_ocv,(new_width,new_height))
            depth_image_ocv = depth_image_zed.get_data()
            depth_image_ocv = cv2.resize(depth_image_ocv,(new_width,new_height))

            #Contour Area
            threshold_min = 230 #min pixel intensity for depth
            threshold_max = 255 #max pixel intensity for depth
            contour_area_threshold = 1500 #threshold value for contour area size

            gray_frame = cv2.cvtColor(depth_image_ocv, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(gray_frame, threshold_min, threshold_max, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [contour for contour in contours if cv2.contourArea(contour) > contour_area_threshold]
            largest_contour = max(valid_contours, key=cv2.contourArea, default=None)
            centers = []
            if largest_contour is not None:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0: #if total number of pixels on the moment is not 0
                    center_x = int(M["m10"] / M["m00"]) # sum of x coordinates / total pixels in image
                    center_y = int(M["m01"] / M["m00"]) # sum of y coordinates / total pixels in image
                    centers.append((center_x, center_y))
                    #create point cloud value for measuring distance at the center point
                    err, point_cloud_value = point_cloud.get_value(center_x, center_y)
                    #calculate the distance to that center point
                    distance = Ecludian_Distance(point_cloud_value)
                    if not np.isnan(distance) and not np.isinf(distance) and (distance*3.28084) > 7:  #just testing for now a value of 7ft away from camera before it'll display the contour center and the text
                        # distance * 3.28084 is used for conversion of m to ft
                        print("Distance to center of contour at ({}, {}) (image center): {:1.3} ft".format(center_x, center_y, distance*3.28084), end="\r")

                        #create circle at center of the largest contour for visual purposes
                        cv2.circle(depth_image_ocv, (center_x, center_y), 10, (0, 0, 255), -1)
                        
                        #printing distance measured at largest contour center at top right of frame
                        cv2.putText(depth_image_ocv, str(round(distance*3.28084,2)) + " ft", (550,30), font, font_scale, font_color, thickness)
                        
                        if 25 < center_x < 360 and 25 < center_y < new_height - 100: #just testing values to prevent corner of screen distance issues
                            cv2.putText(depth_image_ocv, "Go Right", text_coord, font, font_scale, font_color, thickness)
                        elif 675 > center_x >= 360 and 50 < center_y < new_height - 100: #just testing values too prevent corner of screen distance issues
                            cv2.putText(depth_image_ocv, "Go Left", text_coord, font, font_scale, font_color, thickness)
                        elif center_y >= new_height - 100:
                            cv2.putText(depth_image_ocv, "Go Up ", text_coord, font, font_scale, font_color, thickness)
                        

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

            cv2.line(depth_image_ocv, (0, new_height - 100), (new_width, new_height - 100), (0, 0, 255), 2)
            # Split camera in two
            cv2.line(depth_image_ocv, (new_width // 2, 0), (new_width // 2, new_height-100), (255, 0, 0), 2)
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