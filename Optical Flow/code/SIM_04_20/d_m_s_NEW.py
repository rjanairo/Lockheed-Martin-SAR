
import Optical_flow_func_Def as fd
from cv2 import OPTFLOW_USE_INITIAL_FLOW
import timeit

#************** RECORDING AND INPUT SETUP *************************************
dir_original = 'ORIGINAL'
dir_opt_flow = 'OPT_FLOW'
dir_source_video = 'TEST_VIDEOS'
global out_original, out_opt_flow, txt_file, time_start # HAD TO MAKE THEM GLOBAL FOR USAGE INSIDE AND OUTSIDE FUNCTIONS
out_original, out_opt_flow, txt_file, time_start = fd.recording_setup_Windows(dir_original,dir_opt_flow)
#******************************************************************************

#******* creating an inital image for the algorithm   *************************
global cam # HAD TO MAKE IT GLOBAL FOR USAGE INSIDE AND OUTSIDE FUNCTIONS
cam = fd.cv.VideoCapture(fd.os.path.join(dir_source_video, 'DJI_0046.MOV'))
#cam = fd.cv.VideoCapture(0)
ret, prev=cam.read()
prev = fd.cv.resize(prev,(640,480))
prevgray = fd.cv.cvtColor(prev,fd.cv.COLOR_BGR2GRAY)
#*******************************************************************************

#**************** DECLARE FRAME COUNTER  ***************************************
global fr_count
fr_count = 0
#***************     MAIN LOOP START     ***************************************
while ret:
    fr_count += 1 # HAD TO MAKE IT GLOBAL FOR USAGE INSIDE AND OUTSIDE FUNCTIONS
    
    ret, img = cam.read() # reads NEXT frame from camera (NEEDED FOR ANY METHOD)

    if ret: # ******** CHECK if the input video is not yet finished *********

        #  ******  RESIZING AND CONVERTING "NEXT" IMAGE ************************
        img = fd.cv.resize(img, (640, 480))
        gray = fd.cv.cvtColor(img, fd.cv.COLOR_BGR2GRAY) 
        #  *********************************************************************

        # *******       GETTING FLOW            ********************************
        flow = fd.cv.calcOpticalFlowFarneback(prevgray,gray,None, 0.5, 10, 20, 5, 7, 1.5, fd.cv.OPTFLOW_FARNEBACK_GAUSSIAN)
        lines = fd.get_grid_coords_w_flow (img, flow, [0, 1], [0, 1],16)
        # **********************************************************************
#==========================AMPLITUDE============================================
        new_frame = fd.draw_simple_flow (gray, lines) #drawing results of flow amplitude method
        new_lines = fd.get_amplitude_over_threshold(lines, 'y', 3, 3)# GETTING COORDS OF PIXELS WITH LARGE FLOW - THIS IS A MAIN PIECE OF INFORMATION FOR TAKING DECISION
        new_frame = fd.draw_simple_flow_over_threshold(new_frame, new_lines) #drawing results of flow amplitude method

#===========================AREAS  !!!TO BE DEVELOPED  !!!=======================
            #new_frame = fd.draw_flow_Areas(gray, lines, 3)
            #decision, info = fd.make_decision_Areas_method(new_frame)#TO BE DEVELOPED
#================================================================================

#=============   PRINTING INFO INTO FRAME AND RECORDING OUTPUT VIDEOS  =========
        decision, info = fd.get_decision_by_flow(new_lines)# taking decision from flow amplitude method
        fd.print_info(new_frame, info, txt_file, fr_count, time_start)#printing decision info into frame
        
        out_original.write(img) # recording the original video input
        out_opt_flow.write(new_frame) # recording the opt flow output
        
        fd.cv.imshow("OpticalFlow", new_frame) # displays image with flow on it, for illustration
#================================================================================
#       IN DRONE VERSION, THERE SHOULD BE A MANEUVER HERE
#       IN SIMULATION VERSION, WE DO "sleepNrecord" PRETENDING WE DO A MANEUVER
#       !!! IMPORTANT CHANGE: VIDEO IS BEING RECORDED DURING THE MANEUVER
        if decision != "0":
            out_original, out_opt_flow, fr_count, cam = fd.sleepNrecord(2, cam, out_original, out_opt_flow, info, txt_file, fr_count, time_start)
#================================================================================            

#       IN DRONE VERSION, HERE SHOULD BE AN ATTEMPT TO MOVE TOWARD DESTINATION :
        #else:
            # move onward
#=================================================================================       
#========== READING AND PROCESSING "PREVOIUS" FRAME FOR OPT FLOW =================
        ret, prev = cam.read() # reads PREV frame from camera (NEEDED FOR ANY METHOD)
        if ret:
            prev = fd.cv.resize(prev,(640,480))
            prevgray = fd.cv.cvtColor(prev,fd.cv.COLOR_BGR2GRAY)
#=================================================================================

        key = fd.cv.waitKey(38)# wait for interrupt key
        if key == ord('q'):
            out_opt_flow.release()
            out_original.release()
            break
#********** stop if video has ended  **********************************************
    else:
        break 
#**************   END OF MAIN LOOP   **********************************************

print('landing')
txt_file.close()

