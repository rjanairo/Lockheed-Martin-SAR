from cv2.cv2 import OPTFLOW_USE_INITIAL_FLOW

import Optical_flow_func_Def as fd

def main():
    dir_original = 'ORIGINAL'
    dir_opt_flow = 'OPT_FLOW'
    dir_source_video = 'TEST_VIDEOS'
    # creating an inital image for the algorithm
    out_orignal, out_opt_flow = fd.recording_setup_Windows(dir_original,dir_opt_flow)
    cam = fd.cv.VideoCapture(fd.os.path.join(dir_source_video, 'DJI_0033.MOV'))
    cam = fd.cv.VideoCapture(0)
    ret, prev=cam.read()
    prev = fd.cv.resize(prev,(640,480))
    prevgray = fd.cv.cvtColor(prev,fd.cv.COLOR_BGR2GRAY)

    fr_count = 0
    while ret:
        fr_count += 1
        print(fr_count)
        ret, img = cam.read() # reads NEXT frame from camera (because we need both prevoius and next to analyze flow)
        if ret:
            img = fd.cv.resize(img, (640, 480))

            gray = fd.cv.cvtColor(img, fd.cv.COLOR_BGR2GRAY) #

            # estimates the flow between prevgray and gray
            flow = fd.cv.calcOpticalFlowFarneback(prevgray,gray,None, 0.5, 10, 20, 5, 7, 1.5, fd.cv.OPTFLOW_FARNEBACK_GAUSSIAN)
            #other_flow = fd.cv.calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 10, 20, 5, 7, 1.5, fd.cv.OPTFLOW_FARNEBACK_GAUSSIAN)
            prevgray = gray # current image now becomes the previous image
            threshold=5
            #lines = fd.get_grid_coords_w_flow (img, flow, [0, .75], [.33, .66],4)

            #new_lines = fd.get_flow_over_threshold(lines, 'y', 1,1)
            #print(len(new_lines))
            #new_frame = fd.draw_simple_flow (gray, lines)
            #new_frame = fd.draw_simple_flow_over_threshold(new_frame, new_lines)
            new_frame = fd.draw_flow_Areas(gray, flow)
            #decision, info = fd.get_decision_by_flow(new_lines)
            decision, info = fd.make_decision_Areas_method(new_frame)
            fd.print_info(new_frame, info)

            #out_orignal.write(img) # takes the original img and puts into the original write
            out_opt_flow.write(new_frame) # takes the opt flow and puts into the opt_flow
        
            fd.cv.imshow("OpticalFlow", new_frame) # displays image with flow on it, for illustration

            key = fd.cv.waitKey(38)
            if key == ord('q'):
                out_opt_flow.release()
                out_orignal
                break
        else:
            break
    print('landing')

main()
