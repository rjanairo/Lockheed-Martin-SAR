import jetson.inference
import jetson.utils

import numpy as np

net = jetson.inference.depthNet(network = "fcn-mobilenet")
video_input = jetson.utils.videoSource("/dev/video0", ["--input-width=1920", "--input-height=1080","--input-rate=21.0"])

depth_field = net.GetDepthField()

while True:
    img = video_input.Capture()	# assumes you have created an input videoSource stream
    net.Process(img)
    jetson.utils.cudaDeviceSynchronize() # wait for GPU to finish processing, so we can use the results on CPU
	
    # find the min/max values with numpy
