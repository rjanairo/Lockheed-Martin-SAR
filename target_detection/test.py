import torch
import utils

display = utils.notebook_init() 
!python detect.py --weights yolov5s.pt --img 640 --conf 0.25 --source data/images
