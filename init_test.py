from imutils.video import VideoStream
from serial import Serial
# from utils.fonction import *
# from utils.list_ports import *

import argparse
import cv2
import imagezmq
import math as m
import numpy as np
import os
import pyzed.sl as sl
import socket
import threading
import time


def initialize():
    """
        DESCRIPTION  : Init all parameters.
    """
    global global_state, is_debug_option, fd, courbe, sender

    data_detection     = np.zeros(3)                        # Format(axes y position, distance, nombre object)

    # READ OPTION ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("debug", help="pass debug to 1 if you want more info")
    parser.add_argument("fd", help="fd is the factor to decrease the power of our motors")
    parser.add_argument("model", help="you can choose your model : 1 for HUMAN_BODY_FAST | 2 for MULTI_CLASS_BOX_MEDIUM | 3 for MULTI_CLASS_BOX")  
    parser.add_argument("courbe", help="pass courbe to 1 if you want the robot to curve")                                                                          # you can choose your model.
    parser.add_argument("ip_server", help="ip adress of server")
    parser.add_argument("ip_brain", help="ip adress of JETSON")
    args = parser.parse_args()

    # DEBUG OPTION.
    if(args.debug == "1"):
        is_debug_option = True

    # FACTOR OPTION.
    fd     = float(args.fd)

    # COURBE OPTION.
    courbe = float(args.courbe)

    # IP OPTION.
    IP     = args.ip_brain                     # This IP use to receive messsage so you have to enter the JETSON IP
    PORT   = 5000
    listeningAddress = (IP, PORT)

    # STREAM VIDEO INIT.
    sender = imagezmq.ImageSender(connect_to=f"tcp://{args.ip_server}:5555")
    
    # ZED CAMERA CONFIGURATION.        
    zed                           = sl.Camera()
    init_params                   = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720                             # Use HD720 video mode.
    init_params.camera_fps        = 60                             
    init_params.coordinate_units  = sl.UNIT.METER                                   # Set coordinate units.
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    
    if(zed.open(init_params) != sl.ERROR_CODE.SUCCESS):                             
        print("[ERR0] Can't open camera zed 2.")
        exit(-1)

    tracking_parameters = sl.PositionalTrackingParameters()                         # Enable tracking from zed.
    if(zed.enable_positional_tracking(tracking_parameters) != sl.ERROR_CODE.SUCCESS):
        print("[ERR0] Can't enable tracking of camera zed 2.")
        exit(-1)

    print(f"[INIT] - open camera zed 2.")
    # ZED OBJECT CONFIGURATION.
    image   = sl.Mat()                                                              # Left image from camera.
    pose    = sl.Pose()  
    runtime = sl.RuntimeParameters()

    # ZED OBJECT DETECTION CONFIGURATION.
    obj_param                     = sl.ObjectDetectionParameters()
    obj_param.enable_tracking     = True                                            # Tracking object enabled so that the objects ID won't change as long as the object stays on the ZED field of view.
    
    zed.enable_object_detection(obj_param)             

    if(args.model == "1"):                                                          # Use the body tracking with is the fastest but less accurate
        obj_param.detection_model     = sl.DETECTION_MODEL.HUMAN_BODY_FAST
        obj_param.enable_body_fitting = True

    if(args.model == "2"):
        obj_param.detection_model     = sl.DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM   # Use the box detection with is fast and accurate accurate

       
    obj_runtime_param                                = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 60

    if(args.model == "2" or args.model == "3"):
        obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]            # Only detect Persons
        
    objects = sl.Objects()
    print(f"[INIT] - all process on zed 2 are running.")
    
    # # OPEN COMMUNICATION WITH MICRO-CONTROLER.
    # port_name      = get_usb()                                                      # Get automaticly the micro controler.
    # ser            = Serial(port_name, 115200)
    # commande_motor = 'e'
    # if(ser.write(commande_motor.encode()) != 1):
    #     print(f"[ERR0] Can't call microcontroler on port {port_name}.")
    #     exit(-1)
    
    # CHECK IF MICRO-CONTROLLER SENDS DATA
    # while(check_ultrason_init(ser) == False):                                     # Check if data was different of zero.
    #     print(f"[WAIT] Waiting for good ultrason data.")

    # print(f"[INIT] - open microcontroler on port {port_name}.")

    # INIT SERVER PARAM AND SETUP SOCKET.   
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(listeningAddress)
    print(f"[INIT] - open server communication.")

    # SEND PARAM.
    return True

if __name__ == '__main__':

    param = initialize()
