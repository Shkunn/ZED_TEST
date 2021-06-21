from utils.fonction import *

import argparse
import cv2
import math as m
import numpy as np
import pyzed.sl as sl
import socket
import threading
import time
# import ogl_viewer.viewer as gl

import firebase_admin
from firebase_admin import credentials
from firebase_admin import db   
import json

cred = credentials.Certificate('FIREBASE/firebase_SDK.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://rexinterface-default-rtdb.europe-west1.firebasedatabase.app/'
})

"""
    INFO    : This is all Global variable.
"""
ser                = None
data_position      = np.zeros(3)
data_detection     = np.zeros(3)                        # Format(axes y position, distance, nombre object)
human_selected     = False
id_selected        = -1

IP_send   = "172.21.72.151"
PORT_send = 5000
SendAddress = (IP_send, PORT_send)


def initialize():
    """
        DESCRIPTION  : Init all parameters.
    """
    global global_state, is_debug_option, fd, courbe, sender

    # READ OPTION ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("debug", help="pass debug to 1 if you want more info")
    parser.add_argument("model", help="you can choose your model : 1 for HUMAN_BODY_FAST | 2 for MULTI_CLASS_BOX_MEDIUM | 3 for MULTI_CLASS_BOX")  
    parser.add_argument("ip_server", help="ip address of server")
    parser.add_argument("ip_brain", help="ip address of JETSON")
    args = parser.parse_args()

    # DEBUG OPTION.
    if(args.debug == "1"):
        is_debug_option = True

    # IP OPTION.
    IP               = args.ip_brain                     # This IP use to receive messsage so you have to enter the JETSON IP
    PORT             = 5000
    listeningAddress = (IP, PORT)



    # ZED CONFIGURATION CAMERA.
    zed                              = sl.Camera()
    init_params                      = sl.InitParameters()
    init_params.camera_resolution    = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.coordinate_units     = sl.UNIT.METER         # Set coordinate units
    init_params.coordinate_system    = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    
    zed.open(init_params)





    # ZED CALIBRATION.
    zed.get_camera_information().camera_configuration.calibration_parameters.left_cam





    # INITIALISATION OBJECT FOR MAPPING.
    pymesh = sl.Mesh()        # Current incremental mesh.
    image = sl.Mat()          # Left image from camera.
    pose = sl.Pose()          # Pose object.





    # TRACKING PARAMETERS.
    tracking_parameters = sl.PositionalTrackingParameters()
    tracking_parameters.enable_area_memory = True
    zed.enable_positional_tracking(tracking_parameters)





    # SPATIAL MAPPING PARAMETERS.
    spatial_mapping_parameters = sl.SpatialMappingParameters(
        #resolution = sl.MAPPING_RESOLUTION.MEDIUM,
        #mapping_range = sl.MAPPING_RANGE.LONG,
        map_type = sl.SPATIAL_MAP_TYPE.MESH,
        use_chunk_only = True,
        max_memory_usage = 6000
    )

    mapping_parameters = sl.SpatialMappingParameters(resolution=sl.MAPPING_RESOLUTION.HIGH, use_chunk_only = True, mapping_range=sl.MAPPING_RANGE.MEDIUM, max_memory_usage = 4096*8)
    mapping_parameters.range_meter = mapping_parameters.get_range_preset(sl.MAPPING_RANGE.LONG)
    mapping_parameters.resolution_meter = 0.02

    zed.enable_spatial_mapping(mapping_parameters)




    # TIME PARAMETERS.
    runtime = sl.RuntimeParameters()




    # CLEAR FOR SAFETY?
    pymesh.clear()
    zed.enable_spatial_mapping()



    # ZED OBJECT DETECTION CONFIGURATION.
    obj_param                     = sl.ObjectDetectionParameters()
    obj_param.enable_tracking     = True                                            # Tracking object enabled so that the objects ID won't change as long as the object stays on the ZED field of view.

    if(args.model == "1"):                                                          # Use the body tracking with is the fastest but less accurate
        obj_param.detection_model     = sl.DETECTION_MODEL.HUMAN_BODY_FAST
        obj_param.enable_body_fitting = True

    if(args.model == "2"):
        obj_param.detection_model     = sl.DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM   # Use the box detection with is fast and accurate accurate

    if(zed.enable_object_detection(obj_param) != sl.ERROR_CODE.SUCCESS):             
        print("[ERR0] Can't enable object detection on camera zed 2.")
        exit(-1)

    obj_runtime_param                                = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 60

    if(args.model == "2" or args.model == "3"):
        obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]            # Only detect Persons
        
    objects = sl.Objects()
    print(f"[INIT] - all process on zed 2 are running.")





    # # OPEN COMMUNICATION WITH MICRO-CONTROLER.
    # port_name      = get_usb()                                                        # get automaticly the micro controler.
    # ser            = Serial(port_name, 115200)
    # commande_motor = 'e'
    # if(ser.write(commande_motor.encode()) != 1):
    #     print(f"[ERR0] Can't call microcontroler on port {port_name}.")
    #     exit(-1)





    # INIT SERVER PARAM AND SETUP SOCKET.   
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(listeningAddress)

    # CHANGE STATE.
    global_state = Robot_state.WAITING

    print("INITIALIZE FINISH")

    # SEND PARAM.
    params = ParamsInit(zed, image, pose, ser, sock, runtime, pymesh, objects, obj_runtime_param)
    return params

def thread_pointcloud(params):

    zed, image, pose, ser, sock, runtime, pymesh, objects, obj_runtime_param = params

    last_call = time.time()

    # Init time to get the FPS 
    last_time = time.time()

    key = ''
    while key != 113:
        # print("HZ SLAM THREAD    :", 1/(time.time() - last_time))
        last_time = time.time()

        # -get image.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)

        #region GET MAPPING 3D POINTS

        # -get position and spatial mapping state.
        zed.get_position(pose)
        zed.get_spatial_mapping_state()

        # -get duration for time mapping.
        duration = time.time() - last_call  
        
        if(duration > .05):
            # -see if spatial mapping is available.
            zed.request_spatial_map_async()
        
        if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
            # -if spatial mapping is available go mapping.
            zed.retrieve_spatial_map_async(pymesh)
            last_call = time.time()

        a = pose.pose_data()
        pose2 = np.array([[a[0,0],a[0,1],a[0,2],a[0,3]],
                          [a[1,0],a[1,1],a[1,2],a[1,3]],
                          [a[2,0],a[2,1],a[2,2],a[2,3]],
                          [a[3,0],a[3,1],a[3,2],a[3,3]]])

        # cv2.imshow("ZED", image.get_data())
        # key = cv2.waitKey(5)
        
        # In background, spatial mapping will use new images, depth and pose to create and update the mesh. No specific functions are required here.
        mapping_state = zed.get_spatial_mapping_state()
        # check param.
        mapping_param = zed.get_spatial_mapping_parameters()
    
        # Print spatial mapping state and FPS
        print("\rImages captured: {0} || {1} || {2} || {3}".format(mapping_state, mapping_param.resolution_meter, pymesh.vertices.shape[0], 1/(time.time() - last_time)))

        #endregion

    # cv2.destroyAllWindows()

    image.free(memory_type=sl.MEM.CPU)
    # Disable modules and close camera
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()

def thread_pointcloud_firebase(params):

    zed, image, pose, ser, sock, runtime, pymesh, objects, obj_runtime_param = params

    last_call = time.time()

    # Init time to get the FPS 
    last_time = time.time()

    points3D_dict              = {}
    points3D_dict["3D_points"] = {}

    limit_send_to_firebase = 0

    key = ''
    while key != 113:
        # print("HZ SLAM THREAD    :", 1/(time.time() - last_time))
        last_time = time.time()

        # get image.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)

        #region GET MAPPING 3D POINTS

        # get position and spatial mapping state.
        zed.get_position(pose)
        zed.get_spatial_mapping_state()

        # get duration for time mapping.
        duration = time.time() - last_call  
        
        if(duration > .05):
            # -see if spatial mapping is available.
            zed.request_spatial_map_async()
        
        if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
            # -if spatial mapping is available go mapping.
            zed.retrieve_spatial_map_async(pymesh)
            last_call = time.time()

        # a = pose.pose_data()
        # pose2 = np.array([[a[0,0],a[0,1],a[0,2],a[0,3]],
        #                   [a[1,0],a[1,1],a[1,2],a[1,3]],
        #                   [a[2,0],a[2,1],a[2,2],a[2,3]],
        #                   [a[3,0],a[3,1],a[3,2],a[3,3]]])

        # cv2.imshow("ZED", image.get_data())
        # key = cv2.waitKey(5)
        
        # In background, spatial mapping will use new images, depth and pose to create and update the mesh. No specific functions are required here.
        mapping_state = zed.get_spatial_mapping_state()
        # check param.
        mapping_param = zed.get_spatial_mapping_parameters()
    
        # Print spatial mapping state and FPS
        print("\rImages captured: {0} || {1} || {2} || {3}".format(mapping_state, mapping_param.resolution_meter, pymesh.vertices.shape[0], 1/(time.time() - last_time)))

        if pymesh.vertices.shape[0] > limit_send_to_firebase:
            mat_tolist = pymesh.vertices.tolist()

            points3D_dict["3D_points"] = mat_tolist
            
            # print(points3D_dict)
            
            ref = db.reference('/')
            ref.set(points3D_dict)

            limit_send_to_firebase += 500



        #endregion

    # cv2.destroyAllWindows()

    image.free(memory_type=sl.MEM.CPU)
    # Disable modules and close camera
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()

def thread_detection(params):
    """
        DESCRIPTION  : This thread will listen the camera zed sdk information and transfert
                       data to other thread. It will also send camera flux to server.
    """

    zed, image, pose, ser, sock, runtime, pymesh, objects, obj_runtime_param = params
    global data_position, data_detection, keypoint_to_home, global_state, human_selected, id_selected, copy_image_stream, new_image, lock

    # Init time to get the FPS 
    last_time = time.time()

    # Init object for the ZED camera
    object    = sl.ObjectData()

    while True:
        print("HZ SLAM THREAD    :", 1/(time.time() - last_time))
        last_time = time.time()

        # GET IMAGE.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)                             
        
        # CHECKING OBJECT DETECTION.
        zed.retrieve_objects(objects, obj_runtime_param)                                        # get 3D objects detection.   
        validation, i    = get_id_nearest_humain(objects)                                       # sort all object abd get the nearest.

        # CREATE THE IMAGE WE WILL SEND.
        image_draw       = image.get_data()

        if validation:
            if not human_selected:
                index = 0
                for obj in objects.object_list:
                    humain        = obj.bounding_box_2d
                    id            = obj.id
                    point_A       = (int(humain[0][0]), int(humain[0][1]))
                    point_B       = (int(humain[1][0]), int(humain[1][1]))
                    point_C       = (int(humain[2][0]), int(humain[2][1]))
                    point_D       = (int(humain[3][0]), int(humain[3][1]))
                    color         = None
                    if index == i:
                        color     = (   0,   0, 255)
                        data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
                        data_detection[1] = obj.position[0]
                        data_detection[2] = len(objects.object_list)
                    else:
                        color     = (   0, 255,   0)
                    
                    image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
                    image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
                    image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
                    image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)
                    
                    font          = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale     = 2
                    fontColor     = (255,255,255)
                    lineType      = 2

                    middle_x      = (point_A[0] + point_B[0]) / 2
                    middle_y      = (point_A[1] + point_D[1]) / 2

                    cv2.putText(image_draw, str(id), 
                        (int(middle_x), int(middle_y)), 
                        font, 
                        fontScale,
                        fontColor,
                        lineType)
                    
                    index += 1

                    # print("\rImages captured: {0} || {1}".format(point_A, point_D))
            
            if human_selected and check_if_search_id_is_present(id, objects): 
                id = id_selected
                objects.get_object_data_from_id(object, id)                         # return the object of the selected id
                humain        = object.bounding_box_2d

                point_A       = (int(humain[0][0]), int(humain[0][1]))
                point_B       = (int(humain[1][0]), int(humain[1][1]))
                point_C       = (int(humain[2][0]), int(humain[2][1]))
                point_D       = (int(humain[3][0]), int(humain[3][1]))
                color         = (255, 0, 0)
                image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
                image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
                image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
                image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)

                font          = cv2.FONT_HERSHEY_SIMPLEX
                fontScale     = 2
                fontColor     = (255,255,255)
                lineType      = 2

                middle_x      = (point_A[0] + point_B[0]) / 2
                middle_y      = (point_A[1] + point_D[1]) / 2

                # text = 'ID:' + str(id)
                cv2.putText(image_draw, str(id), 
                    (int(middle_x), int(middle_y)), 
                    font, 
                    fontScale,
                    fontColor,
                    lineType)

                data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
                data_detection[1] = object.position[0]                                                          # WARING! Normalement il renvoie tout le temps une valeur valide.
                data_detection[2] = len(objects.object_list)
        else:
            data_detection    = np.zeros(3)  
        
        print("\rImages captured: {0}".format(1/(time.time() - last_time)))
        
        # msg = "hello"
        # cc = msg.encode()
        # with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as opened_socket:
        #     opened_socket.setblocking(0)
        #     opened_socket.sendto(cc, SendAddress)

        # print(msg)
            

        # RESIZE WINDOW OTHERWISE STREAM TO SLOW 
        image_draw = cv2.resize(image_draw, (int(352), int(240)))

        #CREATE A COPY OF THE IMAGE TO SEND IT IN ANOTHER THREAD OTHERWISE LOOSE A LOT OF FPS
        with lock:
            new_image = True
        # copy_image_stream = image_draw.copy()
        cv2.imshow("ZED", image_draw)
        key = cv2.waitKey(5)

def both_thread_in_one(params):
    zed, image, pose, ser, sock, runtime, pymesh, objects, obj_runtime_param = params

    """
    ADD HUMAN DETECTION INSIDE THIS THREAD
    Data that refers to human detection are between comments 
    """
    global data_position, data_detection, keypoint_to_home, global_state, human_selected, id_selected, copy_image_stream, new_image, lock
    
    # Init object for the ZED camera
    object    = sl.ObjectData()

    """
    ^^^^ INIT OBJECT FOR DETECTION ^^^^
    """


    last_call = time.time()
    last_time = time.time()


    key = ''
    while key != 113:
        # -get image.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)

        #region GET MAPPING 3D POINTS

        # -get position and spatial mapping state.
        zed.get_position(pose)
        zed.get_spatial_mapping_state()

        # -get duration for time mapping.
        duration = time.time() - last_call  
        
        if(duration > .05):
            # -see if spatial mapping is available.
            zed.request_spatial_map_async()
        
        if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
            # -if spatial mapping is available go mapping.
            zed.retrieve_spatial_map_async(pymesh)
            last_call = time.time()

        # a = pose.pose_data()
        # pose2 = np.array([[a[0,0],a[0,1],a[0,2],a[0,3]],
        #                   [a[1,0],a[1,1],a[1,2],a[1,3]],
        #                   [a[2,0],a[2,1],a[2,2],a[2,3]],
        #                   [a[3,0],a[3,1],a[3,2],a[3,3]]])

        # cv2.imshow("ZED", image.get_data())
        # key = cv2.waitKey(5)
        
        # In background, spatial mapping will use new images, depth and pose to create and update the mesh. No specific functions are required here.
        mapping_state = zed.get_spatial_mapping_state()
        # check param.
        mapping_param = zed.get_spatial_mapping_parameters()
        #endregion


        #region HUMAN DETECTION
        
        # CHECKING OBJECT DETECTION.
        zed.retrieve_objects(objects, obj_runtime_param)                                        # get 3D objects detection.   
        validation, i    = get_id_nearest_humain(objects)                                       # sort all object abd get the nearest.

        # CREATE THE IMAGE WE WILL SEND.
        image_draw       = image.get_data()

        if validation:
            if not human_selected:
                index = 0
                for obj in objects.object_list:
                    humain        = obj.bounding_box_2d
                    id            = obj.id
                    point_A       = (int(humain[0][0]), int(humain[0][1]))
                    point_B       = (int(humain[1][0]), int(humain[1][1]))
                    point_C       = (int(humain[2][0]), int(humain[2][1]))
                    point_D       = (int(humain[3][0]), int(humain[3][1]))
                    color         = None
                    if index == i:
                        color     = (   0,   0, 255)
                        data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
                        data_detection[1] = obj.position[0]
                        data_detection[2] = len(objects.object_list)
                    else:
                        color     = (   0, 255,   0)
                    
                    image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
                    image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
                    image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
                    image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)
                    
                    font          = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale     = 2
                    fontColor     = (255,255,255)
                    lineType      = 2

                    middle_x      = (point_A[0] + point_B[0]) / 2
                    middle_y      = (point_A[1] + point_D[1]) / 2

                    cv2.putText(image_draw, str(id), 
                        (int(middle_x), int(middle_y)), 
                        font, 
                        fontScale,
                        fontColor,
                        lineType)
                    
                    index += 1
            
            if human_selected and check_if_search_id_is_present(id, objects): 
                id = id_selected
                objects.get_object_data_from_id(object, id)                         # return the object of the selected id
                humain        = object.bounding_box_2d

                point_A       = (int(humain[0][0]), int(humain[0][1]))
                point_B       = (int(humain[1][0]), int(humain[1][1]))
                point_C       = (int(humain[2][0]), int(humain[2][1]))
                point_D       = (int(humain[3][0]), int(humain[3][1]))
                color         = (255, 0, 0)
                image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
                image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
                image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
                image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)

                font          = cv2.FONT_HERSHEY_SIMPLEX
                fontScale     = 2
                fontColor     = (255,255,255)
                lineType      = 2

                middle_x      = (point_A[0] + point_B[0]) / 2
                middle_y      = (point_A[1] + point_D[1]) / 2

                # text = 'ID:' + str(id)
                cv2.putText(image_draw, str(id), 
                    (int(middle_x), int(middle_y)), 
                    font, 
                    fontScale,
                    fontColor,
                    lineType)

                data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
                data_detection[1] = object.position[0]                                                          # WARING! Normalement il renvoie tout le temps une valeur valide.
                data_detection[2] = len(objects.object_list)
        else:
            data_detection    = np.zeros(3)  

        # RESIZE WINDOW OTHERWISE STREAM TO SLOW 
        image_draw = cv2.resize(image_draw, (int(352), int(240)))

        #CREATE A COPY OF THE IMAGE TO SEND IT IN ANOTHER THREAD OTHERWISE LOOSE A LOT OF FPS
        # with lock:
        #     new_image = True
        # copy_image_stream = image_draw.copy()
        cv2.imshow("ZED", image_draw)
        key = cv2.waitKey(5)

        #endregion


        # Print spatial mapping state and FPS
        print("\rImages captured: {0} || {1} || {2} || {3}".format(mapping_state, mapping_param.resolution_meter, pymesh.vertices.shape[0], 1/(time.time() - last_time)))
        last_time = time.time()


    cv2.destroyAllWindows()

    image.free(memory_type=sl.MEM.CPU)
    # Disable modules and close camera
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()

def thread_detection_socket(params):
    """
        DESCRIPTION  : This thread will listen the camera zed sdk information and transfert
                       data to other thread. It will also send camera flux to server.
    """

    zed, image, pose, ser, sock, runtime, pymesh, objects, obj_runtime_param = params
    global data_position, data_detection, keypoint_to_home, global_state, human_selected, id_selected, copy_image_stream, new_image, lock

    # Init time to get the FPS 
    last_time = time.time()

    human_dict               = {}
    human_dict["Human_pose"] = {}

    # Init object for the ZED camera
    object    = sl.ObjectData()

    while True:
        print("HZ SLAM THREAD    :", 1/(time.time() - last_time))
        last_time = time.time()

        # GET IMAGE.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)                             
        
        # CHECKING OBJECT DETECTION.
        zed.retrieve_objects(objects, obj_runtime_param)                                        # get 3D objects detection.   
        validation, i    = get_id_nearest_humain(objects)                                       # sort all object abd get the nearest.

        # CREATE THE IMAGE WE WILL SEND.
        image_draw       = image.get_data()

        if validation:
            if not human_selected:
                index = 0
                for obj in objects.object_list:
                    humain        = obj.bounding_box_2d
                    id            = obj.id
                    point_A       = (int(humain[0][0]), int(humain[0][1]))
                    point_B       = (int(humain[1][0]), int(humain[1][1]))
                    point_C       = (int(humain[2][0]), int(humain[2][1]))
                    point_D       = (int(humain[3][0]), int(humain[3][1]))
                    color         = None

                    human_poseA = np.asarray([point_A])
                    human_poseD = np.asarray([point_D])

                    human_pose = np.append(human_poseA, human_poseD)

                    mat_tolist = human_pose.tolist()

                    human_dict["Human_pose"][str(id)] = mat_tolist

                    if index == i:
                        color     = (   0,   0, 255)
                        data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
                        data_detection[1] = obj.position[0]
                        data_detection[2] = len(objects.object_list)
                    else:
                        color     = (   0, 255,   0)
                    
                    image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
                    image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
                    image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
                    image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)
                    
                    font          = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale     = 2
                    fontColor     = (255,255,255)
                    lineType      = 2

                    middle_x      = (point_A[0] + point_B[0]) / 2
                    middle_y      = (point_A[1] + point_D[1]) / 2

                    cv2.putText(image_draw, str(id), 
                        (int(middle_x), int(middle_y)), 
                        font, 
                        fontScale,
                        fontColor,
                        lineType)
                    
                    index += 1

                json_msg = json.dumps(human_dict).encode('utf-8')
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as opened_socket:
                    opened_socket.setblocking(0)
                    opened_socket.sendto(json_msg, SendAddress)

                human_dict = {}
                human_dict["Human_pose"] = {}
                

            if human_selected and check_if_search_id_is_present(id, objects): 
                id = id_selected
                objects.get_object_data_from_id(object, id)                         # return the object of the selected id
                humain        = object.bounding_box_2d

                point_A       = (int(humain[0][0]), int(humain[0][1]))
                point_B       = (int(humain[1][0]), int(humain[1][1]))
                point_C       = (int(humain[2][0]), int(humain[2][1]))
                point_D       = (int(humain[3][0]), int(humain[3][1]))


                human_poseA = np.asarray([point_A])
                human_poseD = np.asarray([point_D])

                human_pose = np.append(human_poseA, human_poseD)

                mat_tolist = human_pose.tolist()

                human_dict["Human_pose"][str(id)] = mat_tolist


                color         = (255, 0, 0)
                image_draw    = cv2.line(image_draw, point_A, point_B, color, 5)
                image_draw    = cv2.line(image_draw, point_B, point_C, color, 5)
                image_draw    = cv2.line(image_draw, point_C, point_D, color, 5)
                image_draw    = cv2.line(image_draw, point_D, point_A, color, 5)

                font          = cv2.FONT_HERSHEY_SIMPLEX
                fontScale     = 2
                fontColor     = (255,255,255)
                lineType      = 2

                middle_x      = (point_A[0] + point_B[0]) / 2
                middle_y      = (point_A[1] + point_D[1]) / 2

                # text = 'ID:' + str(id)
                cv2.putText(image_draw, str(id), 
                    (int(middle_x), int(middle_y)), 
                    font, 
                    fontScale,
                    fontColor,
                    lineType)

                data_detection[0] = int((humain[0][0]+humain[1][0])/2) - (int(image_draw.shape[1]/2))
                data_detection[1] = object.position[0]                                                          # WARING! Normalement il renvoie tout le temps une valeur valide.
                data_detection[2] = len(objects.object_list)

                json_msg = json.dumps(human_dict).encode('utf-8')
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as opened_socket:
                    opened_socket.setblocking(0)
                    opened_socket.sendto(json_msg, SendAddress)


                human_dict = {}
                human_dict["Human_pose"] = {}
        
        else:
            data_detection    = np.zeros(3)  
            

        # RESIZE WINDOW OTHERWISE STREAM TO SLOW 
        image_draw = cv2.resize(image_draw, (int(352), int(240)))

        #CREATE A COPY OF THE IMAGE TO SEND IT IN ANOTHER THREAD OTHERWISE LOOSE A LOT OF FPS
        with lock:
            new_image = True
        # copy_image_stream = image_draw.copy()
        cv2.imshow("ZED", image_draw)
        key = cv2.waitKey(5)

if __name__ == "__main__":
    
    params = initialize()
    lock = threading.Lock()

    # # Thread human detection.
    # thread_1 = threading.Thread(target=thread_detection         , args=(params,))
    # thread_1.start()

    # Thread pointcloud.
    # thread_2 = threading.Thread(target=thread_pointcloud        , args=(params,))
    # thread_2.start()

    # # Thread both thread in one.
    # thread_3 = threading.Thread(target=both_thread_in_one       , args=(params,))
    # thread_3.start()

    thread_4 = threading.Thread(target=thread_detection_socket    , args=(params,))
    thread_4.start()

    # thread_5 = threading.Thread(target=thread_pointcloud_firebase , args=(params,))
    # thread_5.start()
    
    # thread_1.join()
    # thread_2.join()
    # thread_3.join()
    thread_4.join()
    # thread_5.join()