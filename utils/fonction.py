from serial import Serial
from typing import NamedTuple

import fcntl
import math as m
import numpy as np
import pyzed.sl as sl
import socket
import struct


class Robot_state:
    INITIALISATION  = "initialisation"
    WAITING         = "waiting"
    FOLLOWING       = "follow"
    LOST            = "lost"
    HOME            = "home"
    MANUALMODE      = "manual"
    RESET           = "reset"
 
class Control_user: 
    STOP            = "0"
    FORWARD         = "1"
    BACKWARD        = "2"
    LEFT            = "3"
    RIGHT           = "4"
    TURN_LEFT       = "5"
    TURN_RIGHT      = "6"
    DIAG_FOR_LEFT   = "7"
    DIAG_FOR_RIGHT  = "8"
    DIAG_BACK_LEFT  = "9"
    DIAG_BACK_RIGHT = "10"

class ParamsInit(NamedTuple):
    zed: sl.Camera
    image: sl.Mat
    pose: sl.Pose
    ser: Serial
    socket: socket.SocketType
    runtime: sl.RuntimeParameters
    pymesh: sl.Mesh
    objects: sl.Objects
    obj_runtime_param: sl.ObjectDetectionRuntimeParameters

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

def get_id_nearest_humain(objects):
    '''
    DESCRIPTION: This function will take all object detect
                 and send back the index of the nearest humain.
    INPUT      :
        *objects    > list of objects from zed sdl. 
    OUTPUT     :
        *valid      > if list is bigger than 0.
        *index_list > position of nearest humain in objects list.
    '''

    if len(objects.object_list) == 0:
        return False, None
    
    min_distance = 5
    index_list   = 0
    i            = 0
    for obj in objects.object_list:
        if obj.position[0] < min_distance:
            min_distance = obj.position[0]
            index_list = i 
        i += 1

    return True, index_list    

def check_ultrason_init(ser):
    '''
        DESCRIPTION: This function will run at the begining in
                     the initialisation process and will check if 
                     the ultrason sensor connection is working.
    ''' 

    data_ultrasensor = np.zeros(4)
    data = ser.readline()
    encodor_data  = (data.decode('utf-8')).split(sep='/')
    
    if len(encodor_data) == 5:
        data_ultrasensor[0] = float(encodor_data[0])
        data_ultrasensor[1] = float(encodor_data[1])
        data_ultrasensor[2] = float(encodor_data[2])
        data_ultrasensor[3] = float(encodor_data[3])
        if(data_ultrasensor[0] != 0 or data_ultrasensor[1] != 0 or data_ultrasensor[2] != 0 or data_ultrasensor[3] != 0):
            return True

    return False

def compute_distance_between_points(point_A, point_B):
    return (((point_B[0] - point_A[0,0])**2)+((point_B[1] - point_A[0,1])**2))**0.5

def check_if_new_keypoint(keypoints, current_position, threshold, debug):
    """
        DESCRIPTION: This function will check current position and
                     if mouvement since last keypoint is bigger than a
                     threshold, we'll add new keypoint.
        INPUT:
            *keypoints        > numpy(numpy(x, y, yaw)) it's all keypoints.
            *current_position > numpy(x, y, yaw).
            *threshold        > double in meters.
            *debug            > bool to show keypoints list when we add new.
    """

    # Compute distance since last keypoints.
    distance = compute_distance_between_points(current_position, keypoints[-1])

    if debug:
        pass
        # print("KEYPOINTS LIST:", keypoints)

    if distance > threshold:
        keypoints = np.concatenate((keypoints, current_position), axis=0)
        # if debug:
        #     print(keypoints)
        return keypoints

    return keypoints

def calcul_vector(current_position, keypoint):
    """
        DESCRIPTION: calcul angle between current pose and keypoint in 
                     global reference.
        OUTPUT:
            * current_position = (x, y) index of current robot position.
            * testouille       = (x, y)
        INPUT:
            * angle_degree     = (float) of vector from path
            * norm_vector      = (float) of norm from vector
    """

    x_sum = keypoint[0]-current_position[0]
    y_sum = keypoint[1]-current_position[1]
    
    angle_degree = np.arccos((x_sum)/(x_sum**2+y_sum**2)**0.5)
    if y_sum < 0:
        angle_degree = (m.pi - angle_degree) + m.pi

    return angle_degree * (180/m.pi)

def check_if_we_reach_keypoint(point_A, point_B, threshold):
    """
        DESCRIPTION: check if we reach keypoint.
        INPUT:
            * point_A    > 2 dimensions vector.
            * point_B    > 1 dimension vector.
    """
    return compute_distance_between_points(point_A, point_B) < threshold

def check_if_search_id_is_present(id, objects):
    """
        DESCRIPTION: check if the id select by user interface is 
                     still observable on screen.
        INPUT:
            * id         > id select by user.
            * objects    > list of observable objects.
    """
    for obj in objects.object_list:
        if id == obj.id:
            return True
    return False

def return_index_with_id(id, objects):
    """
        DESCRIPTION: return the index in list objects with object
            with this id.
        INPUT:
            * id         > id select by user.
            * objects    > list of observable objects.
    """
    index = 0
    for obj in objects.object_list:
        if id == obj.id:
            return index
        index += 1
    return -1
    