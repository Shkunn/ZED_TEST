import pyzed.sl as sl
import cv2
import pangolin
import OpenGL.GL as gl
import time
import numpy as np
import argparse

def pangolin_draw_coordinate():
    """
        DESCRIPTION: this function draw the world origine of the map.
    """
    gl.glLineWidth(3)
    gl.glColor3f(0, 0.0, 0.0)
    pangolin.DrawLines(
        [[0,0,0]], 
        [[1,0,0]], 3)   
    gl.glColor3f(0, 255, 0)
    pangolin.DrawLines(
        [[0,0,0]], 
        [[0,1,0]], 3)
    gl.glColor3f(0, 0, 255)
    pangolin.DrawLines(
        [[0,0,0]], 
        [[0,0,1]], 3)

def pangolin_init(w, h):
    """
        DESCRIPTION: init pangolin.
    """
    W, H = w, h
    pangolin.CreateWindowAndBind('Main', W, H)
    gl.glEnable(gl.GL_DEPTH_TEST)

    # Define Projection and initial ModelView matrix
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(W, H, 420, 420, W // 2, H // 2, 0.2, 100),
        pangolin.ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin.AxisDirection.AxisY))
    handler = pangolin.Handler3D(scam)

    # Create Interactive View in window
    dcam = pangolin.CreateDisplay()
    dcam.SetBounds(0.0, 1.0, 0.0, 1.0, W/H)
    dcam.SetHandler(handler)

    # Create area to show video stream
    dimg = pangolin.Display('image')
    dimg.SetBounds(1.0, 0.66, 0, 0.33, W/H) # (debut hauteur, fin hauteur, debut largeur, fin largeur, ratio)
    dimg.SetLock(pangolin.Lock.LockLeft, pangolin.Lock.LockTop)

    return scam, dcam, dimg

def showing_mapping():
    """
        DESCRIPTION: Run a 3D mapping section and get live result.
            * first - only position
    """
    # ARG.
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    args = parser.parse_args()

    # PANGOLIN CONFIGURATION & INITIALISATION.
    w, h = 1280, 720
    scam, dcam, dimg = pangolin_init(w, h)
    texture = pangolin.GlTexture(
        w, h, gl.GL_RGB, False, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE,
    )

    # ZED CONFIGURATION CAMERA.
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.coordinate_units = sl.UNIT.METER         # Set coordinate units
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
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
    """spatial_mapping_parameters = sl.SpatialMappingParameters(resolution=sl.MAPPING_RESOLUTION.LOW, use_chunk_only = True, mapping_range=sl.MAPPING_RANGE.LONG, max_memory_usage = 4096*8)
    spatial_mapping_parameters.range_meter = spatial_mapping_parameters.get_range_preset(sl.MAPPING_RANGE.LONG)
    spatial_mapping_parameters.resolution_meter = 0.1"""

    mapping_parameters = sl.SpatialMappingParameters(resolution=sl.MAPPING_RESOLUTION.HIGH, use_chunk_only = True, mapping_range=sl.MAPPING_RANGE.MEDIUM, max_memory_usage = 4096*8)
    mapping_parameters.range_meter = mapping_parameters.get_range_preset(sl.MAPPING_RANGE.LONG)
    mapping_parameters.resolution_meter = 0.02

    """spatial_mapping_parameters.set_resolution()
    spatial_mapping_parameters.use_chunk_only = True
    spatial_mapping_parameters.save_texture = False         # Set to True to apply texture over the created mesh
    spatial_mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.MESH
    #spatial_mapping_parameters.set_range = sl.MAPPING_RANGE.LONG"""
    zed.enable_spatial_mapping(mapping_parameters)

    # TIME PARAMETERS.
    last_call = time.time()
    runtime = sl.RuntimeParameters()

    # CLEAR FOR SAFETY?
    pymesh.clear()

    # i FOR FILTER.
    i = 0
    zed.enable_spatial_mapping()


    while not pangolin.ShouldQuit():
        # -clear all.
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)
        dcam.Activate(scam)

        # -draw coordinate origine.
        pangolin_draw_coordinate()

        # -get image.
        zed.grab(runtime)
        zed.retrieve_image(image, sl.VIEW.LEFT)

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

        # DRAW ALL STATE.
        gl.glLineWidth(1)
        gl.glColor3f(0.0, 0.0, 1.0)

        # -transform brute data in numpy to draw pose.
        a = pose.pose_data()
        pose2 = np.array([[a[0,0],a[0,1],a[0,2],a[0,3]],
                          [a[1,0],a[1,1],a[1,2],a[1,3]],
                          [a[2,0],a[2,1],a[2,2],a[2,3]],
                          [a[3,0],a[3,1],a[3,2],a[3,3]]])
        pangolin.DrawCamera(pose2, 0.5, 0.75, 0.8)

        # -draw all points on the map.
        gl.glPointSize(2)
        gl.glColor3f(1.0, 0.0, 0.0)
        pangolin.DrawPoints(pymesh.vertices)    

        # DISPLAY CAMERA VIDEO.
        img = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2RGB)
        img = cv2.flip(img, 0)
        texture.Upload(img, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
        dimg.Activate()
        gl.glColor3f(1.0, 1.0, 1.0)
        texture.RenderToViewport()

        # FILTER INTERACTION.
        if False:
            filter_params = sl.MeshFilterParameters()
            filter_params.set(sl.MESH_FILTER.LOW)
            pymesh.flter(filter_params)
            i = 0
            print("filter")
        i+=1

        #print(pose.get_euler_angles.get())
        # SHOW INFORMATION.
        #print("Number of vertices : {0}, Info 2 : {1}\n".format(pymesh.vertices.shape[0], spatial_mapping_parameters.set_range()))
        # In background, spatial mapping will use new images, depth and pose to create and update the mesh. No specific functions are required here.
        mapping_state = zed.get_spatial_mapping_state()
        # check param.
        mapping_param = zed.get_spatial_mapping_parameters()
        # Print spatial mapping state
        print("\rImages captured: {0} / 100 || {1} || {2} || {3}".format(i, mapping_state, mapping_param.resolution_meter, pymesh.vertices.shape[0]))

        # END OF CYCLE.
        pangolin.FinishFrame()
  
    # NOW SAVE THE MAP________________________________________.
    zed.extract_whole_spatial_map(pymesh)

    # FILTER__________________________________________________.
    """filter_params = sl.MeshFilterParameters()
    filter_params.set(sl.MESH_FILTER.MEDIUM) 
    pymesh.filter(filter_params, True)"""

    # SAVE AND CLEAN ALL PROCESS______________________________.
    filepath = "/home/thomas/Documents/rane_slam/mk3slam.1.0/data/"
    status = pymesh.save("{0}{1}.obj".format(filepath, args.path))
    status2 = zed.save_area_map("{0}{1}.area".format(filepath, args.path))

    if status:
        print("Mesh saved under " + filepath + args.path + ".obj")
    else:
        print("Failed to save the mesh under " + filepath + args.path + ".obj")

    if status2:
        print("Area saved under " + filepath + args.path + ".area")
    else:
        print("Failed to save the area under " + filepath + args.path + ".area")
    
    #sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()

if __name__=="__main__":
    showing_mapping()