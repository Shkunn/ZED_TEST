"""
    This sample shows how to capture a real-time 3D reconstruction      
    of the scene using the Spatial Mapping API. The resulting mesh      
    is displayed as a wireframe on top of the left image using OpenGL.  
    Spatial Mapping can be started and stopped with the Space Bar key
"""
import cv2
import sys
import time
import numpy as np
import pyzed.sl as sl
# import ogl_viewer.viewer as gl

def main():

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

    mapping_parameters = sl.SpatialMappingParameters(resolution=sl.MAPPING_RESOLUTION.HIGH, use_chunk_only = True, mapping_range=sl.MAPPING_RANGE.MEDIUM, max_memory_usage = 4096*8)
    mapping_parameters.range_meter = mapping_parameters.get_range_preset(sl.MAPPING_RANGE.LONG)
    mapping_parameters.resolution_meter = 0.02

    zed.enable_spatial_mapping(mapping_parameters)

    # TIME PARAMETERS.
    last_call = time.time()
    runtime = sl.RuntimeParameters()

    # CLEAR FOR SAFETY?
    pymesh.clear()


    zed.enable_spatial_mapping()

    key = ''
    while key != 113:
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

        a = pose.pose_data()
        pose2 = np.array([[a[0,0],a[0,1],a[0,2],a[0,3]],
                          [a[1,0],a[1,1],a[1,2],a[1,3]],
                          [a[2,0],a[2,1],a[2,2],a[2,3]],
                          [a[3,0],a[3,1],a[3,2],a[3,3]]])

        cv2.imshow("ZED", image.get_data())
        key = cv2.waitKey(5)
        
        # In background, spatial mapping will use new images, depth and pose to create and update the mesh. No specific functions are required here.
        mapping_state = zed.get_spatial_mapping_state()
        # check param.
        mapping_param = zed.get_spatial_mapping_parameters()
        # Print spatial mapping state
        print("\rImages captured: {0} || {1} || {2} || {3}".format(mapping_state, mapping_param.resolution_meter, pymesh.vertices.shape[0], pymesh.vertices))

    cv2.destroyAllWindows()

    image.free(memory_type=sl.MEM.CPU)
    # Disable modules and close camera
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()

#region VIEWER
    # while viewer.is_available():
    #     # Grab an image, a RuntimeParameters object must be given to grab()
    #     if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
    #         # Retrieve left image
    #         zed.retrieve_image(image, sl.VIEW.LEFT)
    #         # Update pose data (used for projection of the mesh over the current image)
    #         tracking_state = zed.get_position(pose)

    #         if mapping_activated:
    #             mapping_state = zed.get_spatial_mapping_state()
    #             # Compute elapsed time since the last call of Camera.request_spatial_map_async()
    #             duration = time.time() - last_call  
    #             # Ask for a mesh update if 500ms elapsed since last request
    #             if(duration > .5 and viewer.chunks_updated()):
    #                 zed.request_spatial_map_async()
    #                 last_call = time.time()
                
    #             if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
    #                 zed.retrieve_spatial_map_async(pymesh)
    #                 viewer.update_chunks()
                
    #         change_state = viewer.update_view(image, pose.pose_data(), tracking_state, mapping_state)

    #         if change_state:
    #             if not mapping_activated:
    #                 init_pose = sl.Transform()
    #                 zed.reset_positional_tracking(init_pose)

    #                 # Configure spatial mapping parameters
    #                 spatial_mapping_parameters.resolution_meter = sl.SpatialMappingParameters().get_resolution_preset(sl.MAPPING_RESOLUTION.MEDIUM)
    #                 spatial_mapping_parameters.use_chunk_only = True
    #                 spatial_mapping_parameters.save_texture = False         # Set to True to apply texture over the created mesh
    #                 spatial_mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.MESH

    #                 # Enable spatial mapping
    #                 zed.enable_spatial_mapping()

    #                 # Clear previous mesh data
    #                 pymesh.clear()
    #                 viewer.clear_current_mesh()

    #                 # Start timer
    #                 last_call = time.time()

    #                 mapping_activated = True
    #             else:
    #                 # Extract whole mesh
    #                 zed.extract_whole_spatial_map(pymesh)

    #                 filter_params = sl.MeshFilterParameters()
    #                 filter_params.set(sl.MESH_FILTER.MEDIUM) 
    #                 # Filter the extracted mesh
    #                 pymesh.filter(filter_params, True)
    #                 viewer.clear_current_mesh()

    #                 # If textures have been saved during spatial mapping, apply them to the mesh
    #                 if(spatial_mapping_parameters.save_texture):
    #                     print("Save texture set to : {}".format(spatial_mapping_parameters.save_texture))
    #                     pymesh.apply_texture(sl.MESH_TEXTURE_FORMAT.RGBA)

    #                 # Save mesh as an obj file
    #                 filepath = "mesh_gen.obj"
    #                 status = pymesh.save(filepath)
    #                 if status:
    #                     print("Mesh saved under " + filepath)
    #                 else:
    #                     print("Failed to save the mesh under " + filepath)
                    
    #                 mapping_state = sl.SPATIAL_MAPPING_STATE.NOT_ENABLED
    #                 mapping_activated = False
    
    # image.free(memory_type=sl.MEM.CPU)
    # pymesh.clear()
    # # Disable modules and close camera
    # zed.disable_spatial_mapping()
    # zed.disable_positional_tracking()
    # zed.close()
#endregion
    
if __name__ == "__main__":
    main()
