#Capture stereo images from zed cam
#returns frames as numpy arrays
#splits left/right image

import pyzed.sl as sl #ZED SDK API
import cv2 #open cv
import numpy as np 

zed = sl.Camera() #camera set up

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720 
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.METER #depth val returned in meters

#connecting to the zed, crashes if cant open
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    raise RuntimeError("Failed to open ZED")

runtime = sl.RuntimeParameters() #default per frame settings

image_left = sl.Mat() #containers for the image frames
depth = sl.Mat() #container for depth map


while True:
    #grab new frames
    if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
        continue
    zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU) #gets left camera image, depth map and stores
    zed.retrieve_measure(depth, sl.MEASURE.DEPTH, sl.MEM.CPU) 

    #converting to nunpy
    left_np = image_left.get_data()          # (H, W, 4) RGBA
    depth_np = depth.get_data()        # (H, W) float32 meters

    img_bgr = cv2.cvtColor(left_np, cv2.COLOR_BGRA2BGR) #convert img to opencv
    cv2.imshow("ZED Left", img_bgr)

    print("LEFT IMAGE:")
    print("  shape:", left_np.shape)
    print("  dtype:", left_np.dtype)
    print("  sample pixel [0,0]:", left_np[0, 0])

    print("DEPTH MAP:")
    print("  shape:", depth_np.shape)
    print("  dtype:", depth_np.dtype)
    print("  center depth (m):", depth_np[depth_np.shape[0]//2, depth_np.shape[1]//2])
    print("-" * 40)

    if cv2.waitKey(1) & 0xFF in (27, ord('q')): #camera stops if q key pressed
        break

zed.close()
cv2.destroyAllWindows()
