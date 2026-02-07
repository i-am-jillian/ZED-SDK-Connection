#Capture stereo images from zed cam
#returns frames as numpy arrays
#splits left/right image

import pyzed.sl as sl #ZED SDK API
import cv2 
import numpy as np 

zed = sl.Camera() #zed camera

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720 
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.METER #depth val returned in meters

#opening the camera
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    raise RuntimeError("Failed to open ZED")

image = sl.Mat()
depth = sl.Mat()
runtime = sl.RuntimeParameters()

while True:
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH, sl.MEM.CPU)

        img_np = image.get_data()          # (H, W, 4) RGBA
        depth_np = depth.get_data()        # (H, W) float32 meters

        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)
        cv2.imshow("ZED Left", img_bgr)

        cy, cx = depth_np.shape[0]//2, depth_np.shape[1]//2
        print("Center depth (m):", depth_np[cy, cx])

    if cv2.waitKey(1) & 0xFF in (27, ord('q')):
        break

zed.close()
cv2.destroyAllWindows()
