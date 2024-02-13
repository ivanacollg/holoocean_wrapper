#!/usr/bin/env python3 

import cv2
import cv_bridge
import rospy
import holoocean
import numpy as np
from pynput import keyboard
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Imu, Image
from bar30_depth.msg import Depth
from sonar_oculus.msg import OculusPing

from pid import Control
from utils import generate_map, parse_keys
from bruce_slam.CFAR import CFAR

print(holoocean.util.get_holoocean_path())

depth_control = Control()
pressed_keys = list()
force = 25

def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))

def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

#scene = ""
#scene = "Dam-HoveringDualSonar" # Works now (Has a spot where it dosent work)
#scene = "OpenWater-HoveringDualSonar" # Sonar donsen't work
scene = "PierHarbor-HoveringDualSonar" #Works now (Has a spot where it dosen't work)
config = holoocean.packagemanager.get_scenario(scene)
depth_command = 0

plt.ion()
#fig,ax = plt.subplots(nrows=1, ncols =3, figsize = (15,5))
step = 0
xs = []
ys = []

bridge = cv_bridge.CvBridge()

# init a rosnode
rospy.init_node("holoocean_stereo")

imu_pub = rospy.Publisher("/vectornav/IMU",Imu,queue_size=200)
depth_pub = rospy.Publisher("/bar30/depth/raw",Depth,queue_size=200)
sonar_horizontal_pub = rospy.Publisher("/sonar_oculus_node/M750d/ping",OculusPing,queue_size=10)
sonar_vertical_pub = rospy.Publisher("/sonar_oculus_node/M1200d/ping",OculusPing,queue_size=10)
camera_pub = rospy.Publisher("/camera/image_raw",Image,queue_size=5)

with holoocean.make(scene) as env:
    #env.should_render_viewport(False)
    while True:
        if 'q' in pressed_keys:
            break
        command, capture = parse_keys(pressed_keys, force, depth_command)

        #send to holoocean
        env.act("auv0", command)
        state = env.tick()

        # update the timestamp based on the number of ticks
        stamp = rospy.Time.from_sec(step / 200) 
        step += 1

        if "HorizontalSonar" in state and "VerticalSonar" in state and "LeftCamera" in state:
            
            #if capture:
                #print("step"+str(step))

            if "HorizontalSonar" in state:
                #map_x, map_y = generate_map(config)
                img = np.array(state["HorizontalSonar"] * 255).astype(np.uint8)
                img = np.array(img * 255).astype(np.uint8)
                params = [cv2.IMWRITE_JPEG_QUALITY,80]
                status, compressed = cv2.imencode(".jpg",img,params) # compress sonar images
                sonar_msg = OculusPing() # package as a sonar ping message
                sonar_msg.ping.data = compressed.tobytes()
                sonar_msg.header.stamp = stamp
                sonar_horizontal_pub.publish(sonar_msg)
                #horizontal_sonar_img = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
            
                #detector = CFAR(40, 20, 0.1, None)
                #threshold = 85
                #peaks = detector.detect(img, "SOCA")
                #peaks &= img > threshold
                #peaks_r = cv2.remap(peaks, map_x, map_y, cv2.INTER_LINEAR)
                #locs = np.c_[np.nonzero(peaks_r)]
                #for loc in locs:
                #    cv2.circle(horizontal_sonar_img, (loc[1],loc[0]),5, (255), -1)

                #ax[0].imshow(horizontal_sonar_img)
                #fig.canvas.draw()
                #if capture:
                #cv2.imwrite("../images/"+str(step)+"horz.png", horizontal_sonar_img)
                #print("Horizontal Sonar")


            if "VerticalSonar" in state:
                #map_x, map_y = generate_map(config)
                img = np.array(state["VerticalSonar"] * 255).astype(np.uint8)
                params = [cv2.IMWRITE_JPEG_QUALITY,80]
                status, compressed = cv2.imencode(".jpg",img,params) # compress sonar images
                sonar_msg = OculusPing() # package as a sonar ping message
                sonar_msg.ping.data = compressed.tobytes()
                sonar_msg.header.stamp = stamp
                sonar_vertical_pub.publish(sonar_msg)
                
                #vertical_sonar_img = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

                #    detector = CFAR(40, 20, 0.1, None)
                #    threshold = 85
                #    peaks = detector.detect(img, "SOCA")
                #    peaks &= img > threshold
                #    peaks_r = cv2.remap(peaks, map_x, map_y, cv2.INTER_LINEAR)
                #    locs = np.c_[np.nonzero(peaks_r)]
                #    for loc in locs:
                #        cv2.circle(vertical_sonar_img, (loc[1],loc[0]),5, (255), -1)

                #    ax[1].imshow(vertical_sonar_img)
                #    fig.canvas.draw()
                #if capture:
                #cv2.imwrite("../images/"+str(step)+"vert.png", vertical_sonar_img)
                #print("Vert Sonar")

            if "LeftCamera" in state:
                img = state["LeftCamera"]#np.array(state["LeftCamera"]* 255).astype(np.uint8)
                img = img[:, :, 0:3]
                img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
                img_msg.header.stamp = stamp
                camera_pub.publish(img_msg)
                #pixels = state["LeftCamera"]
                #ax[2].imshow(pixels)
                #fig.canvas.draw()
                #if capture:
                #cv2.imwrite("../images/"+str(step)+"camera.png", pixels)
                #print("Camera")

            if "DVLSensor" in state:
                # package into a ROS message
                vel_x = state["DVLSensor"][0]
                vel_y = -state["DVLSensor"][1]
                vel_z  = state["DVLSensor"][2]
                #print("Vel: ", vel_x,vel_y,vel_z)

            if "IMUSensor" in state:
                pass

            if "PoseSensor" in state:
                # convert the pose sensor to an IMU message
                roll,pitch,yaw = Rotation.from_matrix(state["PoseSensor"][:3, :3]).as_euler("xyz")
                #x,y,z,w = Rotation.from_euler("xyz",[r+(np.pi/2),p,-y]).as_quat()

                # conver the post sensor to a depth message
                #depth_command = depth_control.control_depth(state["PoseSensor"][2][3],-1)
                x = state["PoseSensor"][0][3]
                y = state["PoseSensor"][1][3]
                z = state["PoseSensor"][2][3]
                #if capture:
                #np.save("../images/"+str(step)+"pose", np.array([x, y, z, roll, pitch, yaw]))
                #print("Pose: ", x,y,z, roll, pitch, yaw)
            #if capture:
            #    print("step"+str(step))
                
            #fig.canvas.flush_events()

#plt.ioff()

