#!/usr/bin/env python3 

import cv2
import cv_bridge
import rospy
import holoocean
import numpy as np
from pynput import keyboard
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

import tf
from sensor_msgs.msg import Imu, Image
from bar30_depth.msg import Depth
from sonar_oculus.msg import OculusPing
from nav_msgs.msg import Odometry

from pid import Control
from utils import generate_map, parse_keys
from bruce_slam.CFAR import CFAR

print(holoocean.util.get_holoocean_path())

depth_control = Control()
pressed_keys = list()
force = 15

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
odom_pub = rospy.Publisher("localization/odom", Odometry, queue_size=10)
tf = tf.TransformBroadcaster()

rate = rospy.Rate(100)

with holoocean.make(scene) as env:
    #env.should_render_viewport(False)
    while not rospy.is_shutdown():
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
                imgh = np.array(state["HorizontalSonar"])
                #img = img ** 3. # you can apply alpha to images here
                imgh = imgh / np.max(imgh)
                imgh = np.array(imgh * 255).astype(np.uint8)
                params = [cv2.IMWRITE_JPEG_QUALITY,80]
                status, compressed = cv2.imencode(".jpg",imgh,params) # compress sonar images
                sonar_msg = OculusPing() # package as a sonar ping message
                sonar_msg.ping.data = compressed.tobytes()
                sonar_msg.header.stamp = stamp
                sonar_horizontal_pub.publish(sonar_msg)


            if "VerticalSonar" in state:
                #map_x, map_y = generate_map(config)
                imgv = np.array(state["VerticalSonar"])
                imgv = imgv / np.max(imgv)
                imgv = np.array(imgv * 255).astype(np.uint8)
                params = [cv2.IMWRITE_JPEG_QUALITY,80]
                status, compressed = cv2.imencode(".jpg",imgv,params) # compress sonar images
                sonar_msg = OculusPing() # package as a sonar ping message
                sonar_msg.ping.data = compressed.tobytes()
                sonar_msg.header.stamp = stamp
                sonar_vertical_pub.publish(sonar_msg)
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


            if "IMUSensor" in state:
                pass

            if "PoseSensor" in state:
                # convert the pose sensor to an IMU message
                roll,pitch,yaw = Rotation.from_matrix(state["PoseSensor"][:3, :3]).as_euler("xyz")
                qx,qy,qz,qw = Rotation.from_euler("xyz",[roll+(np.pi/2),pitch,-yaw]).as_quat()

                # conver the post sensor to a depth message
                #depth_command = depth_control.control_depth(state["PoseSensor"][2][3],-1)
                x = state["PoseSensor"][0][3]#-520
                y = state["PoseSensor"][1][3]#+646
                z = state["PoseSensor"][2][3]#+12

                imu_msg = Imu()
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw
                imu_msg.header.stamp = stamp
                imu_pub.publish(imu_msg)

                header = rospy.Header()
                header.stamp = stamp
                header.frame_id = "map"

                odom_msg = Odometry()
                odom_msg.header = header
		        # pose in odom frame
                odom_msg.pose.pose.position.x = x#-520
                odom_msg.pose.pose.position.y = y#+646
                odom_msg.pose.pose.position.z = z#+12
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw

		        # twist in local frame
                odom_msg.child_frame_id = "base_link"
		        # Local planer behaves worse
		        # odom_msg.twist.twist.linear.x = self.prev_vel[0]
		        # odom_msg.twist.twist.linear.y = self.prev_vel[1]
		        # odom_msg.twist.twist.linear.z = self.prev_vel[2]
		        # odom_msg.twist.twist.angular.x = self.prev_omega[0]
		        # odom_msg.twist.twist.angular.y = self.prev_omega[1]
		        # odom_msg.twist.twist.angular.z = self.prev_omega[2]
                odom_msg.twist.twist.linear.x = 0
                odom_msg.twist.twist.linear.y = 0
                odom_msg.twist.twist.linear.z = 0
                odom_msg.twist.twist.angular.x = 0
                odom_msg.twist.twist.angular.y = 0
                odom_msg.twist.twist.angular.z = 0
                odom_pub.publish(odom_msg)   

                p = odom_msg.pose.pose.position
                q = odom_msg.pose.pose.orientation
		        # send a transform for the submapping system
                tf.sendTransform(
		        	(x,y,z), (qx, qy, qz, qw), header.stamp, "base_link", "map"
		        )

        rate.sleep()

            #if capture:
            #np.save("../images/"+str(step)+"pose", np.array([x, y, z, roll, pitch, yaw]))
            #print("Pose: ", x,y,z, roll, pitch, yaw)
        #if capture:
        #    print("step"+str(step))
            
        #fig.canvas.flush_events()

#plt.ioff()

