from visual_guidance import visual_guidance
from qr_code import QRdetect
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from commander_2 import Commander
import rospy
import cv2
import time
import numpy as np

#vg = visual_guidance("target.png")
image = cv2.imread("target.png")

qr = QRdetect(image)
cvbridge = CvBridge()

local_pose = None
suitable_translation = None
suitable_normal = None
scale = None
idx = None
found_target = True
com = Commander()

def image_callback(image):
    global suitable_translation, suitable_normal, scale, idx, found_target
    cv_image = None

    try:
        cv_image = cvbridge.imgmsg_to_cv2(image, "bgr8")

    except CvBridgeError as e:
        print(e)

    if cv_image is not None:

        rotations, translations, normals = qr.process_image(cv_image)

        if rotations is None:
            print("Target not detected!")
            return

        for translation in translations:
            print("translation: ", translation)

        # Here we only use translation and neglect the rotation.
        for idxs, normal in enumerate(normals):
            print("normal: ", normal)

            # target normal should be (0, -1, 0) as it is on the ground and drone camera is facing forward
            if normals[idxs][1] + 1 < 0.05:
                suitable_normal = normals[idxs]
                idx = idxs

        suitable_translation = translations[idx]
        print("suitable normal is: ", suitable_normal)
        print("suitable translation is: ", suitable_translation)

        '''
        movement wrt camera frame is Right(X) Down(Y) Forward(Z)
        '''
        # scale = np.abs((local_pose.z - 1.5) / suitable_translation[1])
        scale = np.abs((local_pose.z-1) / suitable_translation[1])
        pending_movement = [0, 0, 0]
        pending_movement[0] = suitable_translation[2] * scale
        pending_movement[1] = - suitable_translation[0] * scale

        # first move in the horizontally and then land
        if(found_target):
            print("current height is: ", local_pose.z)
            print("current scale is: ", scale)
            print("Moving in body(FLU) frame: ", (pending_movement[0], pending_movement[1], - local_pose.z))

            com.move(pending_movement[0], pending_movement[1], pending_movement[2])
            time.sleep(10)
            com.move(0, 0, - local_pose.z)
            found_target = False

def pose_callback(data):
    global local_pose
    local_pose = data.pose.position


if __name__ == '__main__':

    image_sub = rospy.Subscriber("/gi/simulation/left/image_raw", Image, image_callback)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    rospy.spin()
