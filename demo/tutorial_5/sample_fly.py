from visual_guidance import visual_guidance
from qr_code import QRdetect

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from commander import Commander
import rospy

import cv2

#vg = visual_guidance("calibration_board.png")

image = cv2.imread("1.png")
qr = QRdetect(image)

cvbridge = CvBridge()

def image_callback(image):

    cv_image = None

    try:
        cv_image = cvbridge.imgmsg_to_cv2(image, "bgr8")

    except CvBridgeError as e:
        print(e)

    # if cv_image is not None:
    #     rotation, translation = vg.match_image(cv_image)
    #
    #     if rotation and translation:
    #         print("Found Target !")
    #         print("Rotation is: ", rotation)
    #         print("Translation is: ", translation)

    if cv_image is not None:
        rotation, translation = qr.process_image(cv_image)

        print("Found Target !")
        print("Rotation is: ", rotation)
        print("Translation is: ", translation)

if __name__ == '__main__':

    rospy.init_node('image_converter', anonymous=True)

    image_sub = rospy.Subscriber("/gi/simulation/left/image_raw", Image, image_callback)

    rospy.spin()
