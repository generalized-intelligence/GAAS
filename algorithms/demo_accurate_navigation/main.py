

import rospy
from sensor_msgs.msg import Image
#import service....


pSceneRequest = None
image_current = None


def get_to_approximate_target_by_gps(target):
    pass#TODO

def get_to_approximate_target(target):
    if (gps_signal_valid):
        return get_to_approximate_target_by_gps(target)
    else:
        return False
    

def _parse_RT(RT):
    pass#TODO

def request_accurate_pos_relative_to_target(target):
    if image_current isinstance(Image):
        result = get_accurate_pos_of_target(image_current)
        success,RT = result
        if success:
            target_heading_angle,target_translation = _parse_RT(RT)
            mav_move_to(target_translation,target_heading_angle)
    else:
        pass

def get_accurate_pos_of_target(image_in):
    result_of_matching = pSceneRequest(image_in)
    success,RT = result_of_matching
    if success:
        return True,RT
    else:
        return False,None




if __name__ == '__main__':
    get_to_approximate_target()



