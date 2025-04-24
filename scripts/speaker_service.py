#!/usr/bin/env python3

import rospy
import random

from tiago_sar_cogarch.srv import Speaker, SpeakerResponse

def speaker_callback(req):
    rospy.loginfo(f"Received a message: {req.message}")
    
    num = random.randint(1, 100)
    
    if(num < 10):
        rospy.loginfo("speaker failed")
        success = False
    else:
        # Reproduce the message
        success = True
    rospy.sleep(5)

    return SpeakerResponse(success)

def speaker_service():
    rospy.init_node('speaker_service')
    service = rospy.Service('/speaker', Speaker, speaker_callback)
    rospy.loginfo("Speaker service is ready!")
    rospy.spin()

if __name__ == '__main__':
    speaker_service()
