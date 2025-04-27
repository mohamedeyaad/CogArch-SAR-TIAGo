#!/usr/bin/env python3
""" 
Speaker Service Node.

This script defines a ROS service node that simulates a speaker's behavior. 

It listens for service requests containing a message and randomly decides if the speaker successfully reproduces the message.

"""

import rospy
import random

from tiago_sar_cogarch.srv import Speaker, SpeakerResponse

def speaker_callback(req):
    """
    Callback function for the speaker service.

    This function is triggered when a service request is received.
    It logs the received message, randomly determines whether the speaker
    successfully reproduces the message, simulates a processing delay,
    and returns the success status.

    :param req: The service request containing the message to be reproduced.
    :type req: tiago_sar_cogarch.srv.SpeakerRequest
    :return: The response indicating whether the speaker successfully reproduced the message.
    :rtype: tiago_sar_cogarch.srv.SpeakerResponse
    """
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
    """
    Initializes and runs the speaker service node.

    This function initializes the ROS node, advertises the '/speaker' service,
    and keeps the node running to handle incoming service requests.
    """
    rospy.init_node('speaker_service')
    service = rospy.Service('/speaker', Speaker, speaker_callback)
    rospy.loginfo("Speaker service is ready!")
    rospy.spin()

if __name__ == '__main__':
    """
    Main execution block.

    If this script is run as the main program, it will start the speaker service node.
    """
    speaker_service()

