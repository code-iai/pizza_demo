import logging
logging.basicConfig(level=logging.DEBUG)
import rospy

# imports the service
from prac2cram.srv import Prac2Cram, CancelSim
# import the messages
from prac2cram.msg import Task, ActionCore, ActionRole

rospy.wait_for_service('prac2cram/cancel_sim', timeout=5)
try:
    prac2cramCancelSim = rospy.ServiceProxy('prac2cram/cancel_sim', CancelSim)
    response = prac2cramCancelSim()
    print {'status': response.status, 'result': response.result}
except rospy.ServiceException, e:
    print "Service call failed with the following error: %s" %e
