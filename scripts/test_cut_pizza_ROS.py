#!/usr/bin/env python

import logging
logging.basicConfig(level=logging.DEBUG)
import rospy

# imports the service
from prac2cram.srv import Prac2Cram, CancelSim
# import the messages
from prac2cram.msg import Task, ActionCore, ActionRole

def getROSActionCores(action_cores_RPC):
  action_cores_ROS = []
  for action_core_RPC in action_cores_RPC:
    action_core_ROS = ActionCore()
    action_core_ROS.action_core_name = action_core_RPC['action_core_name']
    action_core_ROS.action_roles = []
    for role_RPC in action_core_RPC['action_roles']:
      role_ROS = ActionRole(role_name=role_RPC['role_name'], role_value=role_RPC['role_value'])
      action_core_ROS.action_roles.append(role_ROS)
    action_cores_ROS.append(action_core_ROS)

  
  return action_cores_ROS

def getROSTasks(tasks_RPC):

  print 'received: %s' %tasks_RPC
  tasks_ROS = []
  for task in tasks_RPC:
    tasks_ROS.append(Task(action_cores = getROSActionCores(task['action_cores'])))
  print 'tasks_ROS: %s' %tasks_ROS
  return tasks_ROS

# for the generated plan strings
def getStringList(strings_ROS):
  strings_RPC = []
  for string in strings_ROS:
    strings_RPC.append(string)
  return strings_RPC


cut_a_slice = {'action_cores': [{'action_core_name': 'Cutting', 'action_roles': [{'role_name': 'utensil', 'role_value': 'knife'}, {'role_name': 'unit', 'role_value': 'slice'}, {'role_name': 'obj_to_be_cut', 'role_value': 'bread'}, {'role_name': 'action_verb', 'role_value': 'cut'}, {'role_name': 'amount', 'role_value': '4'}]}]}
tasks_ROS = getROSTasks(cut_a_slice)

rospy.wait_for_service('prac2cram', timeout=5) # in seconds

try:
    # create a handle to the service
    prac2cram = rospy.ServiceProxy('prac2cram', Prac2Cram)

    #h = std_msgs.msg.Header()
    #h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

    # simplified style
    response = prac2cram(tasks_ROS)

    # formal style
    #resp2 = prac2cram.call(Prac2CramRequest(params))

    print {'status': response.status, 'messages': getStringList(response.messages), 'plan_strings': getStringList(response.plan_strings)}

except rospy.ServiceException, e:
    print "Service call failed with the following error: %s" %e

