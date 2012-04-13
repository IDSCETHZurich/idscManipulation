#!/usr/bin/env python
import roslib; roslib.load_manifest('durga_description')

import rospy
import tf
import actionlib

from pr2_controllers_msgs.msg import JointTrajectoryAction
from move_base_msgs.msg import MoveBaseAction

# care-o-bot includes
from cob_srvs.srv import *

class gazebo_services():

	def __init__(self):
		#arm
		self.arm_client = actionlib.SimpleActionClient('/arm_controller/joint_trajectory_action', JointTrajectoryAction)
		self.arm_init_srv = rospy.Service('/arm_controller/init', Trigger, self.arm_init_cb)
		self.arm_stop_srv = rospy.Service('/arm_controller/stop', Trigger, self.arm_stop_cb)
		self.arm_recover_srv = rospy.Service('/arm_controller/recover', Trigger, self.arm_recover_cb)
		self.arm_set_operation_mode_srv = rospy.Service('/arm_controller/set_operation_mode', SetOperationMode, self.arm_set_operation_mode_cb)
		self.arm_set_joint_stiffness_srv = rospy.Service('/arm_controller/set_joint_stiffness', SetJointStiffness, self.arm_set_joint_stiffness_cb)

		#arm left
		#self.arm_left_client = actionlib.SimpleActionClient('/arm_left_controller/joint_trajectory_action', JointTrajectoryAction)
		#self.arm_left_init_srv = rospy.Service('/arm_left_controller/init', Trigger, self.arm_left_init_cb)
		#self.arm_left_stop_srv = rospy.Service('/arm_left_controller/stop', Trigger, self.arm_left_stop_cb)
		#self.arm_left_recover_srv = rospy.Service('/arm_left_controller/recover', Trigger, self.arm_left_recover_cb)
		#self.arm_left_set_operation_mode_srv = rospy.Service('/arm_left_controller/set_operation_mode', SetOperationMode, self.arm_left_set_operation_mode_cb)
		
		#arm right
		#self.arm_right_client = actionlib.SimpleActionClient('/arm_right_controller/joint_trajectory_action', JointTrajectoryAction)
		#self.arm_right_init_srv = rospy.Service('/arm_right_controller/init', Trigger, self.arm_right_init_cb)
		#self.arm_right_stop_srv = rospy.Service('/arm_right_controller/stop', Trigger, self.arm_right_stop_cb)
		#self.arm_right_recover_srv = rospy.Service('/arm_right_controller/recover', Trigger, self.arm_right_recover_cb)
		#self.arm_right_set_operation_mode_srv = rospy.Service('/arm_right_controller/set_operation_mode', SetOperationMode, self.arm_right_set_operation_mode_cb)

		#sdh
		self.sdh_client = actionlib.SimpleActionClient('/sdh_controller/joint_trajectory_action', JointTrajectoryAction)
		self.sdh_init_srv = rospy.Service('/sdh_controller/init', Trigger, self.sdh_init_cb)
		self.sdh_stop_srv = rospy.Service('/sdh_controller/stop', Trigger, self.sdh_stop_cb)
		self.sdh_recover_srv = rospy.Service('/sdh_controller/recover', Trigger, self.sdh_recover_cb)
		self.sdh_set_operation_mode_srv = rospy.Service('/sdh_controller/set_operation_mode', SetOperationMode, self.sdh_set_operation_mode_cb)
		
		#sdh left
		#self.sdh_left_client = actionlib.SimpleActionClient('/sdh_left_controller/joint_trajectory_action', JointTrajectoryAction)
		#self.sdh_left_init_srv = rospy.Service('/sdh_left_controller/init', Trigger, self.sdh_left_init_cb)
		#self.sdh_left_stop_srv = rospy.Service('/sdh_left_controller/stop', Trigger, self.sdh_left_stop_cb)
		#self.sdh_left_recover_srv = rospy.Service('/sdh_left_controller/recover', Trigger, self.sdh_left_recover_cb)
		#self.sdh_left_set_operation_mode_srv = rospy.Service('/sdh_left_controller/set_operation_mode', SetOperationMode, self.sdh_left_set_operation_mode_cb)
		
		#sdh right
		#self.sdh_right_client = actionlib.SimpleActionClient('/sdh_right_controller/joint_trajectory_action', JointTrajectoryAction)
		#self.sdh_right_init_srv = rospy.Service('/sdh_right_controller/init', Trigger, self.sdh_right_init_cb)
		#self.sdh_right_stop_srv = rospy.Service('/sdh_right_controller/stop', Trigger, self.sdh_right_stop_cb)
		#self.sdh_right_recover_srv = rospy.Service('/sdh_right_controller/recover', Trigger, self.sdh_right_recover_cb)
		#self.sdh_right_set_operation_mode_srv = rospy.Service('/sdh_right_controller/set_operation_mode', SetOperationMode, self.sdh_right_set_operation_mode_cb)


	# arm
	def arm_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_stop_cb(self, req):
		self.arm_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def arm_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	def arm_set_joint_stiffness_cb(self, req):
		resp = SetJointStiffnessResponse()
		resp.success.data = True
		return resp
		
	'''# arm left
	def arm_left_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_left_stop_cb(self, req):
		self.arm_left_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def arm_left_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_left_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp
		
	# arm right
	def arm_right_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_right_stop_cb(self, req):
		self.arm_right_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def arm_right_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_right_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp'''

	# sdh
	def sdh_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_stop_cb(self, req):
		self.sdh_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def sdh_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp
		
	'''# sdh left
	def sdh_left_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_left_stop_cb(self, req):
		self.sdh_left_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def sdh_left_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_left_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp
		
	# sdh right
	def sdh_right_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_right_stop_cb(self, req):
		self.sdh_right_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def sdh_right_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_right_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp'''


if __name__ == "__main__":
   rospy.init_node('gazebo_services')
   gazebo_services()
   rospy.loginfo("gazebo_services running")
   rospy.spin()

