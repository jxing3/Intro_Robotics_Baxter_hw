#!/usr/bin/env python

import math
import numpy
from threading import Thread, Lock

import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
import rospy
from sensor_msgs.msg import JointState
import tf
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker


# This is the function that must be filled in as part of Homework 2.
# For a description of the expected code, and detailed information on the
# function parameters, see the class handout for Lecture 8.
def cartesian_control(joint_transforms, 
                      q_current, q0_desired,
                      b_T_ee_current, b_T_ee_desired):
    num_joints = len(joint_transforms)
    
    v_ee = calc_vee(b_T_ee_current, b_T_ee_desired)

    inv_b_T_ee = numpy.linalg.inv(b_T_ee_current)

    V_tot = []

    for i in range(0,len(joint_transforms)):

	curr_transform = joint_transforms[i]
        curr_transform = numpy.dot(inv_b_T_ee,curr_transform)
	rot = curr_transform[0:3,0:3]
	s = calc_s(curr_transform)
	rot_s=numpy.dot(-1*rot,s)
	V = numpy.vstack((rot_s,rot))
	V_tot.append(V[:,2])

    J = numpy.matrix(V_tot).T
    J_p = calc_pinv(J, True) #numpy.linalg.pinv(J,1e-2)
    J_p_nopro = calc_pinv(J,False) #numpy.linalg.pinv(J)

    dq= numpy.dot(J_p, v_ee).getA1()

    if(max(dq)>1):
    	dq = dq/max(dq)*1

    dq = numpy.matrix(dq)
    q_sec = numpy.zeros(num_joints)
    q_sec[0] = 40*(q0_desired - q_current[0])

    inner = numpy.identity(num_joints)-numpy.dot(J_p_nopro,J)

    q_null = numpy.dot(inner, q_sec.T)
    q_null = q_null.getA1()

    if(max(q_null)>1):
    	q_null = q_null/max(q_null)*1

    q_null = numpy.matrix(q_null)

    dq = dq + q_null
    dq = dq.getA1()



    return dq

# Implementation of my own pinv function
def calc_pinv(matrix, thres):

    U, s, V  = numpy.linalg.svd(matrix, full_matrices=False)

    U_t = U.T
    V_t = V.T

    s1 = s

    numrows = len(s1)
    
    for i in range(0,numrows):
	if(thres and (s1[i]<1e-2)):
		s1[i] = 0
		continue
	s1[i] = 1/s1[i]

    pinv = numpy.dot(numpy.dot(V_t, numpy.diag(s1)),U_t)
    return pinv

def calc_s(curr_transform):
    trans = numpy.linalg.inv(curr_transform)[0:3,3]
    return [[0, -trans[2], trans[1]],[trans[2], 0, -trans[0]], [-trans[1], trans[0], 0]]


def calc_vee(b_T_ee_current, b_T_ee_desired):

    #matrix = numpy.dot(b_T_ee_desired,numpy.linalg.inv(b_T_ee_current))

    #rot = matrix[0:3,0:3]
    #trans = matrix[0:3,3]
    #rot = rotation_from_matrix(rot)
    #fin_rot = rot[0]*rot[1]
    rot_1 = b_T_ee_current[0:3,0:3]
    rot_2 = b_T_ee_desired[0:3,0:3]
    trans1 = b_T_ee_current[0:3,3]
    trans2 = b_T_ee_desired[0:3,3]
    
    
    #rot = numpy.dot(rot_2,rot_1.T)

    rot_1 = rotation_from_matrix(rot_1)
    rot_2 = rotation_from_matrix(rot_2)

    fin_rot1 = rot_1[0]*rot_1[1]
    fin_rot2 = rot_2[0]*rot_2[1]

    delt_x_trans = trans2 - trans1
    delt_x_rot = fin_rot2 - fin_rot1
    #fin_trans = trans2-trans1
    #cur = numpy.append(trans1, fin_rot1)
    #des = numpy.append(trans2, fin_rot2)

   # delt_x = numpy.append(trans, fin_rot)
    v_ee = numpy.append(delt_x_trans*5, delt_x_rot*7)
   # v_ee = delt_x * gain
    v_trans = v_ee[0:3]
    v_rot = v_ee[3:]

    length_trans = numpy.linalg.norm(v_trans)
    length_rot = numpy.linalg.norm(v_rot)
    if (length_trans > .1):
        v_trans = v_trans/ length_trans * .1
    if(length_rot > 1):
	v_rot = v_rot/length_rot *1
    v_ee = numpy.append(v_trans,v_rot)
    return v_ee
    
def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

class CartesianControl(object):

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()

        #Create "Interactive Marker" that we can manipulate in RViz
        self.init_marker()
        self.ee_tracking = 0
        self.red_tracking = 0
        self.q_current = []

        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0

        self.mutex = Lock()
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def init_marker(self):

        self.server = InteractiveMarkerServer("control_markers")

        control_marker = InteractiveMarker()
        control_marker.header.frame_id = "/world_link"
        control_marker.name = "cc_marker"

        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.orientation.w = 1
        move_control.orientation.x = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_y"
        move_control.orientation.w = 1
        move_control.orientation.y = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_z"
        move_control.orientation.w = 1
        move_control.orientation.z = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)

        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_x"
        move_control.orientation.w = 1
        move_control.orientation.x = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_y"
        move_control.orientation.w = 1
        move_control.orientation.z = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_z"
        move_control.orientation.w = 1
        move_control.orientation.y = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)

        control_marker.scale = 0.25
        self.server.insert(control_marker, self.control_marker_feedback)

        redundancy_marker = InteractiveMarker()
        redundancy_marker.header.frame_id = "/lwr_arm_1_link"
        redundancy_marker.name = "red_marker"
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_z"
        rotate_control.orientation.w = 1
        rotate_control.orientation.y = 1
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        redundancy_marker.controls.append(rotate_control)
        redundancy_marker.scale = 0.25
        self.server.insert(redundancy_marker, self.redundancy_marker_feedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def update_ee_marker(self, T):
        # Note that we keep marker orientation fixed so that robot can be taken out of
        # singularity even when only doing translation only control
        # Otherwise, arrows always point along arm, and arm stays stretched out
        # forever if it starts out that way.
        Ttrans = tf.transformations.translation_matrix(
            tf.transformations.translation_from_matrix(T)
        )
        self.server.setPose("cc_marker", convert_to_message(Ttrans))
        self.server.applyChanges()

    def update_red_marker(self,q0):
        Trot = tf.transformations.rotation_matrix(q0,(0,0,1))
        self.server.setPose("red_marker", convert_to_message(Trot))
        self.server.applyChanges()

    def ring_angle(self, feedback):
        q = feedback.pose.orientation
        qvec = ((q.x, q.y, q.z, q.w))
        R = tf.transformations.quaternion_matrix(qvec)
        angle, direction, point = tf.transformations.rotation_from_matrix(R)
        return angle

    def redundancy_marker_feedback(self, feedback):       
        if feedback.event_type == feedback.MOUSE_DOWN:
            self.x_target = self.x_current
            self.red_tracking = 1
        elif feedback.event_type == feedback.MOUSE_UP:
            self.q0_desired = self.q_current[0]
            self.red_tracking = 0
        if feedback.event_type == feedback.POSE_UPDATE:
            angle = self.ring_angle(feedback)
            self.mutex.acquire()
            if abs(self.q0_desired - angle) < 1.0: self.q0_desired = angle
            self.mutex.release()

    def control_marker_feedback(self, feedback):
        if feedback.event_type == feedback.MOUSE_DOWN:
            # since default marker orientation stays fixed, change in orientation
            # must be applied relative to reference orientation when we started dragging
            self.R_base = self.x_current
            self.R_base[0:3,3]=0
            self.ee_tracking = 1
        elif feedback.event_type == feedback.MOUSE_UP:
            self.ee_tracking = 0
            self.x_target = self.x_current
        elif feedback.event_type == feedback.POSE_UPDATE:
            self.mutex.acquire()
            R = tf.transformations.quaternion_matrix((feedback.pose.orientation.x,
                                                      feedback.pose.orientation.y,
                                                      feedback.pose.orientation.z,
                                                      feedback.pose.orientation.w))
            T = tf.transformations.translation_matrix((feedback.pose.position.x, 
                                                       feedback.pose.position.y, 
                                                       feedback.pose.position.z))
            self.x_target = numpy.dot( T,numpy.dot(R,self.R_base) )
            self.mutex.release()

    def timer_callback(self, event):
        msg = JointState()
        if self.ee_tracking or self.red_tracking:
            self.mutex.acquire()
            dq = cartesian_control(self.joint_transforms, 
                                   self.q_current, self.q0_desired,
                                   self.x_current, self.x_target)
            self.mutex.release()
            msg.velocity = dq
        else:            
            msg.velocity = numpy.zeros(7)
        self.pub_vel.publish(msg)
        
    def callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        if not self.ee_tracking:
            self.update_ee_marker(self.x_current)
        if not self.red_tracking:
            self.update_red_marker(self.q_current[0])
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0,0,1])
        x = numpy.array([1,0,0])
        dot = numpy.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map: 
            self.x_current = T
            return
        for i in range(0,len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]        

            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerror("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)
        

if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()