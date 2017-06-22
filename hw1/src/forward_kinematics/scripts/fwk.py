#!/usr/bin/env python

import numpy

import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

# Jesse Xing HW1 Introduction to Robotics

""" Starting from a computed transform T, creates a message that can be
communicated over the ROS wire. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child."""
def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):


    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)


    """This function is called every time the robot publishes its joint values. We must use
    the information we get to compute forward kinematics.

    We will iterate through the entire chain, and publish the transform for each link we find.
    """
    def callback(self, unordered_joint_values):
        # We will start at the root
        link = self.robot.get_root()
        # This will hold our results, which we will publish at the end
        all_transforms = tf.msg.tfMessage()
        # Prepare structures to hold information
        link_transforms = []
        joints = []
        joint_values = []
        links = []
        # Append the root with a fake fixed joint 
        links.append(link)
        (jn, nl) = self.robot.child_map[link][0]
        root_joint = self.robot.joint_map[jn]
        root_joint.type='fixed'
        joints.append(root_joint)
        joint_values.append(0)
        # Cycle through the joints and collect relevant information
        while True:
            # Find the joint connected at the end of this link, or its "child"
            # Make sure this link has a child
            if link not in self.robot.child_map:
                break
            # Make sure it has a single child (we don't deal with forks)
            if len(self.robot.child_map[link]) != 1:
                rospy.logerror("Forked kinematic chain!")
                break
            # Get the name of the child joint, as well as the link it connects to
            (joint_name, next_link) = self.robot.child_map[link][0]
            # Get the actual joint based on its name
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                break;
            joint = self.robot.joint_map[joint_name]
            # Append link transform to list
            link_transforms.append(joint.origin)
            joints.append(joint)
            # Find the relevant joint axis and value and append to list
            if joint.type != 'fixed' and joint.name in unordered_joint_values.name:
                index = unordered_joint_values.name.index(joint.name)
                joint_values.append(unordered_joint_values.position[index])
            else:
                joint_values.append(0)
            # Append the next link to list
            links.append(next_link)
            # Move to the next link
            link = next_link
        
        # Append one final identity link transform which is not used
        link_transforms.append(link_transforms[-1])
        link_transforms[-1].rpy=(0,0,0)
        link_transforms[-1].xyz=(0,0,0)

        # Compute all transforms
        transforms = self.forward_kinematics(link_transforms, joints, joint_values)

        # Check if the right number if transforms has been returned
        if len(transforms) != len(links):
            print 'Transforms returned: '  + str(len(transforms))
            rospy.logerror("Incorrect number of transforms returned")
            return

        # Convert the result into a message and prepare it to be published        
        for i in range(0,len(transforms)):
            transform_msg = convert_to_message(transforms[i], links[i], "world_origin")
            all_transforms.transforms.append(transform_msg)

        # Publish all the transforms
        self.pub_tf.publish(all_transforms)


    """ This is the core function for our forward kinematics. Given information about all the links 
    and joints of the robot, as well as the current joint angles, it must compute the transform from 
    the base frame to each of the link coordinate frames of the robot.

    Parameters are as follows:

    - "link_transforms" is a vector containing the rigid transform associated with each link of the robot,
    as designed by the manufacturer. link_transforms[i] contains T_L_i, as described in the homework
    handout. The information is encoded as:
     * link_transforms[i].xyz: the translation component of T_L_i
     * link_transforms[i].rpy: the rotation component of T_L_i, in ROLL-PITCH-YAW XYZ convention

    - "joints" is a vector containing information about each of the robot's joints. joints[i] contains
    the following information about the i-th joint of the robot (J_i):
     * joints[i].type: either 'fixed' or 'revolute'. A fixed joint does not move; it is meant to 
       contain a static transform. If a joint is 'fixed', the transform associated with it is the identity. 
       A 'revolute' joint on the  other hand also has a joint angle that changes as the robot moves; 
       this information is contained in the "joint_values" parameter explained below.
     * joints[i].axis: (only if type is 'revolute') the axis of rotation of joint J_i, expressed in the
       coordinate frame associated with the end of the previuos link (or the coordinate frame *after
       processing link L_{i-1})

     - "joint_values" contains the current joint values in the robot. In particular, joint_values[i] 
    contains q_i, the value of the i-th joint of the robot. The joints in this list are in the same order 
    as in the "joints" variable.

    The goal of this function is to return the list of transforms between the base coordinate frame and
    each of the robot's links. You must return this information in the transforms[] list, which must 
    contain the appropriate number of entries. As described in the homework handout, transforms[k] must 
    contain the transform between the base coordinate system and coordinate system {k}.
    """    
    def forward_kinematics(self, link_transforms, joints, joint_values):
        transforms = []

	if(joints[0].type=='fixed'):
		t_ji = tf.transformations.identity_matrix()
	else:
		t_ji = tf.transformations.rotation_matrix(joint_values[0], joints[0].axis)

	transforms.append(t_ji)

	for i in range(1,len(link_transforms)):
		#print link_transforms[i]
		rpy = link_transforms[i-1].rpy
		xyz = link_transforms[i-1].xyz
		T_rot_euler = tf.transformations.euler_matrix(rpy[0], rpy[1], rpy[2], 'rxyz')
		T_trans_yz = tf.transformations.translation_matrix((xyz[0], xyz[1], xyz[2]))
		T_li = numpy.dot(T_trans_yz, T_rot_euler)
		if(joints[i].type=='fixed'):
			t_ji = tf.transformations.identity_matrix()
		else:
			t_ji = tf.transformations.rotation_matrix(joint_values[i], joints[i].axis)
		transforms.append(numpy.dot(numpy.dot(transforms[i-1], T_li),t_ji))

        return transforms
       
if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()
