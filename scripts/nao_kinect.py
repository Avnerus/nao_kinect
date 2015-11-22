#!/usr/bin/env python
import rospy
import tf.msg
import naoqi_bridge_msgs.msg

PREFIX = '/tracker/user_1'

JOINT_MAP = {
   PREFIX + '/right_hand' : 'RShoulderPitch',
   PREFIX + '/left_hand' : 'LShoulderPitch'
}

pub = rospy.Publisher('/joint_angles', naoqi_bridge_msgs.msg.JointAnglesWithSpeed, queue_size=10)

def callback(data):
    tf = data.transforms[0]
    if tf.header.frame_id == "/tracker_depth_frame":
       if tf.child_frame_id in JOINT_MAP:
          nao_joint = JOINT_MAP[tf.child_frame_id]
	  message = naoqi_bridge_msgs.msg.JointAnglesWithSpeed()
	  message.header.frame_id = 'nao_kinect'
	  message.header.seq = 200
	  message.joint_names = [nao_joint]
	  message.joint_angles = [0]

          '{header: {frame_id: "test", seq: 200}, joint_names: ["LAnklePitch", "RAnklePitch"], joint_angles: [0, 0], speed: 1}'
	  rospy.loginfo("%s: %s", nao_joint,  tf.transform.translation )

def nao_kinect():
    rospy.init_node('nao_kinect', anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + ": Starting NAO Kinect Controller")
    rospy.Subscriber("/tf", tf.msg.tfMessage, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    nao_kinect()

