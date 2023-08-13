#!/usr/bin/env python3  
import rospy
import math
import tf
from geometry_msgs.msg import Quaternion 
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import numpy

if __name__ == '__main__':
    rospy.init_node('correct_tf')

    listener = tf.TransformListener()

    listener.waitForTransform("/map", "/odom", rospy.Time(), rospy.Duration(4.0))
    rate = rospy.Rate(15.0)

    while not rospy.is_shutdown():
        try:
            (transaux2cam,rotaux2cam) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Can't lookup transform")
            continue
        transaux2cam_mat = tf.transformations.translation_matrix(transaux2cam)
        rotaux2cam_mat   = tf.transformations.quaternion_matrix(rotaux2cam)
        tf_aux2cam = numpy.dot(transaux2cam_mat, rotaux2cam_mat)
        tf_cam2aux = tf.transformations.inverse_matrix(tf_aux2cam)

        correct_trans = [0, 0, 0]                               ## Change here if the robot doesn't start from the same world location for consecutive runs
        correct_rot = quaternion_from_euler(0,0,180,'rxyz')       ## Change here if the robot doesn't start from the same world location for consecutive runs
        correct_trans_mat = tf.transformations.translation_matrix(correct_trans)
        correct_rot_mat = tf.transformations.quaternion_matrix(correct_rot)
        tf_correction = numpy.dot(correct_trans_mat, correct_rot_mat)
        tf_world2aux = numpy.dot(tf_correction,tf_cam2aux)

        br = tf.TransformBroadcaster()
        br.sendTransform(tf.transformations.translation_from_matrix(tf_world2aux),
                         tf.transformations.quaternion_from_matrix(tf_world2aux),
                         rospy.Time.now(),
                         "map",
                         "world")
        rate.sleep()