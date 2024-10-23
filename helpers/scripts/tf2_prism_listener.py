#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('tf2_prism_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    prism_pose_pub = rospy.Publisher('transf/prism_in_map', PoseStamped, queue_size=1)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            #rospy.Time(0) should provide the latest transform available according to tf tutorial on listeners.
            trans = tfBuffer.lookup_transform('map', 'prism', rospy.Time(0))
            
            prism_pose = PoseStamped()
            
            prism_pose.header.stamp = rospy.Time.now()
            prism_pose.header.frame_id = "prism_in_map"
            prism_pose.pose.position.x = trans.transform.translation.x
            prism_pose.pose.position.y = trans.transform.translation.y
            prism_pose.pose.position.z = trans.transform.translation.z
            prism_pose.pose.orientation.x = trans.transform.rotation.x
            prism_pose.pose.orientation.y = trans.transform.rotation.y
            prism_pose.pose.orientation.z = trans.transform.rotation.z
            prism_pose.pose.orientation.w = trans.transform.rotation.w

            prism_pose_pub.publish(prism_pose)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        rate.sleep()
