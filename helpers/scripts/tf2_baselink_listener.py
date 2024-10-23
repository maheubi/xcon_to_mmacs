#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('tf2_baselink_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    bl_pose_pub = rospy.Publisher('transf/bl_in_map', PoseStamped, queue_size=1)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            #rospy.Time(0) should provide the latest transform available according to tf tutorial on listeners.
            trans = tfBuffer.lookup_transform('odom', 'base_fotprint', rospy.Time(0))
            
            bl_pose = PoseStamped()
            
            bl_pose.header.stamp = rospy.Time.now()
            bl_pose.header.frame_id = "baselink_in_map"
            bl_pose.pose.position.x = trans.transform.translation.x
            bl_pose.pose.position.y = trans.transform.translation.y
            bl_pose.pose.position.z = trans.transform.translation.z
            bl_pose.pose.orientation.x = trans.transform.rotation.x
            bl_pose.pose.orientation.y = trans.transform.rotation.y
            bl_pose.pose.orientation.z = trans.transform.rotation.z
            bl_pose.pose.orientation.w = trans.transform.rotation.w

            bl_pose_pub.publish(bl_pose)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        rate.sleep()
