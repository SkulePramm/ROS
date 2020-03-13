#! /usr/bin/env python

# Assignment 1 for Week1: In this assignment you will subscribe to the topic that
# publishes sensor information. Then, you will transform the sensor reading from
# the reference frame of the sensor to compute the height of a box based on the
# illustration shown in the assignment document. Then, you will publish the box height
# on a new message type ONLY if the height of the box is more than 10cm.

# All necessary python imports go here.
import rospy

#SensorInformation.msg must be defined in CMakeLists.txt
from hrwros_msgs.msg import SensorInformation
from hrwros_msgs.msg import BoxHeightInformation

#Process
def sensor_info_callback(data, bhi_pub):
    rospy.loginfo('Distance reading from the sensor is: %f', data.sensor_data.range)

    # Compute the height of the box.
    box_height = 2.0 - data.sensor_data.range
    rospy.loginfo('box_height: %f', box_height)

    # Boxes that are detected to be shorter than 10cm are due to sensor noise. Don't publish!
    if ((box_height < 0.1) | (box_height > 1.9)):
        rospy.loginfo('NO BOX')
        rospy.loginfo('===========================')
        pass
    else:
        rospy.loginfo('BOX ON BELT')
        rospy.loginfo('===========================')

        #Create message object for publishing the box height information.
        box_height_info = BoxHeightInformation()

        #Update height of box.
        box_height_info.box_height = 2.0 - data.sensor_data.range

        # Publish box height using the publisher argument passed to the callback function.
#        bhi_publisher.publish(bhi_pub)
        bhi_publisher.publish(box_height_info)

#MAIN
if __name__ == '__main__':

    #Create ROSnode (rosnode=compute_box_height)
    rospy.init_node('compute_box_height', anonymous = False)

    #Wait for the topic that publishes sensor information to become available - Part1
    rospy.loginfo('Waiting for topic %s to be published...', 'sensor_info')
    rospy.wait_for_message('sensor_info', SensorInformation)
    rospy.loginfo('%s topic is now available!', 'sensor_info')

    # Create publisher Part3 (Argument bhi_publisher is passed to the subscriber)
    bhi_publisher = rospy.Publisher('box_height_info', BoxHeightInformation, queue_size=10)

    #Create Subscriber (topic=sensor_info)(krijg argument van publisher)
    rospy.Subscriber('sensor_info', SensorInformation, sensor_info_callback, bhi_publisher)

    #Run until Ctrl+C
    rospy.spin()
