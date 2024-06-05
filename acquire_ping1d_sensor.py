#!/usr/bin/env python
"""
Author: Ben Marsh
        b.marsh@aims.gov.au

Description:
  ROS Node to read the sonar information from the sensor from Jetson UART and publish the message.
  
"""



import rospy
import time

# Import the Blue Robotics message type.
from br_msgs.msg import ping1d_distance_simple
from brping import Ping1D

TOPIC_PING_DEPTH = "/ping_depth"

class ReefscanAcquireSonar(object):
    # Function:		__init__(self)
    # Description: 	Create ROS Subscriber and Publisher
  
    def __init__(self):
        #self.sub_ping_depth = rospy.Subscriber(TOPIC_PING_DEPTH, ping1d_distance_simple,
        #                                       self.subscriber_ping_depth_callback)
        #self.publisher = rospy.Publish
        self.error_counter = 0
        self.msg_ping_sensor = ping1d_distance_simple()
        # use 0 no signify errorneous data
        self.msg_ping_sensor.distance = 0
        self.msg_ping_sensor.confidence = 0

        self.ping_sensor = Ping1D()
        device = '/dev/ttyTHS1'
        baudrate = 115200
        self.ping_sensor.connect_serial(device, baudrate)
      
        if self.ping_sensor.initialize() is False:
            print("Failed to initialize Ping!")
            exit(1)
        time.sleep(0.5)    # delay for pinger to initialise
        self.pub_sonar_message = rospy.Publisher(TOPIC_PING_DEPTH, ping1d_distance_simple , queue_size=1)
        self.timer_read_sensor = rospy.Timer(rospy.Duration(1.0/10.0), self.read_sensor)
      
    def read_sensor(self, other):
        depth_ping_data = self.ping_sensor.get_distance()
        print(depth_ping_data)
        if depth_ping_data:
            self.msg_ping_sensor.distance = int(depth_ping_data["distance"])
            self.msg_ping_sensor.confidence = int(depth_ping_data["confidence"])
            self.error_counter = 0
        else:
            print("no ping data")
            self.msg_ping_sensor.distance = 0
            self.msg_ping_sensor.confidence = 0 
            self.error_counter += 1
  
        # if there is an error reading the sensor we can resend the previous data
        # but if the error persists for over 2 seconds (20 misreads) set the data to 0
        if self.error_counter > 20:
            self.msg_ping_sensor.distance = 0
            self.msg_ping_sensor.confidence = 9999
  
        self.pub_sonar_message.publish(self.msg_ping_sensor)
        try:
            pass
            # print("ping published")
            # msg_error.data = error_code
            # self.node_handle.publish("ping_depth_error_codes", msg_error)
        except:
            print("can't send ping message")
  
if __name__ == '__main__':
    # Initialise node with rospy
    rospy.init_node('reefscan_acquire_sonar', anonymous=True)


    reefscan_acquire_sonar = ReefscanAcquireSonar()

    # Go into the spin() loop so rospy can
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down") 
