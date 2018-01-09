#!/usr/bin/env python
import rospy
import preparethread
import publisher
from threading import Thread
from order_queue import order_queue
import time
from barrieduino.msg import activateOrder # order message to arduino
from barrieduino.msg import ledRing #
from barrieduino.msg import diaphragm #
from DirectorNode.msg import Order # order message from UDPNode

# ID = Drink
# 0 = Koffie
# 1 = Cappuchino
# 1 = Cola
# 2 = Fanta
# 3 =
# Order of prices in database is leading

cold = [ 1, 2, 3 ]
hot = [ 0 ]

hotring = 1
coldring = 2

def order_received_callback(message):
    # check order
    if (message.order_type in cold):
        order_queue.put((message.order_type, coldring))
    elif (message.order_type in hot):
        order_queue.put((message.order_type, hotring))
    else:
        rospy.loginfo("Error unrecognised drink ordered: " + str(message.order_type))

def setup_subscriber():
    rospy.Subscriber('orders', Order, order_received_callback)

if __name__ == '__main__':
    try:
        rospy.init_node('director')
        setup_subscriber()
        pub = publisher.Publisher()
        prepare_thread = preparethread.PrepareThread()
        prepare_thread.daemon = True
        prepare_thread.start()
        rospy.spin()
    except:
        preparethread.join()
        pass
