#!/usr/bin/env python
import time
import rospy
import os
import threading
import director
import publisher
from order_queue import order_queue
from DirectorNode.msg import Order
from barrieduino.msg import ledRing # order message
from barrieduino.msg import HSL # order message

delay_drop_cup = 2
delay_move_to_coffeemachine = 2
delay_move_coffee_diaphragm = 2

class PrepareThread (threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def is_fininshed(self):
        return self.is_fininshed

    def prepare_hot(self):
        print("hot")
        # prepare cup
        publisher.send_drop_cup()
        rospy.loginfo("Dropping cup")
        time.sleep(delay_drop_cup)
        rospy.loginfo("Cup dropped")
        # move to coffee machine


        # dispense coffee
        publisher.dispense_drink(self.drink)
        # move to endstop

        # open diaphragm

    def prepare_cold(self):
        print("cold")
        # dispense can

        # move down transporter

        # move up transporter

        # open diaphragmn


    def prepare_item(self, ( item, ring )):
        print("starts thread for drink " + str(item))
        if (ring is director.hotring):
            self.prepare_hot()
        elif (ring is director.coldring):
            self.prepare_cold()
        time.sleep(10)
        print("finishes thread for drink " + str(item))
        # prepare can or cup
        # fill
        # transport
        # led update
        # open iris
        # reset all

        # if (self.drink in director.hot):
        #     pass
        # elif (self.drink in director.cold):
        #     pass
        # director.order_process_complete()

    def run(self):
        while True:
            if not order_queue.empty():
                item = order_queue.get()
                self.prepare_item(item)
                order_queue.task_done()
            else:
                rospy.loginfo("waiting for order")
                time.sleep(1)
