#!/usr/bin/env python
import time
import rospy
import os
import threading
import director
import publisher
from order_queue import order_queue
from director_node.msg import Order
from barrieduino.msg import ledRing # order message
from barrieduino.msg import HSL # order message

delay_drop_cup = 2
delay_drop_can = 2
delay_move_to_coffeemachine = 2
delay_move_coffee_diaphragm = 3
delay_move_soda_diaphragm = 3
delay_move_fuly_down = 1.5
delay_back_to_start1 = 1
delay_back_to_start2 = 3
delay_dispense_coffee = 3
delay_move_to_pre_present = 5
delay_present_through_diaphragm = 3

class PrepareThread (threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.publisher = publisher.Publisher()

    def is_fininshed(self):
        return self.is_fininshed

    def prepare_hot(self, drink):
        # prepare cup
        rospy.loginfo("Dropping cup")
        self.publisher.send_drop_cup()
        time.sleep(delay_drop_cup)
        rospy.loginfo("Cup dropped")
        # move to coffee machine
        rospy.loginfo("Going to coffee machine")
        self.publisher.go_to_coffee_machine()
        time.sleep(delay_move_to_coffeemachine)
        rospy.loginfo("At coffee machine")
        # dispense coffee
        rospy.loginfo("Dispensing drink")
        self.publisher.dispense_drink(drink)
        time.sleep(delay_dispense_coffee)
        rospy.loginfo("Dispensed drink")
        # move to endstop
        rospy.loginfo("Moving to pre-present")
        self.publisher.pre_present(director.hotring)
        time.sleep(delay_move_to_pre_present)
        rospy.loginfo("At pre-present")
        # open diaphragm
        rospy.loginfo("Opening diaphragm")
        self.publisher.open_diaphragm(director.hotring)
        time.sleep(delay_open_diaphragm)
        rospy.loginfo("Opened diaphragm")
        #Move through diaphragm
        rospy.loginfo("Presenting..")
        self.publisher.open_diaphragm(director.hotring)
        time.sleep(delay_present_through_diaphragm)
        rospy.loginfo("Presented")

    def prepare_cold(self, drink):
        # dispense can
        rospy.loginfo("Dropping can")
        self.publisher.dispense_drink(drink)
        time.sleep(delay_drop_can)
        rospy.loginfo("Can dropped")
        # move down transporter
        rospy.loginfo("Locking in can")
        self.publisher.lock_in_soda_can()
        time.sleep(delay_move_fuly_down)
        rospy.loginfo("Can locked in")
        # move up transporter
        rospy.loginfo("Moving to pre-present")
        self.publisher.pre_present(drink)
        time.sleep(delay_move_soda_diaphragm)
        rospy.loginfo("At pre-present")
        # open diaphragmn
        rospy.loginfo("Opening diaphragm")
        self.publisher.open_diaphragm(director.coldring)
        time.sleep(delay_open_diaphragm)
        rospy.loginfo("Opened diaphragm")
        #Move through diaphragm
        rospy.loginfo("Presenting..")
        self.publisher.open_diaphragm(director.coldring)
        time.sleep(delay_present_through_diaphragm)
        rospy.loginfo("Presented")

    def reset_to_starting_position(self, lane):
        rospy.loginfo("Going back to begin position")
        self.publisher.move_to_start(lane)
        time.sleep(delay_back_to_start1)
        self.publisher.close_diaphragm(lane)
        time.sleep(delay_back_to_start2)
        rospy.loginfo("Back at begin position")

    def prepare_item(self, ( item, ring )):
        print("starts thread for drink " + str(item))
        if (ring is director.hotring):
            self.prepare_hot(item)
            reset_to_starting_position(director.hotring)
        elif (ring is director.coldring):
            self.prepare_cold(item)
            reset_to_starting_position(director.coldring)
        # time.sleep(10)
        print("finishes thread for drink " + str(item))
        # prepare can or cup
        # fill
        # transport
        # led update
        # open iris
        # reset all

    def run(self):
        while True:
            if not order_queue.empty():
                item = order_queue.get()
                self.prepare_item(item)
                order_queue.task_done()
            else:
                rospy.loginfo("waiting for order")
                time.sleep(1)
