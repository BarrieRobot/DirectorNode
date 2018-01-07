#!/usr/bin/env python
import rospy
from barrieduino.msg import activateOrder # order message
from barrieduino.msg import ledRing # ledring message
from barrieduino.msg import HSL # HSL (color) message
from barrieduino.msg import diaphragm # diaphragm message
from barrieduino.msg import Move

hot = 1
cold = 2

class Publisher():
    def __init__(self):
        self.led_pub = rospy.Publisher('LED_progress', ledRing, queue_size=10)
        self.order_pub = rospy.Publisher('activateOrder', activateOrder, queue_size=10)
        self.diaphragm_pub = rospy.Publisher('diaphragm', activateOrder, queue_size=10)
        self.move_pub = rospy.Publisher('Move', Move, queue_size=10)

    def send_led_progress(self, ring, progress, hsl = (255,255,255)):
        """
        Sends led progress update to fill ledring with appropriate filling
        """
        # uint8 ring, uint8 mode, uint16 param, HSL color
        msg = ledRing()
        msg.ring = ring
        self.led_pub.publish(msg)

    def send_drop_cup(self):
        """
        Sends activateOrder message with order_type=0 to rosserial in order to drop a cup
        """
        order_msg = activateOrder()
        order_msg.order_type = 0 # 0: drop cup
        self.order_pub.publish(order_msg)

    def send_make_drink(self, typ, drink):
        """
        Sends activateOrder message to rosserial to prepare an order
        DEPRECATED - replaced by dispense_drink()
        """
        order_msg = activateOrder()
        order_msg.order_type = typ # assumes ring numbering is same as numbering in activateOrder message!
        order_msg.selection = drink # Undefined!
        self.order_pub.publish(order_msg)

    def dispense_drink(self, drinkid):
        """
        Sends correct message to dispense coffee or a soda based on the drinkid
        (given by order in the database and enum in the GUI)
        """
        order_msg = activateOrder()
        if drinkid in director.cold:
            order_msg.order_type = hot # 1: hot drink
            order_msg.selection = 0 #TODO: TBD
        else:
            order_msg.order_type = cold # 2: cold drink
            order_msg.selection = 0 #TODO: TBD

        self.order_pub.publish(order_msg)

    def go_to_coffee_machine(self):
        """
        Moves cup to the coffee machine
        """
        move_msg = Move()
        move_msg.lane = hot
        move_msg.location = 1 # 1: under coffee machine
        self.move_pub.publish(move_msg)

    def lock_in_soda_can(self):
        """
        Locks soda can in transporter by moving it down to the lowest position
        """
        move_msg = Move()
        move_msg.lane = cold
        move_msg.location = 1 # 1: fully down
        self.move_pub.publish(move_msg)

    def pre_present(self, kind):
        """
        Move transporter (hot or cold) to pre-present location right under diaphragm
        """
        move_msg = Move()
        move_msg.lane = kind
        move_msg.location = 2
        self.move_pub.publish(move_msg)

    def move_to_diaphragm(self):
        pass
