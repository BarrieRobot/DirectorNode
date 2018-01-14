#!/usr/bin/env python
import time
import threading
import rospy
from barrieduino.msg import ledRing # order message
from barrieduino.msg import HSL # order message

send_delay = 0.5
mode_off = 0
mode_progress = 1
mode_wait = 2

class LEDUpdater (threading.Thread):

    def __init__(self, ring, time):
        threading.Thread.__init__(self)
        self.ring = ring
        self.time = time
        self.publisher = rospy.Publisher('LED_progress', ledRing, queue_size=10)

    def run(self):
        t0 = time.time()
        t1 = time.time()
        while(t1 - t0 < self.time):
            print(((t1-t0) / self.time) * 1000)
            self.publisher.publish(ledRing(self.ring, mode_progress, ((t1-t0) / self.time) * 1000, HSL(100,250,200)))
            time.sleep(send_delay)
            t1 = time.time()
        self.publisher.publish(ledRing(self.ring, mode_wait, 1000, HSL(200,250,200)))
