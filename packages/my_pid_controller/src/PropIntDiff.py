#!/usr/bin/env python3

Class PID:
    def __init__(self):
        self.kp=1
        self.ki=1
        self.kd=1

    def changePID(self, p, i, d):
        self.kp=p
        self.ki=i
        seld.kd=d

    def calculateSignal(self
