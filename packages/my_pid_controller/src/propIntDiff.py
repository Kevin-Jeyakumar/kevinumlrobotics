#!/usr/bin/env python3

class propIntDiff:
    def __init__(self):
        self.kp=1
        self.ki=1
        self.kd=1
        self.past_time_stamp=0;

    def __init__(self, p, i, d,time):
        self.kp=p
        self.ki=i
        self.kd=d
        self.past_time_stamp=time

    def changePID(self, p, i, d):
        self.kp=p
        self.ki=i
        seld.kd=d

    def calculateSignal(self, error, time_stamp):
        dt = time_stamp-self.past_time_stamp
        e=error
        new_heading += kp*e
        new_heading += ki*e*dt
        new_heading += kd*e/dt
        self.past_time_stamp=time_stamp
        return new_heading
