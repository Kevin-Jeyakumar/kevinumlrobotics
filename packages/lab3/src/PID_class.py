#!/usr/bin/env python3

class PID:
    def __init__(self):
        self.kp=1
        self.ki=1
        self.kd=1
        self.past_time_stamp=0
        self.integral=0
        self.prev_error=0

    def __init__(self, p, i, d,time):
        self.kp=p
        self.ki=i
        self.kd=d
        self.past_time_stamp=time
        self.integral=0
        self.prev_error=0

    def changePID(self, p, i, d):
        self.kp=p
        self.ki=i
        seld.kd=d

    def calculateSignal(self, error, time_stamp):
        dt = time_stamp-self.past_time_stamp
        if not dt:
            return 0
        differential = (error-self.prev_error)/dt
        self.integral += error*dt
        new_heading = self.kp*error
        new_heading += self.ki*self.integral
        new_heading += self.kd*differential
        self.past_time_stamp=time_stamp
        self.prev_error=error
        return new_heading
