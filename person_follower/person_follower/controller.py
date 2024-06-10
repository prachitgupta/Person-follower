#!usr/bin/env python3

import numpy as np
from simple_pid import PID
import math

class Controller():
    def __init__(self):
        # Initialize PID controllers
        self.pid_linear = PID(-0.05, -0.0001, -0.0000, setpoint=0.7)  # target distance Z = 800
        self.pid_angular = PID(0.0050, 0.00001, 0, setpoint=0)  # target horizontal position X = 0
        self.pid_linear.output_limits = (0, 0.31)  # linear speed limits
        self.pid_angular.output_limits = (-0.5, 0.5)  # angular speed limits
        self.epsilon_linear = 0.1
        self.epsilon_angular = 20

    def call_decision_tree(self, X, Z):
        # ICOMITEE 2019 JEMBER (SIMPLE ALGO FOR PERSON FOLLOWING PAPER)
        if  1 <= Z <= 5:
            if X <= -20:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.3  # Turn left
            else:
                if X >= 20:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -0.3  # Turn right
                else:
                    self.twist.linear.x = 0.0  # STOP
                    self.twist.angular.z = 0.0
        else:
            if Z < 1:
                self.twist.linear.x = 0.0  # Move backwards
                self.twist.angular.z = 0.0
            else:
                if X <= -30:
                    pwm_left = (20 + Z / self.alpha) / 100.0
                    pwm_right = max(0.0, (20 + Z / self.alpha - ((X + 320) - 340) * self.K) / 100.0)
                    self.twist.linear.x = (pwm_left + pwm_right) / 2
                    self.twist.angular.z = (pwm_left - pwm_right) / 2  # lft => anti => =+ve
                else:
                    if X >= 30:
                        pwm_right = (20 + Z / self.alpha) / 100.0
                        pwm_left = max(0.0, (20 + Z / self.alpha - (300 - (X + 320)) * self.K) / 100.0)
                        self.twist.linear.x = (pwm_left + pwm_right) / 2
                        self.twist.angular.z = (pwm_left - pwm_right) / 2  # right => clock => -ve
                    else:
                        self.twist.linear.x = (20 + Z / self.alpha) / 100.0  # Move forward
                        self.twist.angular.z = 0.0