# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from logging import DEBUG, ERROR, FATAL

import pigpio

from VNH5019_driver import VNH5019 as MOTOR

if __name__ == "__main__":

    pi = pigpio.pi()

    motor0 = MOTOR(pi, driver_in1=20, driver_in2=21, pwm_channel=0, logging_level=DEBUG)

    motor0.free()

