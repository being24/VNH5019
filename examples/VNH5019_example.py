# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from datetime import datetime
from logging import DEBUG, ERROR, FATAL

import pigpio

from VNH5019_driver import VNH5019 as MOTOR

if __name__ == "__main__":

    count = 0.0
    prev_current_time = datetime.now()
    one_count = 360 * 4 / (64 * 50)

    pi = pigpio.pi()

    motor0 = MOTOR(pi, driver_in1=20, driver_in2=21, pwm_channel=0)

    motor0.rotate_motor_PID(pwm_duty_cycle=4000, rotation_angle=720)

    time.sleep(1)
    print(motor0.get_current_pos())
