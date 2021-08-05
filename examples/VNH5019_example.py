# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from logging import DEBUG, ERROR, FATAL, INFO

import pigpio

from VNH5019_driver import VNH5019 as MOTOR

if __name__ == "__main__":

    count = 0.0
    one_count = 360 * 4 / (64 * 50)

    pi = pigpio.pi()

    motor0 = MOTOR(
        pi,
        driver_in1=20,
        driver_in2=21,
        pwm_channel=0,
        logging_level=INFO)

    # motor0.motor_speed(speed=200)
    # time.sleep(5)

    motor0.drive_motor_speed_EX(
        speed=4000,
        drive_time=3,
        KP=0.2,
        KI=1,
        KD=0)

    motor0.free()
    print(motor0.get_current_angle())

    time.sleep(1)

    print("-" * 10)

    motor0.rotate_motor_EX(rotation_angle=-2700, KP=2, KI=0.5, KD=0)

    time.sleep(1)
    print("-" * 10)
    print(motor0.get_current_angle())

    # メモ　I制御を導入して平滑化したい、ゲインにスピードの逆数かけるのやめたい
