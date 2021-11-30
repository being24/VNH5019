# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from logging import DEBUG, ERROR, FATAL, INFO, WARN

import pigpio

from VNH5019_driver import VNH5019 as MOTOR

if __name__ == "__main__":

    count = 0.0
    one_count = 360 * 4 / (64 * 50)

    pi = pigpio.pi()

    motor0 = MOTOR(
        pi,
        driver_out1=20,
        driver_out2=21,
        encoder_in1=5,
        encoder_in2=6,
        pwm_channel=0,
        gear_ratio=150,
        logging_level=WARN)

    motor1 = MOTOR(
        pi,
        driver_out1=23,
        driver_out2=24,
        encoder_in1=27,
        encoder_in2=22,
        pwm_channel=1,
        gear_ratio=50,
        logging_level=WARN)

    time.sleep(3)

    motor0.rotate_motor(pwm_duty_cycle=500, rotation_angle=180)
    motor1.rotate_motor(pwm_duty_cycle=500, rotation_angle=180)
    #motor0.drive(pwm_duty_cycle=4095)
    #motor1.drive(pwm_duty_cycle=4095)

    time.sleep(3)
    print("-" * 10)
    print(motor0.get_current_angle())
    print(motor1.get_current_angle())

    # メモ I制御を導入して平滑化したい、ゲインにスピードの逆数かけるのやめたい
