# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import atexit
from logging import DEBUG, INFO, WARN, Formatter, StreamHandler, getLogger

import pigpio

from PCA9685_wrapper import PWM


class TA7291P:
    def __init__(self, gpio_in1: int, gpio_in2: int, pwm_channel: int):
        """TA7291Pのインスタンスを初期化

        Args:
            gpio_in1 (int): gpioピン1
            gpio_in2 (int): gpioピン2
            pwm_channel (int): PCA9685のチャンネル
        """

        # PWMドライバの初期化
        self.pwm = PWM(pwm_channel)

        # pigpioの初期化
        self.pi = pigpio.pi()

        # pin設定をクラス内変数化
        self.in1 = gpio_in1
        self.in2 = gpio_in2

        # 制御ピンの初期化
        self.pi.set_mode(self.in1, pigpio.OUTPUT)
        self.pi.set_mode(self.in2, pigpio.OUTPUT)
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)

        # loggerの設定
        self.init_logger(WARN)

        # 終了時に全出力を切る
        atexit.register(self.cleanup)

    def init_logger(self, level):
        """ロガーの初期化

        Args:
            level (loggingのレベル): INFO,WARN等で指定
        """

        # ロガーオブジェクト
        self.logger = getLogger(__name__)
        # ログが複数回表示されるのを防止
        self.logger.propagate = False
        # ロガー自体のロギングレベル
        self.logger.setLevel(level)

        # ログを標準出力へ
        handler = StreamHandler()
        # ロギングのフォーマット
        handler.setFormatter(
            Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        # このハンドラのロギングレベル
        handler.setLevel(DEBUG)
        # ロガーにハンドラを追加
        self.logger.addHandler(handler)

    def cleanup(self):
        """インスタンス破棄時に実行、出力を止める
        """
        self.free()
        self.pi.stop()

    def free(self):
        """フリー状態にする
        """
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)
        self.pwm.setPWM(0)

    def brake(self):
        """ブレーキ状態にする
        """
        self.pi.write(self.in1, 1)
        self.pi.write(self.in2, 1)
        self.pwm.setPWM(0)

    def drive(self, pwm_duty_cycle: int):
        """モーターを駆動する変数

        Args:
            pwm_duty_cycle (int): int型で設定、エラーハンドリングはライブラリに任せる
        """
        # 正の値であれば正転に設定
        if pwm_duty_cycle > 0:
            self._forward(pwm_duty_cycle)

        # 負の値であれば逆転に設定
        elif pwm_duty_cycle < 0:
            self._back(pwm_duty_cycle)

        # 0ならフリー状態にする
        else:
            self.free()

    def _forward(self, pwm_duty_cycle: int):
        """正転用の関数

        Args:
            pwm_duty_cycle (int): duty_cycle
        """
        self.pi.write(self.in1, 1)
        self.pi.write(self.in2, 0)
        self.pwm.setPWM(abs(pwm_duty_cycle))

    def _back(self, pwm_duty_cycle: int):
        """逆転用の関数

        Args:
            pwm_duty_cycle (int): duty_cycle
        """
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 1)
        self.pwm.setPWM(abs(pwm_duty_cycle))
