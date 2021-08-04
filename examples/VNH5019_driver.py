# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import atexit
from datetime import datetime, timedelta
from rotary_encoder import RotaryEncoder
from logging import DEBUG, INFO, WARN, Formatter, StreamHandler, getLogger

import pigpio
from PCA9685_wrapper import PWM


class VNH5019:
    def __init__(
            self,
            pi: pigpio.pi,
            driver_in1: int,
            driver_in2: int,
            pwm_channel: int,
            logging_level: int = INFO):
        """TA7291Pのインスタンスを初期化

        Args:
            pi (pigpio.pi): pigpioインスタンス
            driver_in1 (int): モータードライバのIN1
            driver_in2 (int): モータードライバのIN2
            pwm_channel (int): PCA9685のチャンネル
            logging_level (int): ロガーのレベル
        """

        # PWMドライバの初期化
        self.pwm = PWM(pwm_channel, freq=1000)

        # pigpioの初期化
        self.pi = pi

        # pin設定をクラス内変数化
        self.in1 = driver_in1
        self.in2 = driver_in2

        # 制御ピンの初期化
        self.pi.set_mode(self.in1, pigpio.OUTPUT)
        self.pi.set_mode(self.in2, pigpio.OUTPUT)
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)

        # loggerの設定
        self.init_logger(logging_level)

        # ロータリーエンコーダ用変数
        self.count = 0.0
        self.prev_current_time = datetime.now()
        self.one_count = 360 * 4 / (64 * 50)
        self.rotation_speed = 0.0
        self.print_counter = 0
        self.pos = 0.0

        # PID制御用変数
        self.P_gain = 0.5

        # ロータリーエンコーダの初期化
        RotaryEncoder(pi, 6, 5, self.callback)

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

    def callback(self, way):
        current_time = datetime.now()

        self.count += way

        self.pos = self.count * self.one_count
        eplased_time = (current_time - self.prev_current_time) / \
            timedelta(seconds=1)
        self.rotation_speed = self.one_count / eplased_time

        self.logger.debug(f"pos={self.pos:.0f}")
        self.logger.debug(f"rotation_speed={self.rotation_speed:.0f}")

        self.prev_current_time = current_time

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

    def drive_until(self, pwm_duty_cycle: int, drive_time: float):
        """一定の秒数モーターを駆動する変数

        Args:
            pwm_duty_cycle (int): int型で設定、エラーハンドリングはライブラリに任せる
            drive_time (float): 駆動秒数(単位は秒)
        """

        drive_time = drive_time * 1000 * 1000

        start_time = datetime.now()
        while True:
            driven_time = (datetime.now() - start_time) / \
                timedelta(microseconds=1)
            self.drive(pwm_duty_cycle)

            if driven_time > drive_time:
                break

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

    def get_current_pos(self) -> float:
        """現在の角度を返す

        Returns:
            float: 今の角度
        """
        return self.pos

    def get_rotation_speed(self) -> float:
        """現在の回転速度を返す

        Returns:
            float: 今の回転速度
        """
        return self.rotation_speed

    def reset_pos(self):
        """角度を初期化する
        """
        self.pos = 0
        self.count = 0

    def motor_speed(self, speed: float):
        """モーターを特定速度で回転させる関数

        Args:
            speed (float): 回転速度(degree/s)
        """
        current_speed = self.get_rotation_speed()

        raw_gain = self.P_gain * (current_speed - speed)
        duty_cycle = raw_gain * 500
        duty_cycle = int(duty_cycle)

        if duty_cycle > 4095:
            duty_cycle = 4095
        elif duty_cycle < -4095:
            duty_cycle = -4095

        if self.print_counter % 50 == 0:
            print(f"diff={(speed - current_speed):.0f}")
            print(f"duty_cycle={duty_cycle}")
            print(f"speed={current_speed:.0f}")

        self.print_counter += 1

        self.drive(pwm_duty_cycle=duty_cycle)

    def drive_motor_speed(self, speed: float, drive_time: float):
        drive_time = drive_time * 1000 * 1000

        start_time = datetime.now()
        while True:
            driven_time = (datetime.now() - start_time) / \
                timedelta(microseconds=1)

            self.motor_speed(speed=speed)

            if driven_time > drive_time:
                self.brake()
                break

    def rotate_motor(self, pwm_duty_cycle: int, rotation_angle: float):
        """モーターを指定角度まで指定のduty比で回転させる

        Args:
            pwm_duty_cycle (int): duty比
            rotation_angle (float): 目標角度
        """

        while True:
            current_angle = self.get_current_pos()

            diff = rotation_angle - current_angle

            if diff > 0:
                self._forward(pwm_duty_cycle)
            else:
                self._back(pwm_duty_cycle)

            self.logger.debug(current_angle)

            if abs(diff) < self.one_count:
                self.free()
                break

        cnt = 0
        while True:
            current_angle = self.get_current_pos()

            diff = rotation_angle - current_angle

            if diff > 0:
                self._forward(600)
            else:
                self._back(600)

            self.logger.debug(current_angle)

            if abs(diff) < self.one_count:
                cnt += 1
                if cnt == 10:
                    self.brake()
                    break

    def rotate_motor_PID(self, pwm_duty_cycle: int, rotation_angle: float):
        """モーターをPID制御で指定角度まで指定のduty比で回転させる

        Args:
            pwm_duty_cycle (int): duty比
            rotation_angle (float): 目標角度
        """

        while True:
            current_angle = self.get_current_pos()

            diff = rotation_angle - current_angle

            pow = abs(diff) * 50
            pow = int(pow)
            if pow < 600:
                pow = 600
            elif pow > 4095:
                pow = 4095

            print(pow)

            if diff > 0:
                self._forward(pow)
            else:
                self._back(pow)

            self.logger.debug(current_angle)

            if abs(diff) < self.one_count:
                self.free()
                break

        cnt = 0
        while True:
            current_angle = self.get_current_pos()

            diff = rotation_angle - current_angle

            if diff > 0:
                self._forward(600)
            else:
                self._back(600)

            self.logger.debug(current_angle)

            if abs(diff) < self.one_count:
                cnt += 1
                if cnt == 10:
                    self.brake()
                    break
